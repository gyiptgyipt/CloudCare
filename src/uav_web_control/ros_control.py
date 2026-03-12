"""ROS2 control node for uav_web_control.

Provides a Python port of the C++ Control class behavior in
`px4_swarm_controller/libs/Control/Control.cpp`.

Exports helper functions `arm_uav(uav_idx)` and `fly_uav(uav_idx, pos)`
which the Flask app can call to command specific drones.
"""
from pathlib import Path
import threading
import yaml
import math
import logging

log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

try:
    import rclpy
    from rclpy.node import Node
    from px4_msgs.msg import (
        VehicleCommand,
        TrajectorySetpoint,
        OffboardControlMode,
        VehicleLocalPosition,
    )
except Exception as e:
    log.error('rclpy or px4_msgs not available: %s', e)
    raise


class WebControl(Node):
    def __init__(self):
        super().__init__('web_control')
        pkg_dir = Path(__file__).parent
        # prefer config/config.yaml to uavs.yaml if present
        cfg_path = pkg_dir / 'config' / 'config.yaml'
        if not cfg_path.exists():
            cfg_path = pkg_dir / 'config' / 'uavs.yaml'

        log.info('Loading control config from %s', cfg_path)
        cfg = yaml.safe_load(cfg_path.read_text())

        self.num_drones = int(cfg.get('num_drones', 0))

        # setpoints: dict of string id -> list of [x,y,z,yaw]
        self.setpoints = {}
        sp_node = cfg.get('setpoints', {})
        for k, v in sp_node.items():
            self.setpoints[int(k)] = [list(map(float, s)) for s in v]

        # local positions
        self.local_positions = {i: [0.0, 0.0, 0.0] for i in range(1, self.num_drones + 1)}

        # publishers and subscribers
        self.offboard_pubs = {}
        self.traj_pubs = {}
        self.cmd_pubs = {}

        self.local_subs = {}

        # create publishers/subscribers for each drone
        for i in range(1, self.num_drones + 1):
            self.offboard_pubs[i] = self.create_publisher(OffboardControlMode, f'/px4_{i}/fmu/in/offboard_control_mode', 10)
            self.traj_pubs[i] = self.create_publisher(TrajectorySetpoint, f'/px4_{i}/fmu/in/trajectory_setpoint', 10)
            self.cmd_pubs[i] = self.create_publisher(VehicleCommand, f'/px4_{i}/fmu/in/vehicle_command', 10)

            # subscriber for local position
            def make_cb(idx):
                def cb(msg):
                    self.local_positions[idx] = [msg.x, msg.y, msg.z]

                return cb

            self.local_subs[i] = self.create_subscription(
                VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position', make_cb(i), 10
            )

        # state for setpoint progression
        self.reached = {i: False for i in range(1, self.num_drones + 1)}
        # assume all drones share same number of setpoints
        self.num_setpoints = len(next(iter(self.setpoints.values()))) if self.setpoints else 0
        self.setpoint_index = 0
        self.offboard_count = 0

        # timer at 10 Hz
        self.create_timer(0.1, self._timer_cb)

        log.info('WebControl initialized for %d drones', self.num_drones)

    def _timer_cb(self):
        # publish offboard control mode and trajectory setpoints
        for i in range(1, self.num_drones + 1):
            self._publish_offboard_mode(i)

            # until we have sent enough setpoints, publish an altitude hold
            if self.offboard_count < 50:
                self._publish_trajectory(i, 0.0, 0.0, -5.0, 0.0)
            else:
                # publish configured setpoint
                if self.setpoints and i in self.setpoints:
                    sp = self.setpoints[i][self.setpoint_index]
                    self._publish_trajectory(i, sp[0], sp[1], sp[2], sp[3] if len(sp) > 3 else 0.0)

        if self.offboard_count < 51:
            self.offboard_count += 1

        # detect reached setpoints
        if self.setpoints:
            all_reached = True
            for i in range(1, self.num_drones + 1):
                sp = self.setpoints[i][self.setpoint_index]
                dist = math.sqrt(sum((self.local_positions[i][j] - sp[j]) ** 2 for j in range(3)))
                if dist < 0.3 and not self.reached[i]:
                    self.reached[i] = True
                if not self.reached[i]:
                    all_reached = False

            if all_reached:
                self.setpoint_index = min(self.setpoint_index + 1, self.num_setpoints - 1)
                for i in range(1, self.num_drones + 1):
                    self.reached[i] = False
                log.info('Advanced to setpoint index %d', self.setpoint_index)

    def _publish_offboard_mode(self, idx):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pubs[idx].publish(msg)

    def _publish_trajectory(self, idx, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pubs[idx].publish(msg)

    def _publish_vehicle_command(self, idx, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pubs[idx].publish(msg)

    # helpers exposed to Flask
    def arm(self, idx):
        # send arm command (VEHICLE_CMD_COMPONENT_ARM_DISARM = 400)
        self._publish_vehicle_command(idx, 400, param1=1.0)
        log.info('arm() sent to %d', idx)

    def offboard(self, idx):
        # set offboard mode
        # VEHICLE_CMD_DO_SET_MODE = 176 (in PX4) but existing C++ used a combo; we'll send DO_SET_MODE=176 param1:1 param2:6
        self._publish_vehicle_command(idx, 176, param1=1.0, param2=6.0)
        log.info('offboard() sent to %d', idx)

    def fly(self, idx, position):
        # position: [x,y,z] or similar
        if not (isinstance(position, (list, tuple)) and len(position) >= 3):
            raise ValueError('position must be [x,y,z]')
        self._publish_trajectory(idx, position[0], position[1], position[2], position[3] if len(position) > 3 else 0.0)
        log.info('fly() sent to %d -> %s', idx, position)


# Create a singleton controller instance and spin in background
_controller = None
_executor = None

def start_controller():
    global _controller, _executor
    if _controller is not None:
        return _controller

    rclpy.init()
    _controller = WebControl()

    from rclpy.executors import SingleThreadedExecutor

    _executor = SingleThreadedExecutor()
    _executor.add_node(_controller)

    def _spin():
        try:
            _executor.spin()
        except Exception:
            pass

    t = threading.Thread(target=_spin, daemon=True)
    t.start()
    log.info('WebControl node started')
    return _controller


def arm_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    c.arm(idx)


def fly_uav(uav_idx, position):
    c = start_controller()
    idx = int(uav_idx)
    c.fly(idx, position)
