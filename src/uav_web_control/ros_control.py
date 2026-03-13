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
import time

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

        # Prefer the px4_swarm_controller config if available in the workspace
        px4_cfg_candidate = Path(__file__).resolve().parents[1] / 'px4_swarm_controller' / 'config' / 'config.yaml'
        px4_cfg_candidate2 = Path(__file__).resolve().parents[1] / 'px4_swarm_controller' / 'config' / 'drones.yaml'

        # local package config (fallback)
        local_cfg1 = pkg_dir / 'config' / 'config.yaml'
        local_cfg2 = pkg_dir / 'config' / 'uavs.yaml'

        cfg_path = None
        if px4_cfg_candidate.exists():
            cfg_path = px4_cfg_candidate
        elif px4_cfg_candidate2.exists():
            cfg_path = px4_cfg_candidate2
        elif local_cfg1.exists():
            cfg_path = local_cfg1
        elif local_cfg2.exists():
            cfg_path = local_cfg2

        if cfg_path is None:
            log.warning('No config file found for ros_control; defaulting to zero drones')
            cfg = {}
        else:
            log.info('Loading control config from %s', cfg_path)
            try:
                cfg = yaml.safe_load(cfg_path.read_text()) or {}
            except Exception:
                log.exception('Failed to parse config at %s', cfg_path)
                cfg = {}

        # num_drones may be stored under different keys in various config formats
        self.num_drones = int(cfg.get('num_drones', cfg.get('n_drones', 0)))

        # publishing parameters: allow configuration of message rate (Hz)
        # and the initial setpoint time window (seconds) during which we
        # publish default altitude hold setpoints before arming/offboard.
        self.publish_hz = int(cfg.get('publish_hz', 10))
        self.initial_setpoint_time_s = float(cfg.get('initial_setpoint_time_s', 5.0))
        # number of timer iterations to send initial setpoints
        self.initial_setpoint_count = max(1, int(self.publish_hz * self.initial_setpoint_time_s))

        # setpoints: dict of string id -> list of [x,y,z,yaw]
        self.setpoints = {}
        sp_node = cfg.get('setpoints', {})
        for k, v in sp_node.items():
            self.setpoints[int(k)] = [list(map(float, s)) for s in v]

        # local positions
        self.local_positions = {i: [0.0, 0.0, 0.0] for i in range(1, self.num_drones + 1)}

        # manual target storage: when a Fly button is pressed, we store the
        # requested position here and publish it for a short duration so PX4
        # receives a stream of setpoints for that drone.
        self.manual_targets = {i: None for i in range(1, self.num_drones + 1)}
        self.manual_target_counters = {i: 0 for i in range(1, self.num_drones + 1)}
        # duration to publish manual target (seconds)
        # use a longer default so PX4 has time to accept the streamed setpoints
        self.manual_target_duration_s = float(cfg.get('manual_target_duration_s', 8.0))
        # armed state per drone
        self.armed = {i: False for i in range(1, self.num_drones + 1)}
        # publishers and subscribers
        self.offboard_pubs = {}
        self.traj_pubs = {}
        self.cmd_pubs = {}

        self.local_subs = {}

        # create publishers/subscribers for each drone
        log.info('Creating publishers/subscribers for %d drones', self.num_drones)
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
            log.info('Created publishers for px4_%d: offboard=%s traj=%s cmd=%s', i, f'/px4_{i}/fmu/in/offboard_control_mode', f'/px4_{i}/fmu/in/trajectory_setpoint', f'/px4_{i}/fmu/in/vehicle_command')

        # state for setpoint progression
        self.reached = {i: False for i in range(1, self.num_drones + 1)}
        # assume all drones share same number of setpoints
        self.num_setpoints = len(next(iter(self.setpoints.values()))) if self.setpoints else 0
        self.setpoint_index = 0
        self.offboard_count = 0
        # flag to indicate we've sent arm/offboard commands after initial setpoints
        self.offboard_arm_sent = False

        # timer at configured rate
        timer_period = 1.0 / float(self.publish_hz)
        self.create_timer(timer_period, self._timer_cb)

        log.info('WebControl initialized for %d drones', self.num_drones)

    def _timer_cb(self):
        # publish offboard control mode and trajectory setpoints
        for i in range(1, self.num_drones + 1):
            self._publish_offboard_mode(i)

            # until we have sent enough initial setpoints, publish an altitude hold
            if self.offboard_count < self.initial_setpoint_count:
                self._publish_trajectory(i, 0.0, 0.0, -5.0, 0.0)
            else:
                # if a manual fly target is active for this drone, publish it
                if self.manual_target_counters.get(i, 0) > 0 and self.manual_targets.get(i) is not None:
                    mt = self.manual_targets[i]
                    self._publish_trajectory(i, mt[0], mt[1], mt[2], mt[3] if len(mt) > 3 else 0.0)
                    # decrement counter
                    self.manual_target_counters[i] = max(0, self.manual_target_counters[i] - 1)
                    if self.manual_target_counters[i] == 0:
                        # persist manual target as the current configured setpoint
                        mt = self.manual_targets[i]
                        try:
                            if self.setpoints and i in self.setpoints and 0 <= self.setpoint_index < len(self.setpoints[i]):
                                self.setpoints[i][self.setpoint_index] = [float(mt[0]), float(mt[1]), float(mt[2])] + ([float(mt[3])] if len(mt) > 3 else [0.0])
                        except Exception:
                            log.exception('failed to persist manual target for %d', i)
                        self.manual_targets[i] = None
                else:
                    # publish configured setpoint
                    if self.setpoints and i in self.setpoints:
                        sp = self.setpoints[i][self.setpoint_index]
                        self._publish_trajectory(i, sp[0], sp[1], sp[2], sp[3] if len(sp) > 3 else 0.0)

        # Do NOT auto-arm all drones. We only publish initial setpoints to
        # establish the offboard stream. Actual arming/offboard switching will
        # be triggered per-drone via the `arm()` helper invoked from Flask.
        if self.offboard_count < (self.initial_setpoint_count + 1):
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
        # ensure integer index
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for vehicle command')

        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        # match C++ behavior: use 0 for target_system (broadcast)
        msg.target_system = 0
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        pub = self.cmd_pubs.get(i)
        if pub is None:
            raise KeyError(f'No vehicle_command publisher for idx {i}')

        try:
            pub.publish(msg)
        except Exception:
            log.exception('publish vehicle_command failed for idx %s', i)
            raise

    # helpers exposed to Flask
    def arm(self, idx):
        # send arm command (VEHICLE_CMD_COMPONENT_ARM_DISARM = 400) for a
        # specific drone and then request offboard mode. We repeat the
        # commands several times (as in C++) to increase chance of reception.
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for arm')

        # wait briefly for PX4 to subscribe to our topics if needed
        pub = self.cmd_pubs.get(i)
        traj_pub = self.traj_pubs.get(i)
        wait_timeout = 5.0
        waited = 0.0
        wait_step = 0.1
        subs = 0
        while waited < wait_timeout:
            try:
                subs = 0
                if pub is not None:
                    subs += int(pub.get_subscription_count())
                if traj_pub is not None:
                    subs += int(traj_pub.get_subscription_count())
            except Exception:
                subs = 0
            if subs > 0:
                break
            time.sleep(wait_step)
            waited += wait_step

        if subs == 0:
            log.warning('No PX4 subscribers detected for px4_%d (waited %.1fs) — arm may fail', i, waited)

        counter = 0
        repeat = 5
        while counter < repeat:
            try:
                self._publish_vehicle_command(i, 400, param1=1.0)
                # set offboard mode: VEHICLE_CMD_DO_SET_MODE = 176, params as in C++
                self._publish_vehicle_command(i, 176, param1=1.0, param2=6.0)
            except Exception:
                log.exception('failed sending arm/offboard to %d (attempt %d)', i, counter + 1)
            counter += 1

        log.info('arm() sent to %d (repeated %d times)', i, repeat)
        # mark as armed locally (informational)
        self.armed[i] = True

    def offboard(self, idx):
        # set offboard mode
        # VEHICLE_CMD_DO_SET_MODE = 176 (in PX4) but existing C++ used a combo; we'll send DO_SET_MODE=176 param1:1 param2:6
        self._publish_vehicle_command(idx, 176, param1=1.0, param2=6.0)
        log.info('offboard() sent to %d', idx)

    def fly(self, idx, position):
        # position: [x,y,z] or similar
        if not (isinstance(position, (list, tuple)) and len(position) >= 3):
            raise ValueError('position must be [x,y,z]')
        # Store the manual target and request that the timer publish it for a
        # short duration to create a stream of setpoints for PX4.
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for fly')

        self.manual_targets[i] = [float(position[0]), float(position[1]), float(position[2])] + ([float(position[3])] if len(position) > 3 else [])
        self.manual_target_counters[i] = int(self.publish_hz * self.manual_target_duration_s)
        log.info('fly() queued for %d -> %s (publishing for %d cycles)', i, self.manual_targets[i], self.manual_target_counters[i])

    def get_uav_status(self):
        """Return status info per UAV: whether publishers have subscribers.

        Returns dict indexed by drone index (int) with keys:
          - 'cmd_subscribers': number of subscribers on vehicle_command
          - 'traj_subscribers': number of subscribers on trajectory_setpoint
          - 'connected': bool (true if any subscriber present)
        """
        status = {}
        for i in range(1, self.num_drones + 1):
            cmd_pub = self.cmd_pubs.get(i)
            traj_pub = self.traj_pubs.get(i)
            try:
                cmd_count = cmd_pub.get_subscription_count() if cmd_pub is not None else 0
            except Exception:
                cmd_count = 0
            try:
                traj_count = traj_pub.get_subscription_count() if traj_pub is not None else 0
            except Exception:
                traj_count = 0
            status[i] = {
                'cmd_subscribers': int(cmd_count),
                'traj_subscribers': int(traj_count),
                'connected': (int(cmd_count) + int(traj_count)) > 0,
                'armed': bool(self.armed.get(i, False)),
                'manual_target_active': bool(self.manual_targets.get(i) is not None),
                'manual_target_cycles': int(self.manual_target_counters.get(i, 0)),
                'local_position': list(self.local_positions.get(i, [0.0, 0.0, 0.0])),
            }
        return status


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
