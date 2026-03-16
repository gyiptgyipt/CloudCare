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
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from px4_msgs.msg import (
        VehicleCommand,
        TrajectorySetpoint,
        OffboardControlMode,
        VehicleLocalPosition,
        # optional status messages
        ActuatorArmed,
        VehicleStatus,
        PositionSetpoint,
    )
    try:
        from px4_msgs.msg import VehicleGlobalPosition
        HAS_GLOBAL_POS = True
    except Exception:
        VehicleGlobalPosition = None
        HAS_GLOBAL_POS = False
    try:
        from px4_msgs.msg import VehicleControlMode
        HAS_VEHICLE_CONTROL_MODE = True
    except Exception:
        VehicleControlMode = None
        HAS_VEHICLE_CONTROL_MODE = False
    try:
        from px4_msgs.msg import BatteryStatus
        HAS_BATTERY_STATUS = True
    except Exception:
        BatteryStatus = None
        HAS_BATTERY_STATUS = False
    try:
        from px4_msgs.msg import VehicleGpsPosition
        HAS_VEHICLE_GPS_POSITION = True
    except Exception:
        VehicleGpsPosition = None
        HAS_VEHICLE_GPS_POSITION = False
    try:
        from px4_msgs.msg import VehicleLandDetected
        HAS_VEHICLE_LAND_DETECTED = True
    except Exception:
        VehicleLandDetected = None
        HAS_VEHICLE_LAND_DETECTED = False
    try:
        from px4_msgs.msg import TakeoffStatus
        HAS_TAKEOFF_STATUS = True
    except Exception:
        TakeoffStatus = None
        HAS_TAKEOFF_STATUS = False
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
        # global positions (lat, lon, alt)
        self.global_positions = {i: None for i in range(1, self.num_drones + 1)}
        self.home_positions = {i: None for i in range(1, self.num_drones + 1)}
        # extra status info
        self.vehicle_status = {i: None for i in range(1, self.num_drones + 1)}
        self.vehicle_control_mode = {i: None for i in range(1, self.num_drones + 1)}
        self.battery_status = {i: None for i in range(1, self.num_drones + 1)}
        self.vehicle_gps_position = {i: None for i in range(1, self.num_drones + 1)}
        self.vehicle_land_detected = {i: None for i in range(1, self.num_drones + 1)}
        self.takeoff_status = {i: None for i in range(1, self.num_drones + 1)}

        # manual target storage: when a Fly button is pressed, we store the
        # requested position here and publish it for a short duration so PX4
        # receives a stream of setpoints for that drone.
        self.manual_targets = {i: None for i in range(1, self.num_drones + 1)}
        self.manual_target_counters = {i: 0 for i in range(1, self.num_drones + 1)}
        # teleop velocity command (vx, vy, vz) and expiry
        self.teleop_cmd = {i: [0.0, 0.0, 0.0] for i in range(1, self.num_drones + 1)}
        self.teleop_until = {i: 0.0 for i in range(1, self.num_drones + 1)}
        self.manual_mode = {i: False for i in range(1, self.num_drones + 1)}
        # duration to publish manual target (seconds)
        # use a longer default so PX4 has time to accept the streamed setpoints
        self.manual_target_duration_s = float(cfg.get('manual_target_duration_s', 8.0))
        # armed state per drone
        self.armed = {i: False for i in range(1, self.num_drones + 1)}
        # publishers and subscribers
        self.offboard_pubs = {}
        self.traj_pubs = {}
        self.cmd_pubs = {}
        self.pos_pubs = {}

        self.local_subs = {}
        self.global_subs = {}

        # create publishers/subscribers for each drone
        log.info('Creating publishers/subscribers for %d drones', self.num_drones)
        for i in range(1, self.num_drones + 1):
            self.offboard_pubs[i] = self.create_publisher(OffboardControlMode, f'/px4_{i}/fmu/in/offboard_control_mode', 10)
            self.traj_pubs[i] = self.create_publisher(TrajectorySetpoint, f'/px4_{i}/fmu/in/trajectory_setpoint', 10)
            # publisher for global position setpoints (WGS84)
            try:
                self.pos_pubs[i] = self.create_publisher(PositionSetpoint, f'/px4_{i}/fmu/in/position_setpoint', 10)
            except Exception:
                self.pos_pubs[i] = None
            self.cmd_pubs[i] = self.create_publisher(VehicleCommand, f'/px4_{i}/fmu/in/vehicle_command', 10)

            # subscriber for local position
            def make_cb(idx):
                def cb(msg):
                    self.local_positions[idx] = [msg.x, msg.y, msg.z]

                return cb

            self.local_subs[i] = self.create_subscription(
                VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position', make_cb(i), 10
            )

            # optional global position (GPS)
            if HAS_GLOBAL_POS and VehicleGlobalPosition is not None:
                qos = QoSProfile(depth=1)
                qos.reliability = ReliabilityPolicy.BEST_EFFORT
                qos.durability = DurabilityPolicy.VOLATILE
                qos.history = HistoryPolicy.KEEP_LAST
                def make_gcb(idx):
                    def gcb(msg):
                        try:
                            lat = float(msg.lat)
                            lon = float(msg.lon)
                            alt = float(getattr(msg, 'alt', 0.0))
                            # heuristic: PX4 VehicleGlobalPosition uses 1e7 scaled degrees
                            if abs(lat) > 180.0 or abs(lon) > 180.0:
                                lat = lat / 1e7
                                lon = lon / 1e7
                            self.global_positions[idx] = [lat, lon, alt]
                            if self.home_positions.get(idx) is None:
                                self.home_positions[idx] = [lat, lon, alt]
                        except Exception:
                            pass
                    return gcb

                self.global_subs[i] = self.create_subscription(
                    VehicleGlobalPosition, f'/px4_{i}/fmu/out/vehicle_global_position', make_gcb(i), qos
                )

            qos = QoSProfile(depth=1)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            qos.durability = DurabilityPolicy.VOLATILE
            qos.history = HistoryPolicy.KEEP_LAST

            def make_status_cb(idx):
                def scb(msg):
                    try:
                        self.vehicle_status[idx] = {
                            'nav_state': int(getattr(msg, 'nav_state', -1)),
                            'arming_state': int(getattr(msg, 'arming_state', -1)),
                            'failsafe': bool(getattr(msg, 'failsafe', False)),
                            'pre_flight_checks_pass': bool(getattr(msg, 'pre_flight_checks_pass', False)),
                        }
                    except Exception:
                        pass
                return scb

            def make_ctrl_cb(idx):
                def ccb(msg):
                    try:
                        self.vehicle_control_mode[idx] = {
                            'flag_armed': bool(getattr(msg, 'flag_armed', False)),
                            'flag_control_manual_enabled': bool(getattr(msg, 'flag_control_manual_enabled', False)),
                            'flag_control_offboard_enabled': bool(getattr(msg, 'flag_control_offboard_enabled', False)),
                            'flag_control_auto_enabled': bool(getattr(msg, 'flag_control_auto_enabled', False)),
                        }
                    except Exception:
                        pass
                return ccb

            def make_batt_cb(idx):
                def bcb(msg):
                    try:
                        self.battery_status[idx] = {
                            'voltage_v': float(getattr(msg, 'voltage_v', 0.0)),
                            'current_a': float(getattr(msg, 'current_a', 0.0)),
                            'remaining': float(getattr(msg, 'remaining', -1.0)),
                        }
                    except Exception:
                        pass
                return bcb

            def make_gps_cb(idx):
                def gpcb(msg):
                    try:
                        lat = float(getattr(msg, 'lat', 0.0))
                        lon = float(getattr(msg, 'lon', 0.0))
                        alt = float(getattr(msg, 'alt', 0.0))
                        if abs(lat) > 180.0 or abs(lon) > 180.0:
                            lat = lat / 1e7
                            lon = lon / 1e7
                        self.vehicle_gps_position[idx] = {
                            'lat': lat,
                            'lon': lon,
                            'alt': alt,
                            'fix_type': int(getattr(msg, 'fix_type', -1)),
                            'satellites_used': int(getattr(msg, 'satellites_used', -1)),
                        }
                    except Exception:
                        pass
                return gpcb

            def make_land_cb(idx):
                def lcb(msg):
                    try:
                        self.vehicle_land_detected[idx] = {
                            'landed': bool(getattr(msg, 'landed', False)),
                            'freefall': bool(getattr(msg, 'freefall', False)),
                            'maybe_landed': bool(getattr(msg, 'maybe_landed', False)),
                        }
                    except Exception:
                        pass
                return lcb

            def make_takeoff_cb(idx):
                def tcb(msg):
                    try:
                        self.takeoff_status[idx] = {
                            'takeoff_state': int(getattr(msg, 'takeoff_state', -1)),
                        }
                    except Exception:
                        pass
                return tcb

            self.create_subscription(VehicleStatus, f'/px4_{i}/fmu/out/vehicle_status', make_status_cb(i), qos)
            if HAS_VEHICLE_CONTROL_MODE and VehicleControlMode is not None:
                self.create_subscription(VehicleControlMode, f'/px4_{i}/fmu/out/vehicle_control_mode', make_ctrl_cb(i), qos)
            if HAS_BATTERY_STATUS and BatteryStatus is not None:
                self.create_subscription(BatteryStatus, f'/px4_{i}/fmu/out/battery_status', make_batt_cb(i), qos)
            if HAS_VEHICLE_GPS_POSITION and VehicleGpsPosition is not None:
                self.create_subscription(VehicleGpsPosition, f'/px4_{i}/fmu/out/vehicle_gps_position', make_gps_cb(i), qos)
            if HAS_VEHICLE_LAND_DETECTED and VehicleLandDetected is not None:
                self.create_subscription(VehicleLandDetected, f'/px4_{i}/fmu/out/vehicle_land_detected', make_land_cb(i), qos)
            if HAS_TAKEOFF_STATUS and TakeoffStatus is not None:
                self.create_subscription(TakeoffStatus, f'/px4_{i}/fmu/out/takeoff_status', make_takeoff_cb(i), qos)

            # subscribe to ActuatorArmed or VehicleStatus if available so we
            # can reflect the real armed state (PX4 may reject disarm while
            # in-air or for safety reasons). We add callbacks defensively.
            try:
                def make_armed_cb(idx):
                    def acb(msg):
                        # Try common field names
                        if hasattr(msg, 'armed'):
                            self.armed[idx] = bool(msg.armed)
                        elif hasattr(msg, 'arming_state'):
                            # best-effort: consider non-zero arming_state as armed
                            try:
                                self.armed[idx] = int(msg.arming_state) != 0
                            except Exception:
                                self.armed[idx] = False
                        else:
                            # unknown message layout
                            pass

                    return acb

                # ActuatorArmed topic (preferred)
                self.create_subscription(ActuatorArmed, f'/px4_{i}/fmu/out/actuator_armed', make_armed_cb(i), qos)
            except Exception:
                try:
                    # fallback to VehicleStatus
                    self.create_subscription(VehicleStatus, f'/px4_{i}/fmu/out/vehicle_status', make_armed_cb(i), qos)
                except Exception:
                    # no status subscriptions available; continue
                    pass
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
            # if manual mode, publish velocity setpoint (teleop or zero) and skip position stream
            if self.manual_mode.get(i, False):
                self._publish_offboard_mode_pos_vel(i)
                vx, vy, z_hold = self.teleop_cmd.get(i, [0.0, 0.0, 0.0])
                self._publish_velocity_hold_z(i, vx, vy, z_hold, 0.0)
                continue

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

    def _publish_offboard_mode_velocity(self, idx):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pubs[idx].publish(msg)

    def _publish_offboard_mode_pos_vel(self, idx):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
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

    def _publish_velocity(self, idx, vx, vy, vz, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pubs[idx].publish(msg)

    def _publish_velocity_hold_z(self, idx, vx, vy, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float(z)]
        msg.velocity = [float(vx), float(vy), float('nan')]
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

    def disarm(self, idx):
        # send disarm command (VEHICLE_CMD_COMPONENT_ARM_DISARM = 400 with param1=0)
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for disarm')

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
            log.warning('No PX4 subscribers detected for px4_%d (waited %.1fs) — disarm may fail', i, waited)

        repeat = 5
        targets = [i, 1, 0]  # try specific idx, system 1, then broadcast
        sent_any = False
        for tgt in targets:
            for attempt in range(repeat):
                try:
                    msg = VehicleCommand()
                    msg.param1 = float(0.0)
                    msg.param2 = float(0.0)
                    msg.command = int(400)
                    msg.target_system = int(tgt)
                    msg.target_component = 1
                    msg.source_system = 1
                    msg.source_component = 1
                    msg.from_external = True
                    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

                    pub.publish(msg)
                    sent_any = True
                    log.debug('disarm publish attempt %d for px4_%d via target %s', attempt + 1, i, tgt)
                except Exception:
                    log.exception('failed sending disarm to %d (attempt %d) via target %s', i, attempt + 1, tgt)

            # give PX4 a short moment to apply
            time.sleep(0.15)
            # check if PX4 reported disarmed via subscribed status
            try:
                if not bool(self.armed.get(i, False)):
                    log.info('disarm accepted for px4_%d via target %s', i, tgt)
                    return True
            except Exception:
                pass

        log.info('disarm() attempted to %d via targets %s (sent_any=%s)', i, targets, sent_any)
        # if PX4 didn't report disarmed, leave local flag as-is (reflects actual state)
        return False

    def force_disarm(self, idx):
        # force disarm using MAV_CMD_COMPONENT_ARM_DISARM param2=21196
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for force disarm')

        pub = self.cmd_pubs.get(i)
        if pub is None:
            raise KeyError(f'No vehicle_command publisher for idx {i}')

        repeat = 5
        targets = [i, 1, 0]
        for tgt in targets:
            for attempt in range(repeat):
                try:
                    msg = VehicleCommand()
                    msg.param1 = float(0.0)
                    msg.param2 = float(21196.0)
                    msg.command = int(400)
                    msg.target_system = int(tgt)
                    msg.target_component = 1
                    msg.source_system = 1
                    msg.source_component = 1
                    msg.from_external = True
                    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                    pub.publish(msg)
                    log.debug('force disarm publish attempt %d for px4_%d via target %s', attempt + 1, i, tgt)
                except Exception:
                    log.exception('failed force disarm to %d (attempt %d) via target %s', i, attempt + 1, tgt)
            time.sleep(0.15)

        log.info('force_disarm() attempted to %d via targets %s', i, targets)
        return True

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

    def teleop_velocity(self, idx, vx, vy, z_hold):
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for teleop')
        self.manual_mode[i] = True
        self.teleop_cmd[i] = [float(vx), float(vy), float(z_hold)]
        self.teleop_until[i] = time.time() + 0.6
        self._publish_offboard_mode_pos_vel(i)
        self._publish_velocity_hold_z(i, vx, vy, z_hold, 0.0)
        log.debug('teleop_velocity to %d: vx=%.2f vy=%.2f z=%.2f', i, vx, vy, z_hold)

    def set_manual_mode(self, idx, enabled):
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for manual mode')
        self.manual_mode[i] = bool(enabled)
        if not enabled:
            self.teleop_until[i] = 0.0
            self.teleop_cmd[i] = [0.0, 0.0, 0.0]
        # clear any pending position targets when entering manual
        if enabled:
            self.manual_targets[i] = None
            self.manual_target_counters[i] = 0

    def send_global_setpoint(self, idx, lat, lon, alt=None, yaw=0.0):
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for global setpoint')

        pub = self.pos_pubs.get(i) if hasattr(self, 'pos_pubs') else None
        if pub is None:
            raise KeyError(f'No position_setpoint publisher for idx {i}')

        msg = PositionSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.valid = True
        msg.type = PositionSetpoint.SETPOINT_TYPE_POSITION
        msg.lat = float(lat)
        msg.lon = float(lon)
        msg.alt = float(alt) if alt is not None else 0.0
        msg.yaw = float(yaw)
        # acceptance radii left default
        try:
            pub.publish(msg)
            log.info('Published global position setpoint to px4_%d: %s,%s,%s', i, lat, lon, alt)
        except Exception:
            log.exception('failed publishing global setpoint to %d', i)


    def land(self, idx):
        # command the drone to land at current position using MAV_CMD_NAV_LAND (21)
        try:
            i = int(idx)
        except Exception:
            raise ValueError('invalid idx for land')

        # wait briefly for subscriptions
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
            log.warning('No PX4 subscribers detected for px4_%d (waited %.1fs) — land may fail', i, waited)

        # send MAV_CMD_NAV_LAND (21) multiple times
        counter = 0
        repeat = 5
        while counter < repeat:
            try:
                # param1..param7 left as defaults; PX4 should interpret this as 'land here'
                self._publish_vehicle_command(i, 21)
            except Exception:
                log.exception('failed sending land to %d (attempt %d)', i, counter + 1)
            counter += 1

        log.info('land() sent to %d (repeated %d times)', i, repeat)

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
                'global_position': self.global_positions.get(i),
                'vehicle_status': self.vehicle_status.get(i),
                'vehicle_control_mode': self.vehicle_control_mode.get(i),
                'battery_status': self.battery_status.get(i),
                'vehicle_gps_position': self.vehicle_gps_position.get(i),
                'vehicle_land_detected': self.vehicle_land_detected.get(i),
                'takeoff_status': self.takeoff_status.get(i),
            }
        return status

    def get_global_position(self, idx):
        try:
            i = int(idx)
        except Exception:
            return None
        return self.global_positions.get(i)

    def get_home_position(self, idx):
        try:
            i = int(idx)
        except Exception:
            return None
        return self.home_positions.get(i)

    def get_local_position(self, idx):
        try:
            i = int(idx)
        except Exception:
            return None
        return self.local_positions.get(i)


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


def teleop_velocity_uav(uav_idx, vx, vy, z_hold):
    c = start_controller()
    idx = int(uav_idx)
    c.teleop_velocity(idx, vx, vy, z_hold)


def set_manual_mode_uav(uav_idx, enabled):
    c = start_controller()
    idx = int(uav_idx)
    c.set_manual_mode(idx, enabled)


def land_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    c.land(idx)


def disarm_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    return c.disarm(idx)


def force_disarm_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    return c.force_disarm(idx)


def global_setpoint_uav(uav_idx, lat, lon, alt=None, yaw=0.0):
    c = start_controller()
    idx = int(uav_idx)
    return c.send_global_setpoint(idx, lat, lon, alt, yaw)


def get_global_position_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    return c.get_global_position(idx)


def get_home_position_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    return c.get_home_position(idx)


def get_local_position_uav(uav_idx):
    c = start_controller()
    idx = int(uav_idx)
    return c.get_local_position(idx)
