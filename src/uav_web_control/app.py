from flask import Flask, render_template, jsonify, request, send_file
import yaml
from pathlib import Path
import logging
import os
import threading
import time

log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

try:
    import roslibpy
except Exception:
    roslibpy = None

# Rosbridge websocket configuration
ROSBRIDGE_HOST = os.environ.get('ROSBRIDGE_HOST', 'localhost')
ROSBRIDGE_PORT = int(os.environ.get('ROSBRIDGE_PORT', '9090'))
ROSBRIDGE_USE_SSL = os.environ.get('ROSBRIDGE_USE_SSL', 'false').lower() in ('1', 'true', 'yes')
ROSBRIDGE_CONNECT_TIMEOUT_S = float(os.environ.get('ROSBRIDGE_CONNECT_TIMEOUT_S', '3.0'))

_ros_lock = threading.Lock()
_ros_client = None
_ros_topics = {}
_ros_subs = {}
_ros_cache = {}
_ros_subscribed = False


def _ensure_rosbridge():
    if roslibpy is None:
        raise RuntimeError('roslibpy not installed. Add it to requirements and install dependencies.')
    global _ros_client
    with _ros_lock:
        if _ros_client is None:
            _ros_client = roslibpy.Ros(
                host=ROSBRIDGE_HOST,
                port=ROSBRIDGE_PORT,
                is_secure=ROSBRIDGE_USE_SSL,
            )
            _ros_client.run()
        # wait for connection
        start = time.time()
        while not _ros_client.is_connected:
            if time.time() - start > ROSBRIDGE_CONNECT_TIMEOUT_S:
                raise RuntimeError(f'rosbridge not connected at {ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}')
            time.sleep(0.05)
    return _ros_client


def _get_topic(topic_name, msg_type):
    key = (topic_name, msg_type)
    if key in _ros_topics:
        return _ros_topics[key]
    client = _ensure_rosbridge()
    topic = roslibpy.Topic(client, topic_name, msg_type)
    _ros_topics[key] = topic
    return topic


def _publish(topic_name, msg_type, msg):
    topic = _get_topic(topic_name, msg_type)
    topic.publish(roslibpy.Message(msg))


def _ensure_cache(idx):
    if idx not in _ros_cache:
        _ros_cache[idx] = {
            'local_position': None,
            'global_position': None,
            'vehicle_status': None,
            'battery_status': None,
            'takeoff_status': None,
            'armed': False,
            'home': None,
            'last_seen': 0.0,
        }


def _touch(idx):
    _ensure_cache(idx)
    _ros_cache[idx]['last_seen'] = time.time()


def _subscribe_topic(topic_name, msg_type, cb):
    key = (topic_name, msg_type)
    if key in _ros_subs:
        return
    topic = _get_topic(topic_name, msg_type)
    topic.subscribe(cb)
    _ros_subs[key] = topic


def _uav_ids_from_config():
    uavs = load_uavs()
    ids = []
    for uid in uavs.keys():
        idx = _uav_index(uid)
        try:
            ids.append(int(idx))
        except Exception:
            continue
    if ids:
        return sorted(set(ids))
    # fallback to config with num_drones if present
    for cfg in (PX4_CONFIG_CANDIDATE, PX4_CONFIG_CANDIDATE2, Path(__file__).parent / 'config' / 'config.yaml'):
        try:
            if cfg.exists():
                data = yaml.safe_load(cfg.read_text()) or {}
                n = int(data.get('num_drones', 0))
                if n > 0:
                    return list(range(1, n + 1))
        except Exception:
            pass
    return []


def _ensure_subscriptions():
    global _ros_subscribed
    if _ros_subscribed:
        return
    _ensure_rosbridge()
    ids = _uav_ids_from_config()
    for idx in ids:
        _ensure_cache(idx)

        def make_local_cb(i):
            def cb(msg):
                _touch(i)
                _ros_cache[i]['local_position'] = [
                    float(msg.get('x', 0.0)),
                    float(msg.get('y', 0.0)),
                    float(msg.get('z', 0.0)),
                ]
            return cb

        def make_global_cb(i):
            def cb(msg):
                _touch(i)
                lat = float(msg.get('lat', 0.0))
                lon = float(msg.get('lon', 0.0))
                alt = float(msg.get('alt', 0.0))
                if abs(lat) > 180.0 or abs(lon) > 180.0:
                    lat = lat / 1e7
                    lon = lon / 1e7
                _ros_cache[i]['global_position'] = [lat, lon, alt]
                if _ros_cache[i]['home'] is None:
                    _ros_cache[i]['home'] = [lat, lon, alt]
            return cb

        def make_status_cb(i):
            def cb(msg):
                _touch(i)
                _ros_cache[i]['vehicle_status'] = msg
                try:
                    arming_state = int(msg.get('arming_state', -1))
                    _ros_cache[i]['armed'] = arming_state == 2
                except Exception:
                    pass
            return cb

        def make_batt_cb(i):
            def cb(msg):
                _touch(i)
                _ros_cache[i]['battery_status'] = msg
            return cb

        def make_takeoff_cb(i):
            def cb(msg):
                _touch(i)
                _ros_cache[i]['takeoff_status'] = msg
            return cb

        def make_armed_cb(i):
            def cb(msg):
                _touch(i)
                try:
                    _ros_cache[i]['armed'] = bool(msg.get('armed', False))
                except Exception:
                    pass
            return cb

        _subscribe_topic(f'/px4_{idx}/fmu/out/vehicle_local_position', 'px4_msgs/msg/VehicleLocalPosition', make_local_cb(idx))
        _subscribe_topic(f'/px4_{idx}/fmu/out/vehicle_global_position', 'px4_msgs/msg/VehicleGlobalPosition', make_global_cb(idx))
        _subscribe_topic(f'/px4_{idx}/fmu/out/vehicle_status', 'px4_msgs/msg/VehicleStatus', make_status_cb(idx))
        _subscribe_topic(f'/px4_{idx}/fmu/out/battery_status', 'px4_msgs/msg/BatteryStatus', make_batt_cb(idx))
        _subscribe_topic(f'/px4_{idx}/fmu/out/takeoff_status', 'px4_msgs/msg/TakeoffStatus', make_takeoff_cb(idx))
        _subscribe_topic(f'/px4_{idx}/fmu/out/actuator_armed', 'px4_msgs/msg/ActuatorArmed', make_armed_cb(idx))

    _ros_subscribed = True


def _connected(idx, timeout_s=3.0):
    _ensure_cache(idx)
    return (time.time() - _ros_cache[idx]['last_seen']) < timeout_s


def _publish_vehicle_command(idx, command, param1=0.0, param2=0.0):
    msg = {
        'param1': float(param1),
        'param2': float(param2),
        'command': int(command),
        'target_system': int(idx),
        'target_component': 1,
        'source_system': 1,
        'source_component': 1,
        'from_external': True,
        'timestamp': 0,
    }
    _publish(f'/px4_{idx}/fmu/in/vehicle_command', 'px4_msgs/msg/VehicleCommand', msg)


def _publish_offboard_setpoint(idx, x, y, z, yaw=0.0):
    offboard = {
        'timestamp': 0,
        'position': True,
        'velocity': False,
        'acceleration': False,
        'attitude': False,
        'body_rate': False,
        'thrust_and_torque': False,
        'direct_actuator': False,
    }
    traj = {
        'timestamp': 0,
        'position': [float(x), float(y), float(z)],
        'velocity': [0.0, 0.0, 0.0],
        'acceleration': [0.0, 0.0, 0.0],
        'jerk': [0.0, 0.0, 0.0],
        'yaw': float(yaw),
        'yawspeed': 0.0,
    }
    _publish(f'/px4_{idx}/fmu/in/offboard_control_mode', 'px4_msgs/msg/OffboardControlMode', offboard)
    _publish(f'/px4_{idx}/fmu/in/trajectory_setpoint', 'px4_msgs/msg/TrajectorySetpoint', traj)


def _publish_offboard_burst(idx, x, y, z, yaw=0.0, count=6, interval=0.1):
    def worker():
        for _ in range(count):
            try:
                _publish_offboard_setpoint(idx, x, y, z, yaw)
            except Exception:
                log.exception('offboard publish failed')
                break
            time.sleep(interval)
    threading.Thread(target=worker, daemon=True).start()


def _publish_global_setpoint(idx, lat, lon, alt_m):
    msg_obj = {
        'timestamp': 0,
        'valid': True,
        'type': 0,
        'lat': float(lat),
        'lon': float(lon),
        'alt': float(alt_m) if alt_m is not None else 0.0,
        'yaw': 0.0,
        'vx': 0.0,
        'vy': 0.0,
        'vz': 0.0,
    }
    _publish(f'/px4_{idx}/fmu/in/position_setpoint', 'px4_msgs/msg/PositionSetpoint', msg_obj)

app = Flask(__name__, static_folder='static', template_folder='templates')
manual_mode_flags = {}

LOCAL_CONFIG_PATH = Path(__file__).parent / 'config' / 'uavs.yaml'
# Try to prefer the px4_swarm_controller config if available in the workspace
PX4_CONFIG_CANDIDATE = Path(__file__).resolve().parents[1] / 'px4_swarm_controller' / 'config' / 'config.yaml'
PX4_CONFIG_CANDIDATE2 = Path(__file__).resolve().parents[1] / 'px4_swarm_controller' / 'config' / 'drones.yaml'

# Path to last command log produced by controller stub
LAST_CMD = Path(__file__).parent / 'last_command.json'

# Remove last command file at startup to clear state
try:
    if LAST_CMD.exists():
        LAST_CMD.unlink()
        log.info('Cleared last command file: %s', LAST_CMD)
except Exception:
    pass


def _uav_index(uav_id):
    """Normalize uav identifier to a numeric index string.

    Examples:
    - 'uav1' -> '1'
    - 'px4_1' -> '1'
    - '1' -> '1'
    """
    if uav_id is None:
        return None
    s = str(uav_id)
    # strip common prefixes
    if s.startswith('px4_'):
        s = s[4:]
    if s.startswith('uav'):
        s = s[3:]
    # extract first sequence of digits
    import re
    m = re.search(r"(\d+)", s)
    if m:
        return m.group(1)
    return s


def load_uavs():
    # If px4_swarm_controller config exists, use it to build UAV entries
    try:
        cfg_path = None
        if PX4_CONFIG_CANDIDATE.exists():
            cfg_path = PX4_CONFIG_CANDIDATE
        elif PX4_CONFIG_CANDIDATE2.exists():
            cfg_path = PX4_CONFIG_CANDIDATE2

        if cfg_path is not None:
            cfg = yaml.safe_load(cfg_path.read_text()) or {}
            uavs = {}
            # initial_positions contains mapping of string id -> { initial_pose: { x, y } }
            initial = cfg.get('initial_positions', {})
            for sid, val in initial.items():
                idx = sid
                pose = val.get('initial_pose', {})
                x = float(pose.get('x', 0.0))
                y = float(pose.get('y', 0.0))
                z = -5.0
                uavs[f'uav{idx}'] = {'name': f'Drone {idx}', 'position': [x, y, z]}
            return uavs
    except Exception:
        log.exception('Failed to load px4_swarm_controller config')

    # fallback to local config
    if LOCAL_CONFIG_PATH.exists():
        with open(LOCAL_CONFIG_PATH, 'r') as f:
            return yaml.safe_load(f) or {}
    return {}


def find_geo_origin():
    """Try to find a geo origin (lat, lon, alt) from config files.

    Looks in px4_swarm_controller config first, then local config files.
    Returns tuple (lat, lon, alt) or None.
    """
    # candidates
    candidates = []
    if PX4_CONFIG_CANDIDATE.exists():
        candidates.append(PX4_CONFIG_CANDIDATE)
    if 'PX4_CONFIG_CANDIDATE2' in globals() and PX4_CONFIG_CANDIDATE2.exists():
        candidates.append(PX4_CONFIG_CANDIDATE2)
    local_cfg = Path(__file__).parent / 'config' / 'config.yaml'
    if local_cfg.exists():
        candidates.append(local_cfg)

    for p in candidates:
        try:
            cfg = yaml.safe_load(Path(p).read_text()) or {}
            go = cfg.get('geo_origin') or cfg.get('origin') or cfg.get('geo')
            if go and isinstance(go, dict) and 'lat' in go and 'lon' in go:
                lat = float(go.get('lat'))
                lon = float(go.get('lon'))
                alt = float(go.get('alt', 0.0))
                return (lat, lon, alt)
        except Exception:
            continue
    return None


def latlon_to_local(lat, lon, alt=None, alt_frame=None):
    """Convert lat/lon/alt to local x,y,z (meters).

    Uses a simple equirectangular approximation around a geo_origin found
    in config. Returns (x, y, z) where x=east, y=north, z=negative down
    similar to existing setpoint conventions (z negative for altitude).
    """
    origin = find_geo_origin()
    if origin is None:
        raise RuntimeError('geo_origin not found in config; add geo_origin: {lat: .., lon: .., alt: ..} to your config')
    lat0, lon0, alt0 = origin
    # Earth radius (m)
    R = 6378137.0
    import math
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    mean_lat = math.radians((lat + lat0) / 2.0)
    east = R * dlon * math.cos(mean_lat)
    north = R * dlat
    # altitude: if provided use it, else use origin altitude or -5m default
    if alt is None:
        if alt0 is not None:
            z = -(alt0)
        else:
            z = -5.0
    else:
        if alt_frame == 'agl':
            # AGL height: local NED z is negative down
            z = -(float(alt))
        else:
            # AMSL: convert to local relative to geo_origin alt
            base_alt = float(alt0) if alt0 is not None else 0.0
            z = -(float(alt) - base_alt)
    return [float(east), float(north), float(z)]


def local_to_latlon(x, y, z=None):
    """Convert local x,y,z (meters) to lat/lon/alt using geo_origin."""
    origin = find_geo_origin()
    if origin is None:
        raise RuntimeError('geo_origin not found in config; add geo_origin: {lat: .., lon: .., alt: ..} to your config')
    lat0, lon0, alt0 = origin
    import math
    R = 6378137.0
    dlat = float(y) / R
    dlon = float(x) / (R * math.cos(math.radians(lat0)))
    lat = lat0 + math.degrees(dlat)
    lon = lon0 + math.degrees(dlon)
    if z is None:
        alt = float(alt0) if alt0 is not None else 0.0
    else:
        alt = float(alt0) - float(z)
    return lat, lon, alt


@app.route('/')
def index():
    uavs = load_uavs()
    return render_template('index.html', uavs=uavs)


@app.route('/fly', methods=['POST'])
def fly():
    data = request.get_json() or {}
    uav_id = data.get('id')
    uavs = load_uavs()
    if uav_id not in uavs:
        return jsonify({'status': 'error', 'message': 'unknown uav id'}), 400
    # allow lat/lon selection from UI
    lat = data.get('lat') if data.get('lat') is not None else data.get('latitude')
    lon = data.get('lon') if data.get('lon') is not None else data.get('longitude')
    alt = data.get('alt') if data.get('alt') is not None else None
    alt_ft = data.get('alt_ft') if data.get('alt_ft') is not None else None
    alt_frame = data.get('alt_frame')
    if alt_ft is not None and alt is None:
        try:
            alt = float(alt_ft) * 0.3048
        except Exception:
            return jsonify({'status': 'error', 'message': 'invalid alt_ft value'}), 400
    if alt is not None and not alt_frame:
        alt_frame = 'agl'
    if lat is None or lon is None:
        return jsonify({'status': 'error', 'message': 'lat/lon required'}), 400

    # Compute AMSL altitude without geo_origin, using UAV GPS if AGL was requested.
    try:
        lat = float(lat)
        lon = float(lon)
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid lat/lon values'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400

    alt_m = float(alt) if alt is not None else (20.0 * 0.3048)
    try:
        _ensure_subscriptions()
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

    cache = _ros_cache.get(idx, {})
    gp = cache.get('global_position')
    if alt_frame == 'agl':
        if not gp or len(gp) < 3:
            return jsonify({'status': 'error', 'message': 'no GPS altitude available for AGL conversion'}), 400
        try:
            alt_m = float(gp[2]) + float(alt_m or 0.0)
        except Exception:
            return jsonify({'status': 'error', 'message': 'invalid AGL conversion'}), 400
    elif alt_m is None and gp and len(gp) >= 3:
        # default to current GPS altitude if no altitude provided
        try:
            alt_m = float(gp[2])
        except Exception:
            pass

    # Prefer local offboard setpoint derived from current GPS + local position
    try:
        lp = cache.get('local_position')
        if gp and lp:
            try:
                cur_lat, cur_lon, cur_alt = float(gp[0]), float(gp[1]), float(gp[2])
                cur_x, cur_y, cur_z = float(lp[0]), float(lp[1]), float(lp[2])
                import math
                R = 6378137.0
                dlat = math.radians(lat - cur_lat)
                dlon = math.radians(lon - cur_lon)
                mean_lat = math.radians((lat + cur_lat) / 2.0)
                east = R * dlon * math.cos(mean_lat)
                north = R * dlat
                # PX4 local frame is NED: x=north, y=east, z=down
                target_x = cur_x + north
                target_y = cur_y + east
                if alt_frame == 'agl':
                    target_z = cur_z - float(alt_m or 0.0)
                else:
                    target_z = cur_z + (cur_alt - float(alt_m if alt_m is not None else cur_alt))
                _publish_offboard_burst(idx, target_x, target_y, target_z, 0.0)
                return jsonify({'status': 'ok', 'message': f'local setpoint sent to {uav_id} (px4_{idx})'})
            except Exception:
                log.exception('failed to compute local target from GPS')
        # fallback to global setpoint
        _publish_global_setpoint(idx, lat, lon, alt_m)
        return jsonify({'status': 'ok', 'message': f'global setpoint sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('rosbridge publish failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/arm', methods=['POST'])
def arm():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400

    try:
        _ensure_subscriptions()
        _publish_vehicle_command(idx, 400, param1=1.0, param2=0.0)
        _publish_vehicle_command(idx, 176, param1=1.0, param2=6.0)
        return jsonify({'status': 'ok', 'message': f'arm+offboard commands sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('rosbridge arm failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/disarm', methods=['POST'])
def disarm():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400

    try:
        _ensure_subscriptions()
        _publish_vehicle_command(idx, 400, param1=0.0, param2=0.0)
        return jsonify({'status': 'ok', 'message': f'disarm command sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('rosbridge disarm failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/force_disarm', methods=['POST'])
def force_disarm():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400

    try:
        _ensure_subscriptions()
        _publish_vehicle_command(idx, 400, param1=0.0, param2=21196.0)
        return jsonify({'status': 'ok', 'message': f'force disarm sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('rosbridge force disarm failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/home', methods=['POST'])
def go_home():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400

    try:
        _ensure_subscriptions()
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

    cache = _ros_cache.get(idx, {})
    home = cache.get('home')
    if not home or len(home) < 2:
        return jsonify({'status': 'error', 'message': 'home position not available yet'}), 400

    lat, lon = float(home[0]), float(home[1])
    alt_m = float(home[2]) if len(home) > 2 else None

    try:
        gp = cache.get('global_position')
        lp = cache.get('local_position')
        if gp and lp:
            try:
                cur_lat, cur_lon, cur_alt = float(gp[0]), float(gp[1]), float(gp[2])
                cur_x, cur_y, cur_z = float(lp[0]), float(lp[1]), float(lp[2])
                import math
                R = 6378137.0
                dlat = math.radians(lat - cur_lat)
                dlon = math.radians(lon - cur_lon)
                mean_lat = math.radians((lat + cur_lat) / 2.0)
                east = R * dlon * math.cos(mean_lat)
                north = R * dlat
                target_x = cur_x + north
                target_y = cur_y + east
                target_z = cur_z + (cur_alt - float(alt_m if alt_m is not None else cur_alt))
                _publish_offboard_burst(idx, target_x, target_y, target_z, 0.0)
                return jsonify({'status': 'ok', 'message': f'home local setpoint sent to {uav_id} (px4_{idx})'})
            except Exception:
                log.exception('failed to compute local home target')
        _publish_global_setpoint(idx, lat, lon, alt_m)
        return jsonify({'status': 'ok', 'message': f'home setpoint sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('rosbridge home failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/teleop', methods=['POST'])
def teleop():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    try:
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        alt_ft = float(data.get('alt_ft', 10.0))
        speed = float(data.get('speed', 2.0))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid teleop values'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400
    if not manual_mode_flags.get(uav_id, False):
        return jsonify({'status': 'error', 'message': 'manual mode disabled'}), 400

    try:
        _ensure_subscriptions()
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

    lp = _ros_cache.get(idx, {}).get('local_position')
    if not lp or len(lp) < 3:
        return jsonify({'status': 'error', 'message': 'local position not available'}), 500

    # velocity command with altitude hold from slider
    alt_m = float(alt_ft) * 0.3048
    target_z = -alt_m  # NED down
    vx = x * speed
    vy = y * speed
    try:
        cur_x, cur_y = float(lp[0]), float(lp[1])
        target_x = cur_x + vx * 0.2
        target_y = cur_y + vy * 0.2
        _publish_offboard_burst(idx, target_x, target_y, target_z, 0.0)
        return jsonify({'status': 'ok'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/manual_mode', methods=['POST'])
def manual_mode():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400
    enabled = bool(data.get('enabled'))
    manual_mode_flags[uav_id] = enabled
    if enabled:
        try:
            idx = int(_uav_index(uav_id))
            _ensure_subscriptions()
            _publish_vehicle_command(idx, 400, param1=1.0, param2=0.0)
            _publish_vehicle_command(idx, 176, param1=1.0, param2=6.0)
        except Exception:
            log.exception('manual_mode arm/offboard failed')
    return jsonify({'status': 'ok', 'enabled': enabled})


@app.route('/land', methods=['POST'])
def land():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    try:
        idx = int(_uav_index(uav_id))
    except Exception:
        return jsonify({'status': 'error', 'message': 'invalid uav id'}), 400

    try:
        _ensure_subscriptions()
        _publish_vehicle_command(idx, 21, param1=0.0, param2=0.0)
        return jsonify({'status': 'ok', 'message': f'land command sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('rosbridge land failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/status', methods=['GET'])
def status():
    """Return controller status for each UAV (connected / subscriber counts)."""
    try:
        _ensure_subscriptions()
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

    out = {}
    for idx in _uav_ids_from_config():
        _ensure_cache(idx)
        cache = _ros_cache.get(idx, {})
        info = {
            'connected': _connected(idx),
            'armed': bool(cache.get('armed', False)),
            'local_position': cache.get('local_position'),
            'global_position': cache.get('global_position'),
            'vehicle_status': cache.get('vehicle_status'),
            'battery_status': cache.get('battery_status'),
            'takeoff_status': cache.get('takeoff_status'),
        }
        try:
            gp = info.get('global_position')
            if isinstance(gp, (list, tuple)) and len(gp) >= 2:
                info['lat'] = float(gp[0])
                info['lon'] = float(gp[1])
                info['alt'] = float(gp[2]) if len(gp) > 2 else 0.0
            else:
                lp = info.get('local_position')
                if isinstance(lp, (list, tuple)) and len(lp) >= 2:
                    lat, lon, alt = local_to_latlon(lp[0], lp[1], lp[2] if len(lp) > 2 else None)
                    info['lat'] = float(lat)
                    info['lon'] = float(lon)
                    info['alt'] = float(alt)
        except Exception:
            pass
        out[f'uav{idx}'] = info

    return jsonify({'status': 'ok', 'data': out})


@app.route('/logo.png')
def logo():
    """Serve the uploaded logo placed under templates/logo/"""
    logo_dir = Path(__file__).parent / 'templates' / 'logo'
    logo_path = logo_dir / 'CloudCare logo.png'
    if not logo_path.exists():
        # fallback: try without space
        alt = logo_dir / 'CloudCare_logo.png'
        if alt.exists():
            logo_path = alt
    if not logo_path.exists():
        return ('', 404)
    return send_file(str(logo_path), mimetype='image/png')


if __name__ == '__main__':
    # Turn off the reloader to avoid multiple processes connecting to rosbridge.
    # Recommended run: `python3 -m uav_web_control.app` or `python3 app.py`.
    app.run(debug=False, host='0.0.0.0', use_reloader=False)
