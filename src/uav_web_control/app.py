from flask import Flask, render_template, jsonify, request, send_from_directory, send_file
import yaml
from pathlib import Path
import logging
import json

# Import controller.fly_to with a fallback so the module can be run as
# a script (python app.py) as well as imported as a package.
try:
    from .controller import fly_to
except (ImportError, SystemError):
    # Fallback for direct script execution where package context is not set
    from controller import fly_to
import shutil
import subprocess
import threading

# Try to import rclpy and px4 message types. If available, we'll use rclpy to
# publish messages directly instead of invoking the `ros2` CLI. This is more
# reliable and avoids the "passed message type is invalid" CLI error.
use_rclpy = False
publisher_lock = threading.Lock()
publishers = {}
ros_node = None
log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

# Module reference to ros_control (if available)
ros_control_mod = None


def import_from_ros_control(attr_name):
    """Robust import for attributes from `ros_control`.

    Tries several import styles so the module works when run as a
    package (`python -m uav_web_control.app`) or as a script
    (`python app.py`). Raises ImportError if the attribute can't be
    imported.
    """
    # Try package-style absolute import first
    try:
        mod = __import__('uav_web_control.ros_control', fromlist=[attr_name])
        return getattr(mod, attr_name)
    except Exception:
        pass

    # Try plain module import (works if script is run with package dir on PYTHONPATH)
    try:
        mod = __import__('ros_control', fromlist=[attr_name])
        return getattr(mod, attr_name)
    except Exception:
        pass

    # Try relative import if package context is available
    try:
        if __package__:
            mod = __import__(f"{__package__}.ros_control", fromlist=[attr_name])
            return getattr(mod, attr_name)
    except Exception:
        pass

    raise ImportError(f"Could not import {attr_name} from ros_control via any supported path")


def import_ros_control_module():
    """Import the ros_control module robustly and return the module object.

    Tries package-style, absolute, and relative imports to support running
    as `python -m uav_web_control.app` or `python app.py`.
    """
    # Try package-style import first
    try:
        return __import__('uav_web_control.ros_control', fromlist=['ros_control'])
    except Exception:
        pass

    # Try plain module import
    try:
        return __import__('ros_control', fromlist=['ros_control'])
    except Exception:
        pass

    # Try relative import if package context is available
    try:
        if __package__:
            return __import__(f"{__package__}.ros_control", fromlist=['ros_control'])
    except Exception:
        pass

    return None

try:
    import rclpy
    # Prefer using the Python controller node mirroring px4_swarm_controller
    try:
        # Try to import the ros_control module and start its controller
        ros_mod = import_ros_control_module()
        if ros_mod is not None:
            ros_control_mod = ros_mod
            try:
                ros_control_mod.start_controller()
                use_rclpy = True
                log.info('rclpy available — using ros_control node for publishing')
            except Exception:
                log.exception('Failed to start ros_control controller')
                use_rclpy = False
        else:
            log.info('ros_control module not found; falling back')
            use_rclpy = False
    except Exception as e:
        log.exception('rclpy present but failed to start ros_control')
        use_rclpy = False
    except Exception as e:
        log.exception('rclpy present but failed to start ros_control')
        use_rclpy = False
except Exception:
    # rclpy not available; we'll fall back to ros2 CLI
    use_rclpy = False

app = Flask(__name__, static_folder='static', template_folder='templates')

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

    idx = _uav_index(uav_id)
    alt_m = float(alt) if alt is not None else (20.0 * 0.3048)
    if use_rclpy and ros_control_mod is not None and hasattr(ros_control_mod, 'get_global_position_uav'):
        gp = ros_control_mod.get_global_position_uav(idx)
    else:
        gp = None
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
        if use_rclpy and ros_control_mod is not None and hasattr(ros_control_mod, 'get_local_position_uav'):
            lp = ros_control_mod.get_local_position_uav(idx)
        else:
            lp = None
        if use_rclpy and ros_control_mod is not None and gp and lp:
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
                ros_control_mod.fly_uav(idx, [target_x, target_y, target_z, 0.0])
                return jsonify({'status': 'ok', 'message': f'local setpoint sent to {uav_id} (px4_{idx})'})
            except Exception:
                log.exception('failed to compute local target from GPS')
        # fallback to global setpoint
        if use_rclpy and ros_control_mod is not None:
            ros_control_mod.global_setpoint_uav(idx, lat, lon, alt_m)
            return jsonify({'status': 'ok', 'message': f'global setpoint sent to {uav_id} (px4_{idx})'})
    except Exception as e:
        log.exception('ros_control.global_setpoint_uav failed')
        return jsonify({'status': 'error', 'message': str(e)}), 500

    ros2 = shutil.which('ros2')
    if ros2:
        try:
            topic = f"/px4_{idx}/fmu/in/position_setpoint"
            msg_obj = {
                "timestamp": 0,
                "valid": True,
                "type": 0,
                "lat": float(lat),
                "lon": float(lon),
                "alt": float(alt_m) if alt_m is not None else 0.0,
                "yaw": 0.0
            }
            cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/PositionSetpoint', json.dumps(msg_obj)]
            log.info('Publishing PositionSetpoint via CLI: %s', ' '.join(cmd))
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if proc.returncode == 0:
                return jsonify({'status': 'ok', 'message': f'global setpoint published to {topic}'})
            else:
                return jsonify({'status': 'error', 'message': proc.stderr or proc.stdout or proc.returncode}), 500
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    return jsonify({'status': 'error', 'message': 'rclpy not available and ros2 CLI not found to publish global setpoint'}), 500


@app.route('/arm', methods=['POST'])
def arm():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    # Normalize the UAV id to a numeric index (uav1 -> 1, px4_1 -> 1)
    idx = _uav_index(uav_id)

    # Build the topic name for the specific drone (used only for logging/fallback)
    topic = f"/px4_{idx}/fmu/in/vehicle_command"

    # Check that ros2 CLI is available in the environment
    ros2 = shutil.which('ros2')
    if not ros2:
        return jsonify({'status': 'error', 'message': 'ros2 CLI not found in PATH. Run Flask in a ROS2-sourced environment.'}), 500

    # VEHICLE_CMD_COMPONENT_ARM_DISARM has numeric value 400
    if use_rclpy and ros_control_mod is not None:
        try:
            ros_control_mod.arm_uav(idx)
            return jsonify({'status': 'ok', 'message': f'arm command sent to {uav_id} (px4_{idx})'})
        except Exception as e:
            log.exception('ros_control.arm_uav failed')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Fallback to ros2 CLI if rclpy unavailable
    # Build a JSON-style message string (use double quotes) to avoid parsing issues
    try:
        tnum = int(idx)
    except Exception:
        tnum = 1
    arm_msg_obj = {
        "param1": 1.0,
        "param2": 0.0,
        "command": 400,
        "target_system": tnum,
        "target_component": 1,
        "source_system": 1,
        "source_component": 1,
        "from_external": True,
        "timestamp": 0,
    }
    arm_msg = json.dumps(arm_msg_obj)

    # Offboard mode: VEHICLE_CMD_DO_SET_MODE (176) with params as used in ros_control
    offboard_msg_obj = {
        "param1": 1.0,
        "param2": 6.0,
        "command": 176,
        "target_system": tnum,
        "target_component": 1,
        "source_system": 1,
        "source_component": 1,
        "from_external": True,
        "timestamp": 0,
    }
    offboard_msg = json.dumps(offboard_msg_obj)

    arm_cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/VehicleCommand', arm_msg]
    offboard_cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/VehicleCommand', offboard_msg]
    log.info('Falling back to ros2 CLI (arm): %s', ' '.join(arm_cmd))
    log.info('Falling back to ros2 CLI (offboard): %s', ' '.join(offboard_cmd))

    try:
        proc = subprocess.run(arm_cmd, capture_output=True, text=True, timeout=10)
        if proc.returncode != 0:
            return jsonify({'status': 'error', 'message': proc.stderr or proc.stdout}), 500
        proc = subprocess.run(offboard_cmd, capture_output=True, text=True, timeout=10)
        if proc.returncode == 0:
            return jsonify({'status': 'ok', 'message': f'arm+offboard commands published to {topic}'})
        return jsonify({'status': 'error', 'message': proc.stderr or proc.stdout}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/disarm', methods=['POST'])
def disarm():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    idx = _uav_index(uav_id)
    topic = f"/px4_{idx}/fmu/in/vehicle_command"

    # If ros_control available, call its disarm helper
    if use_rclpy and ros_control_mod is not None:
        try:
            result = ros_control_mod.disarm_uav(idx)
            if result:
                return jsonify({'status': 'ok', 'message': f'disarm accepted by {uav_id} (px4_{idx})'})
            else:
                return jsonify({'status': 'error', 'message': f'disarm attempted but vehicle still reports armed for {uav_id} (px4_{idx})'}), 500
        except Exception as e:
            log.exception('ros_control.disarm_uav failed')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    ros2 = shutil.which('ros2')
    if not ros2:
        return jsonify({'status': 'error', 'message': 'ros2 CLI not found in PATH. Run Flask in a ROS2-sourced environment.'}), 500

    try:
        tnum = int(idx)
    except Exception:
        tnum = 1
    msg_obj = {
        "param1": 0.0,
        "param2": 0.0,
        "command": 400,
        "target_system": tnum,
        "target_component": 1,
        "source_system": 1,
        "source_component": 1,
        "from_external": True,
        "timestamp": 0,
    }
    msg = json.dumps(msg_obj)

    cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/VehicleCommand', msg]
    log.info('Falling back to ros2 CLI for disarm: %s', ' '.join(cmd))

    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if proc.returncode == 0:
            return jsonify({'status': 'ok', 'message': f'disarm command published to {topic}'})
        else:
            return jsonify({'status': 'error', 'message': proc.stderr or proc.stdout}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/land', methods=['POST'])
def land():
    data = request.get_json() or {}
    uav_id = data.get('id')
    if not uav_id:
        return jsonify({'status': 'error', 'message': 'missing uav id'}), 400

    idx = _uav_index(uav_id)
    topic = f"/px4_{idx}/fmu/in/vehicle_command"

    ros2 = shutil.which('ros2')
    if not ros2:
        return jsonify({'status': 'error', 'message': 'ros2 CLI not found in PATH. Run Flask in a ROS2-sourced environment.'}), 500

    if use_rclpy and ros_control_mod is not None:
        try:
            # call ros_control land helper
            ros_control_mod.land_uav(idx)
            return jsonify({'status': 'ok', 'message': f'land command sent to {uav_id} (px4_{idx})'})
        except Exception as e:
            log.exception('ros_control.land_uav failed')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Fallback to ros2 CLI: send VehicleCommand with command=21 (MAV_CMD_NAV_LAND)
    try:
        tnum = int(idx)
    except Exception:
        tnum = 1
    msg_obj = {
        "param1": 0.0,
        "param2": 0.0,
        "command": 21,
        "target_system": tnum,
        "target_component": 1,
        "source_system": 1,
        "source_component": 1,
        "from_external": True,
        "timestamp": 0,
    }
    msg = json.dumps(msg_obj)

    cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/VehicleCommand', msg]
    log.info('Falling back to ros2 CLI for land: %s', ' '.join(cmd))

    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if proc.returncode == 0:
            return jsonify({'status': 'ok', 'message': f'land command published to {topic}'})
        else:
            return jsonify({'status': 'error', 'message': proc.stderr or proc.stdout}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/status', methods=['GET'])
def status():
    """Return controller status for each UAV (connected / subscriber counts)."""
    # If ros_control is available, ask it for status
    if use_rclpy and ros_control_mod is not None:
        try:
            st = ros_control_mod._controller.get_uav_status() if hasattr(ros_control_mod, '_controller') else ros_control_mod.get_uav_status()
            # convert keys to strings like 'uav1'
            out = {f'uav{k}': v for k, v in st.items()}
            # attach derived lat/lon if geo_origin available
            for key, v in out.items():
                try:
                    gp = v.get('global_position')
                    if isinstance(gp, (list, tuple)) and len(gp) >= 2:
                        v['lat'] = float(gp[0])
                        v['lon'] = float(gp[1])
                        v['alt'] = float(gp[2]) if len(gp) > 2 else 0.0
                    else:
                        lp = v.get('local_position')
                        if isinstance(lp, (list, tuple)) and len(lp) >= 2:
                            lat, lon, alt = local_to_latlon(lp[0], lp[1], lp[2] if len(lp) > 2 else None)
                            v['lat'] = float(lat)
                            v['lon'] = float(lon)
                            v['alt'] = float(alt)
                except Exception:
                    # best-effort: ignore conversion errors
                    pass
            return jsonify({'status': 'ok', 'data': out})
        except Exception as e:
            log.exception('Failed to get ros_control status')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Fallback: indicate no controller
    return jsonify({'status': 'ok', 'data': {}, 'info': 'ros_control not available'})


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
    # Turn off the reloader to avoid multiple processes interfering with rclpy.
    # Recommended run: `python3 -m uav_web_control.app` or run from a ROS2-sourced
    # terminal with `python3 app.py` in this folder. Do not use Flask reloader.
    app.run(debug=False, host='0.0.0.0', use_reloader=False)
