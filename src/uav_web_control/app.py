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
            setpoints = cfg.get('setpoints', {})
            for sid, val in initial.items():
                idx = sid
                pose = val.get('initial_pose', {})
                x = float(pose.get('x', 0.0))
                y = float(pose.get('y', 0.0))
                # try to get altitude from first setpoint if present
                z = None
                if sid in setpoints and len(setpoints[sid]) > 0:
                    try:
                        z = float(setpoints[sid][0][2])
                    except Exception:
                        z = None
                if z is None:
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


def load_setpoints():
    """Load setpoints mapping from the preferred config file.

    Returns dict[int] -> list of setpoints (each setpoint is list [x,y,z,(yaw)])
    """
    try:
        cfg_path = None
        if PX4_CONFIG_CANDIDATE.exists():
            cfg_path = PX4_CONFIG_CANDIDATE
        elif 'PX4_CONFIG_CANDIDATE2' in globals() and PX4_CONFIG_CANDIDATE2.exists():
            cfg_path = PX4_CONFIG_CANDIDATE2
        else:
            local_cfg = Path(__file__).parent / 'config' / 'config.yaml'
            local_uavs = Path(__file__).parent / 'config' / 'uavs.yaml'
            if local_cfg.exists():
                cfg_path = local_cfg
            elif local_uavs.exists():
                cfg_path = local_uavs

        if cfg_path is None:
            return {}

        cfg = yaml.safe_load(cfg_path.read_text()) or {}
        sp = cfg.get('setpoints', {})
        out = {}
        for k, v in sp.items():
            try:
                out[int(k)] = [list(map(float, s)) for s in v]
            except Exception:
                log.exception('invalid setpoint format for %s', k)
        return out
    except Exception:
        log.exception('failed to load setpoints')
        return {}


@app.route('/')
def index():
    uavs = load_uavs()
    setpoints = load_setpoints()
    # convert setpoints keys to 'uav{n}' to match uavs keys used in template
    sp_map = {f'uav{int(k)}': v for k, v in setpoints.items()}
    return render_template('index.html', uavs=uavs, setpoints=sp_map)


@app.route('/fly', methods=['POST'])
def fly():
    data = request.get_json() or {}
    uav_id = data.get('id')
    uavs = load_uavs()
    if uav_id not in uavs:
        return jsonify({'status': 'error', 'message': 'unknown uav id'}), 400
    # default target is the configured initial position (x,y,z)
    target = uavs[uav_id].get('position')
    # allow selecting a setpoint index from config: data may contain 'index' (0-based) or 'sp_index'
    sp_index = data.get('index') if data.get('index') is not None else data.get('sp_index')
    if sp_index is not None:
        try:
            si = int(sp_index)
            setpoints = load_setpoints()
            idx_num = int(_uav_index(uav_id))
            if idx_num not in setpoints:
                return jsonify({'status': 'error', 'message': f'no setpoints for uav {uav_id}'}), 400
            if si < 0 or si >= len(setpoints[idx_num]):
                return jsonify({'status': 'error', 'message': f'invalid setpoint index {si} for {uav_id}'}), 400
            target = setpoints[idx_num][si]
        except ValueError:
            return jsonify({'status': 'error', 'message': 'index must be integer'}), 400
    # Normalize the UAV id to a numeric index (uav1 -> 1, px4_1 -> 1)
    idx = _uav_index(uav_id)

    # If rclpy and ros_control are available, use its API
    if use_rclpy and ros_control_mod is not None:
        try:
            # fly_uav expects numeric index and position
            ros_control_mod.fly_uav(idx, target)
            return jsonify({'status': 'ok', 'message': f'fly command sent to {uav_id} (px4_{idx})'})
        except Exception as e:
            log.exception('ros_control.fly_uav failed')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Fallback: try ros2 CLI (if available) to publish a TrajectorySetpoint,
    # otherwise use the controller stub.
    ros2 = shutil.which('ros2')
    if ros2:
        try:
            # Build message with position array and yaw (if provided)
            x, y, z = float(target[0]), float(target[1]), float(target[2])
            yaw = float(target[3]) if len(target) > 3 else 0.0
            msg_obj = {"position": [x, y, z], "yaw": yaw, "timestamp": 0}
            msg = json.dumps(msg_obj)
            topic = f"/px4_{idx}/fmu/in/trajectory_setpoint"
            cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/TrajectorySetpoint', msg]
            log.info('Publishing TrajectorySetpoint via CLI: %s', ' '.join(cmd))
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if proc.returncode == 0:
                return jsonify({'status': 'ok', 'message': f'fly command published to {topic}'})
            else:
                return jsonify({'status': 'error', 'message': proc.stderr or proc.stdout or proc.returncode}), 500
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Last-resort: controller stub writes last_command.json for debugging
    try:
        fly_to(uav_id, target)
        return jsonify({'status': 'ok', 'message': f'command saved for {uav_id} (no ros2 available)'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


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
    msg_obj = {
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
    msg = json.dumps(msg_obj)

    cmd = [ros2, 'topic', 'pub', '-1', topic, 'px4_msgs/msg/VehicleCommand', msg]
    log.info('Falling back to ros2 CLI: %s', ' '.join(cmd))

    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if proc.returncode == 0:
            return jsonify({'status': 'ok', 'message': f'arm command published to {topic}'})
        else:
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
