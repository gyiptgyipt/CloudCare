from flask import Flask, render_template, jsonify, request
import yaml
from pathlib import Path
import logging

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

try:
    import rclpy
    # Prefer using the Python controller node mirroring px4_swarm_controller
    try:
        # Use the robust helper to import ros_control attributes.
        try:
            start_controller = import_from_ros_control('start_controller')
            arm_uav = import_from_ros_control('arm_uav')
            fly_uav = import_from_ros_control('fly_uav')
            log.info('Imported ros_control via import_from_ros_control')

            # start_controller will init rclpy and spin the node
            start_controller()
            use_rclpy = True
            log.info('rclpy available — using ros_control node for publishing')
        except Exception:
            log.exception('Failed to import ros_control via import_from_ros_control')
            use_rclpy = False
    except Exception as e:
        log.exception('rclpy present but failed to start ros_control')
        use_rclpy = False
except Exception:
    # rclpy not available; we'll fall back to ros2 CLI
    use_rclpy = False

app = Flask(__name__, static_folder='static', template_folder='templates')

CONFIG_PATH = Path(__file__).parent / 'config' / 'uavs.yaml'


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
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH, 'r') as f:
            return yaml.safe_load(f) or {}
    return {}


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
    target = uavs[uav_id].get('position')
    # Normalize the UAV id to a numeric index (uav1 -> 1, px4_1 -> 1)
    idx = _uav_index(uav_id)

    # If rclpy and ros_control are available, use its API
    if use_rclpy:
        try:
            # fly_uav expects numeric index and position
            fly_uav = import_from_ros_control('fly_uav')
            fly_uav(idx, target)
            return jsonify({'status': 'ok', 'message': f'fly command sent to {uav_id} (px4_{idx})'})
        except Exception as e:
            log.exception('ros_control.fly_uav failed')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Fallback: controller stub or ros2 CLI for environments without rclpy
    try:
        fly_to(uav_id, target)
        return jsonify({'status': 'ok', 'message': f'command sent to {uav_id}'})
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
    if use_rclpy:
        try:
            arm_uav = import_from_ros_control('arm_uav')
            arm_uav(idx)
            return jsonify({'status': 'ok', 'message': f'arm command sent to {uav_id} (px4_{idx})'})
        except Exception as e:
            log.exception('ros_control.arm_uav failed')
            return jsonify({'status': 'error', 'message': str(e)}), 500

    # Fallback to ros2 CLI if rclpy unavailable
    # Build a JSON-style message string (use double quotes) to avoid parsing issues
    msg = '{"param1": 1.0, "param2": 0.0, "command": 400, "target_system": 1, "target_component": 1, "source_system": 1, "source_component": 1, "from_external": true, "timestamp": 0}'

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


if __name__ == '__main__':
    # Turn off the reloader to avoid multiple processes interfering with rclpy.
    # Recommended run: `python3 -m uav_web_control.app` or run from a ROS2-sourced
    # terminal with `python3 app.py` in this folder. Do not use Flask reloader.
    app.run(debug=False, host='0.0.0.0', use_reloader=False)
