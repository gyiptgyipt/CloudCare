# uav_web_control

Small starter Flask web app to send simple "fly" commands to UAVs.

Usage

1. Create a Python virtualenv and install dependencies:

```bash
cd src/uav_web_control
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

2. Run the app:

```bash
# option A (flask)
export FLASK_APP=app.py
flask run --host=0.0.0.0

# option B (direct)
python app.py
```

3. Open `http://localhost:5000` and click a UAV's "Fly" button.

Arm notes

- The "Arm" buttons publish a ROS2 `px4_msgs/msg/VehicleCommand` arm message to the topic `/px4_<id>/fmu/in/vehicle_command` using the `ros2` CLI.
- You must run the Flask app in an environment where `ros2` is available (source your ROS2 installation and any workspace, e.g. `source /opt/ros/foxy/setup.bash` or your workspace `install/setup.bash`).
- Example: start Flask from a terminal where you've sourced ROS2.


Notes

- `controller.fly_to` is a placeholder: integrate with your MAVLink/ROS/PX4 APIs there.
- Last command is written to `last_command.json` for quick inspection during development.
