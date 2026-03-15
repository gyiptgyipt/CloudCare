# CloudCare
care flies



# Target


# Resources to research
 
    -Grab (priority) (www.businesstimes.com.sg/startups-tech/startups/Tech-in-Asia/how-chinas-playbook-could-guide-grabs-drone-gamble)
    -Zipline
    -Wingcopter
    -Swoop Aero
# UAV Typhon Statistics

    -Battery
    -Payload
    -effetivity






Map Calibration (SITL)

If you're running SITL, the UAV GPS may not match the map location. Use the
calibration flow to align the map:

1. Open a UAV's Maximize view.
2. Click the map at the spot where the UAV should appear.
3. Click "Calibrate Map".
4. After calibration, the UAV marker and goal points will line up.

Notes:
- Calibration is per-UAV and stored in the browser (page reload clears it).
- Goals are adjusted automatically using the calibration offset.

Arm notes

- The "Arm" buttons publish a ROS2 `px4_msgs/msg/VehicleCommand` arm message to the topic `/px4_<id>/fmu/in/vehicle_command` using the `ros2` CLI.
- You must run the Flask app in an environment where `ros2` is available (source your ROS2 installation and any workspace, e.g. `source /opt/ros/foxy/setup.bash` or your workspace `install/setup.bash`).
- Example: start Flask from a terminal where you've sourced ROS2.


Notes

- `controller.fly_to` is a placeholder: integrate with your MAVLink/ROS/PX4 APIs there.
- Last command is written to `last_command.json` for quick inspection during development.
