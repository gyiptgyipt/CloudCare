# CloudCare (Multi-UAV Simulation + Web Control)

CloudCare is a local simulation setup for running multiple PX4 SITL drones and controlling them from a Flask web UI. This repo focuses on:
- A **tmux-based simulator launcher** (mytmux)
- A **web control server** (uav_web_control)
- **Simulation cases** (initial positions and setpoint lists)

## Quick Start (Simulation + Web UI)

### 1) Start the simulation (tmuxinator)
The tmux profile launches:
- PX4 SITL multi-vehicle simulation
- QGroundControl
- MicroXRCEAgent

```bash
cd src/mytmux/multi_tmux
tmuxinator
```

This uses:
- PX4 Autopilot at `~/PX4-Autopilot`
- QGroundControl at `~/QGroundControl.AppImage`
- MicroXRCEAgent on UDP port `8888`

If those paths differ on your machine, edit:
- `src/mytmux/multi_tmux/.tmuxinator.yml`

### 2) Start the web server
```bash
cd src/uav_web_control
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# In the same terminal, source ROS2 so px4_msgs are available
# Example (pick one that matches your install):
# source /opt/ros/foxy/setup.bash
# source /opt/ros/humble/setup.bash

export FLASK_APP=app.py
flask run --host=0.0.0.0
```

Open:
- `http://localhost:5000`

If ROS2 + `px4_msgs` are available, the web server publishes with `rclpy`. Otherwise, it falls back to `ros2` CLI publishing.

in future version, I will use rosbridge for the non ros native devices.

## Simulation Cases (Initial Poses + Setpoints)
"Cases" live in the configuration files that define spawn locations and mission setpoints.

Primary case file:
- `src/px4_swarm_controller/config/drones.yaml`
  - `initial_positions`: spawn positions for each drone ID
  - `setpoints`: ordered lists of waypoints per drone (x, y, z, yaw)

Secondary case file (used by the web UI if px4_swarm_controller config is missing):
- `src/uav_web_control/config/config.yaml`
  - `initial_positions`: grid positions
  - `geo_origin`: map origin for lat/lon conversions

If you want a smaller demo case, copy and edit:
- `src/uav_web_control/config/uavs.yaml.org` -> `src/uav_web_control/config/uavs.yaml`

## Other Ways To Run The Simulator

### Option A: PX4 SITL multi-run script
PX4 includes a multi-vehicle script. The tmuxinator profile already calls it, but you can run it manually:

```bash
cd ~/PX4-Autopilot/Tools/simulation/gazebo-classic
./sitl_multiple_run.sh -n 20
```

### Option B: Simple 3-UAV script
```bash
bash src/install_scripts/multi_uavs_sim_env.sh
```

This opens GNOME Terminal tabs and starts 3 PX4 instances plus the XRCE agent.

## Map Calibration (SITL)
If your UAV GPS markers do not align with the map:
1. Open a UAV's **Open Mission** view.
2. Click the map where the UAV should appear.
3. Click **Calibrate Map**.

Notes:
- Calibration is per-UAV and stored in the browser (clears on reload).
- Goals are adjusted automatically using the calibration offset.

## Arm / Offboard Notes
- The **Arm** button publishes a ROS2 `px4_msgs/msg/VehicleCommand` to `/px4_<id>/fmu/in/vehicle_command`.
- Run the web app from a terminal where ROS2 is sourced.

## Repo Layout
- `src/mytmux/multi_tmux/.tmuxinator.yml`: tmux launcher
- `src/uav_web_control/`: Flask web UI + control endpoints
- `src/px4_swarm_controller/`: ROS2 control stack + configs
- `src/install_scripts/`: helper scripts

## Troubleshooting
- **No UAVs appear**: check `src/px4_swarm_controller/config/drones.yaml` or create `src/uav_web_control/config/uavs.yaml`.
- **Arm/Setpoint fails**: confirm ROS2 is sourced and `px4_msgs` is built/available.
- **Gazebo not launching**: verify PX4 build exists at `~/PX4-Autopilot/build/px4_sitl_default`.
