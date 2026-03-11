#!/bin/bash

# Kill any existing PX4 instances
pkill -x px4 || true
sleep 1


cd
cd PX4-Autopilot


# Start MicroXRCEAgent first
gnome-terminal --tab --title="XRCE Agent" -- sh -c "MicroXRCEAgent udp4 -p 8888; bash"
sleep 2

# Launch three PX4 instances with proper namespace and system ID configuration
gnome-terminal --tab --title="PX4-1" -- sh -c "PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE='0,0' \
PX4_GZ_MODEL=x500 \
PX4_UXRCE_DDS_NS=px4_1 \
PX4_MAV_SYS_ID=1 \
./build/px4_sitl_default/bin/px4 -i 1; bash"
sleep 5

gnome-terminal --tab --title="PX4-2" -- sh -c "PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE='0,1' \
PX4_GZ_MODEL=x500 \
PX4_UXRCE_DDS_NS=px4_2 \
PX4_MAV_SYS_ID=2 \
./build/px4_sitl_default/bin/px4 -i 2; bash"
sleep 5

gnome-terminal --tab --title="PX4-3" -- sh -c "PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE='0,2' \
PX4_GZ_MODEL=x500 \
PX4_UXRCE_DDS_NS=px4_3 \
PX4_MAV_SYS_ID=3 \
./build/px4_sitl_default/bin/px4 -i 3; bash"
sleep 5

# # Launch the offboard control node after all PX4 instances are ready
# gnome-terminal --tab --title="Offboard Control" -- sh -c "ros2 run your_package multi_vehicle_offboard; bash"0