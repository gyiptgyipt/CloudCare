#!/usr/bin/env bash
set -euo pipefail

# Runtime env setup for multi-device ROS usage.
# Usage:
#   source ./src/uav_web_control/scripts/ensure_runtime_env.sh 3
#   # or: UAV_ID=3 source ./src/uav_web_control/scripts/ensure_runtime_env.sh

DEVICE_ID="${1:-${UAV_ID:-}}"
if [ -z "${DEVICE_ID}" ]; then
  DEVICE_ID="0"
fi

export UAV_ID="${UAV_ID:-$DEVICE_ID}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$DEVICE_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "Runtime env ready:"
echo "  UAV_ID=${UAV_ID}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
