#!/bin/bash
###############################################################################
# Home Robot Navigation System Launcher
#
# This script launches the home robot's navigation simulation system with
# automatic cleanup of previous processes. It handles both SLAM mapping
# and localization modes.
#
# Usage:
#   ./scripts/launch_navigation.sh [--slam]
#
# Options:
#   --slam    Enable SLAM mode for mapping (default: localization mode)
#
# Features:
#   - Automatic cleanup of orphan processes
#   - Lock file to prevent multiple instances
#   - Gazebo, Nav2, and ROS2 node management
#   - Graceful signal handling (SIGINT, SIGTERM, EXIT)
###############################################################################

# This script must be executed, not sourced. If sourced (". script.sh"), any
# internal "exit" would close  your terminal session.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
	echo "ERROR: Do not source this script. Run it like: ./scripts/launch_navigation.sh [--slam]"
	return 1
fi

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Prevent multiple concurrent instances (common cause of duplicate node names)
LOCKFILE="/tmp/home_robot_navigation.lock"
exec 9>"$LOCKFILE"
if ! flock -n 9; then
	echo "ERROR: Another navigation instance is already running (lock: $LOCKFILE)."
	echo "Close the other launch terminal (or kill its ros2 launch) before starting again."
	exit 1
fi

die() {
	echo "ERROR: $*"
	exit 1
}

cleanup() {
	set +e
	echo "Cleaning previous processes..."
	# Gazebo (gz sim is implemented as ruby processes)
	pkill -9 -f "gz sim" 2>/dev/null
	killall -9 gz rviz2 ruby 2>/dev/null

	# ROS launch + common nodes
	pkill -9 -f "ros2 launch" 2>/dev/null
	pkill -9 -f "odom_to_tf" 2>/dev/null
	pkill -9 -f "slam_toolbox" 2>/dev/null
	pkill -9 -f "nav2" 2>/dev/null
	pkill -9 -f "parameter_bridge" 2>/dev/null
	pkill -9 -f "robot_state_publisher" 2>/dev/null
	pkill -9 -f "static_transform_publisher" 2>/dev/null
	pkill -9 -f "map_to_odom_static_tf" 2>/dev/null

	# home_robot nodes that were being left behind and eating CPU
	pkill -9 -f "home_robot.*navigation_service" 2>/dev/null
	pkill -9 -f "navigation_service --ros-args" 2>/dev/null
	pkill -9 -f "navigation_service" 2>/dev/null
	pkill -9 -f "home_robot.*overhead_cam_service" 2>/dev/null
	pkill -9 -f "overhead_cam_service" 2>/dev/null
	set -e
}

# Ensure we don't leave orphan processes behind
trap cleanup EXIT INT TERM

# Ensure we run the *current* workspace overlay (avoid picking up legacy installs)
# Reset overlay-related vars so other workspaces (including ./install) don't win
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Source ROS2 Jazzy base installation
if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
	source /opt/ros/jazzy/setup.bash
fi

# Source workspace overlay
if [[ -f "$PROJECT_ROOT/ros2_ws/install/setup.bash" ]]; then
	source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
else
	die "Missing $PROJECT_ROOT/ros2_ws/install/setup.bash. Build: cd $PROJECT_ROOT/ros2_ws && colcon build --packages-select home_robot --symlink-install"
fi

echo "========================================="
echo "  Autonomous Navigation System"
echo "========================================="

# 1. Clean previous processes
cleanup

sleep 3
echo "Cleaning complete. Starting simulation..."

# Parse command line arguments
USE_SLAM=false
if [[ "$1" == "--slam" ]]; then
	USE_SLAM=true
	shift
fi

# Launch main navigation system
ros2 launch home_robot main_nav.launch.py use_slam:=$USE_SLAM

# launches navigation, odom_to_tf, parameter_bridge, robot_state_publisher; SLAM is optional with --slam

