#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WS="${PROJECT_ROOT}/ros2_ws"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Error: ROS 2 Jazzy not found in /opt/ros/jazzy/"
    exit 1
fi

echo "Building home_robot_pddl package..."
cd "${WS}"
colcon build --symlink-install --packages-select home_robot_pddl
cd "${PROJECT_ROOT}"

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

RANDOM_ARG="false"
SKIP_ERRORS_ARG="false"

for arg in "$@"; do
    case "$arg" in
        --random) RANDOM_ARG="true" ;;
        --skip-errors) SKIP_ERRORS_ARG="true" ;;
    esac
done

echo "Launching PDDL patrol: random=${RANDOM_ARG}, skip_errors=${SKIP_ERRORS_ARG}"
echo "Plan viewer: http://localhost:8080"

# Kill any stale UI server from previous runs
pkill -f "pddl_ui" 2>/dev/null || true
sleep 0.5

# Start UI server in the background
ros2 run home_robot_pddl pddl_ui &
UI_PID=$!
sleep 1

# Try to open browser automatically (best-effort)
(xdg-open http://localhost:8080 || google-chrome http://localhost:8080 || firefox http://localhost:8080) 2>/dev/null || true

# Kill UI server on script exit
trap "echo 'Shutting down UI server...'; kill $UI_PID 2>/dev/null; wait $UI_PID 2>/dev/null; exit" SIGINT SIGTERM EXIT

ros2 launch home_robot_pddl pddl_patrol.launch.py \
    random:="${RANDOM_ARG}" \
    skip_errors:="${SKIP_ERRORS_ARG}"
