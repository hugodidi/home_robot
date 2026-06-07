#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WS="${PROJECT_ROOT}/ros2_ws"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Error: ROS 2 Jazzy not found in /opt/ros/jazzy/"
    exit 1
fi

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

TMP_DIR="${PROJECT_ROOT}/tmp"
mkdir -p "${TMP_DIR}"
DOMAIN="${WS}/src/home_robot_pddl/pddl/patrol_domain.pddl"
PROBLEM="${TMP_DIR}/home_robot_patrol_problem.pddl"

ros2 run home_robot_pddl generate_patrol_problem --output "${PROBLEM}" --start inicio

if command -v popf >/dev/null 2>&1; then
    SOLVER="popf"
elif command -v popf2 >/dev/null 2>&1; then
    SOLVER="popf2"
elif [ -x "/opt/ros/jazzy/lib/popf/popf" ]; then
    SOLVER="/opt/ros/jazzy/lib/popf/popf"
else
    echo "Error: POPF executable not found. Rebuild Docker image or install ros-jazzy-popf."
    exit 1
fi

echo "Running POPF planner"
echo "Domain:  ${DOMAIN}"
echo "Problem: ${PROBLEM}"
"${SOLVER}" "${DOMAIN}" "${PROBLEM}"
