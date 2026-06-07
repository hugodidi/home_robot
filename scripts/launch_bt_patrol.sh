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

echo "Building home_robot_bt package..."
cd "${WS}"
colcon build --symlink-install --packages-select home_robot_bt
cd "${PROJECT_ROOT}"

if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
fi

RANDOM_ARG="false"
SKIP_ERRORS_ARG="false"
BT_XML_ARG="${PROJECT_ROOT}/ros2_ws/src/home_robot_bt/config/bt_patrol.xml"
GROOT_PORT_ARG="1667"
OPEN_GROOT="true"

for arg in "$@"; do
    case "$arg" in
        --random) RANDOM_ARG="true" ;;
        --skip-errors) SKIP_ERRORS_ARG="true" ;;
        --bt-xml=*) BT_XML_ARG="${arg#*=}" ;;
        --groot-port=*) GROOT_PORT_ARG="${arg#*=}" ;;
        --no-groot) OPEN_GROOT="false" ;;
    esac
done

echo "Launching BT patrol: random=${RANDOM_ARG}, skip_errors=${SKIP_ERRORS_ARG}"

echo "Stopping previous BT patrol instances..."
pkill -f "home_robot_bt.*bt_patrol" 2>/dev/null || true
pkill -f "ros2 run home_robot_bt bt_patrol" 2>/dev/null || true
pkill -f "bt_patrol --ros-args" 2>/dev/null || true
sleep 1

if [ -n "${BT_XML_ARG}" ] && [ ! -f "${BT_XML_ARG}" ]; then
    echo "BT XML not found at ${BT_XML_ARG}; falling back to Nav2 default BT."
    BT_XML_ARG=""
fi

if [ -n "${BT_XML_ARG}" ]; then
    echo "BT XML: ${BT_XML_ARG}"
else
    echo "BT XML: Nav2 default"
fi
echo "Groot2 live port: ${GROOT_PORT_ARG}"

ros2 launch home_robot_bt bt_patrol.launch.py \
    random:="${RANDOM_ARG}" \
    skip_errors:="${SKIP_ERRORS_ARG}" \
    bt_xml:="${BT_XML_ARG}" \
    groot_port:="${GROOT_PORT_ARG}" &
LAUNCH_PID=$!

cleanup() {
    kill "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup INT TERM

if [ "${OPEN_GROOT}" = "true" ]; then
    echo "Waiting for Groot2 live server on port ${GROOT_PORT_ARG}..."
    for _ in $(seq 1 40); do
        if timeout 1 bash -c "</dev/tcp/127.0.0.1/${GROOT_PORT_ARG}" >/dev/null 2>&1; then
            break
        fi
        sleep 0.5
    done

    if command -v Groot2 >/dev/null 2>&1; then
        echo "Opening Groot2..."
        Groot2 "${BT_XML_ARG}" >/tmp/home_robot_groot2.log 2>&1 &
    elif command -v groot2 >/dev/null 2>&1; then
        echo "Opening groot2..."
        groot2 "${BT_XML_ARG}" >/tmp/home_robot_groot2.log 2>&1 &
    else
        echo "Groot2 executable not found in the container."
        if [ -n "${BT_XML_ARG}" ]; then
            echo "Open Groot2 and connect to port ${GROOT_PORT_ARG}. XML: ${BT_XML_ARG}"
        fi
    fi
    echo "If Groot2 does not auto-connect, use Monitor -> Connect -> localhost:${GROOT_PORT_ARG}."
fi

wait "${LAUNCH_PID}"
