#!/bin/bash
# ─────────────────────────────────────────────────────────────────────
# launch_fsm_patrol.sh — Convenience script for FSM Patrol
# ─────────────────────────────────────────────────────────────────────

# NO usar set -e por ahora para permitir depuración
# set -e 

echo "════════════════════════════════════════════════════════════════"
echo " 🤖 Home Robot — FSM Patrol (YASMIN)"
echo "════════════════════════════════════════════════════════════════"

# Detectar la raíz del proyecto de forma dinámica
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WS="${PROJECT_ROOT}/ros2_ws"

echo "📍 Project Root: ${PROJECT_ROOT}"

# Sourcing de ROS 2 y dependencias del sistema
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "❌ Error: ROS 2 Jazzy not found in /opt/ros/jazzy/"
    exit 1
fi

if [ -f "/opt/yasmin_ws/install/setup.bash" ]; then
    source /opt/yasmin_ws/install/setup.bash
fi

# Construcción del paquete FSM para evitar ejecutar una instalación obsoleta
echo "⏳ Building home_robot_fsm package..."
cd "${WS}"
colcon build --symlink-install --packages-select home_robot_fsm
cd "${PROJECT_ROOT}"

# Sourcing del espacio de trabajo
if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
else
    echo "⚠️ Warning: Workspace setup.bash not found. Attempting to continue..."
fi

# Parse de argumentos
RANDOM_ARG="false"
SKIP_ERRORS_ARG="false"

for arg in "$@"; do
    case "$arg" in
        --random) RANDOM_ARG="true" ;;
        --skip-errors) SKIP_ERRORS_ARG="true" ;;
    esac
done

echo "📋 Random: ${RANDOM_ARG} | Skip Errors: ${SKIP_ERRORS_ARG}"
echo "🚀 Launching ROS 2 node..."

# Launch firefox automatically only if it is NOT already running
if command -v firefox &> /dev/null; then
    if ! pgrep firefox > /dev/null; then
        echo "🌍 Opening YASMIN Web Viewer..."
        # Disown to avoid blocking the ROS launch
        firefox http://localhost:5000 >/dev/null 2>&1 &
    else
        echo "🌍 Firefox is already running; skipping automatic viewer tab."
    fi
fi

# Lanzar usando el comando completo de ROS 2
ros2 launch home_robot_fsm fsm_patrol.launch.py \
    random:="${RANDOM_ARG}" \
    skip_errors:="${SKIP_ERRORS_ARG}"
