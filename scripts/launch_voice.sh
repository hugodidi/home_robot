#!/bin/bash
###############################################################################
# Voice Controller Launcher
#
# This script launches the Spanish voice control system for the home robot.
# It uses a Python virtual environment for voice-specific dependencies
# (Vosk STT, sounddevice) while integrating with ROS2.
#
# Usage:
#   ./scripts/launch_voice.sh
#
# Prerequisites:
#   - Python virtual environment at PROJECT_ROOT/.venv
#   - Voice models installed (run ./scripts/install_voice_models.sh)
#   - ROS2 Jazzy workspace built
#
# Features:
#   - Automatic cleanup on Ctrl-C
#   - ALSA warnings filtered from output
#   - Virtual environment + ROS2 environment integration
###############################################################################

# This script must be executed, not sourced.
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
	echo "ERROR: Do not source this script. Run it like: ./scripts/launch_voice.sh"
	return 1
fi

# Cleanup function for graceful shutdown (Ctrl-C)
cleanup() {
    echo -e "\n🛑 Shutting down voice controller..."
    pkill -9 -f "voice_controller" 2>/dev/null || true
    exit 0
}

trap cleanup INT TERM

# Project root directory detection
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
echo "Project Root: $PROJECT_ROOT"

# Handle virtual environment vs system python
VENV_PATH="$PROJECT_ROOT/.venv/bin/activate"
if [ -f "$VENV_PATH" ]; then
    echo "Using virtual environment: $PROJECT_ROOT/.venv"
    source "$VENV_PATH"
elif [ -f /.dockerenv ]; then
    echo "Running inside Docker: Using system Python environment"
else
    echo "WARNING: Virtual environment not found. Attempting to use system Python."
fi

# 2. Configure ROS2 environment
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"

# 3. Launch voice controller node
# Filter out ALSA/Jack warnings from stderr to keep output clean
echo "Launching voice control node..."
ros2 run home_robot voice_controller 2> >(grep -v -E "ALSA|jack|Jack|pcm")

