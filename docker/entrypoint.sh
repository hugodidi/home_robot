#!/bin/bash
set -e

# Setup VNC password
mkdir -p ~/.vnc
echo "roser" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Clean up old sockets
vncserver -kill :1 || true
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1

# Start VNC server with XFCE
# Resolution 1280x720 is usually good for simulation
vncserver :1 -geometry 1280x720 -depth 24 -localhost no

# Start noVNC (Proxy for VNC via browser) on port 6080
/usr/share/novnc/utils/launch.sh --vnc localhost:5901 --listen 6080 &

echo "VNC Server started at :1 (Port 5901)"
echo "noVNC (Browser) started at Port 6080"

# Source ROS environments
source /opt/ros/jazzy/setup.bash
source /home/ros/home_robot/ros2_ws/install/setup.bash

# Execute the passed command
exec "$@"
