# Home Robot Navigation System - Command Reference

This document is a comprehensive guide for operating the **Home Robot** system using ROS 2 Jazzy. While the system is fully containerized, you can interact with it using Docker commands or standard ROS 2 commands from within the container's web terminal.

---

## 🐳 Running with Docker (Recommended)

The entire system is pre-configured to run in a portable Docker environment. This eliminates the need for manual dependency installation.

### 1. Start/Stop the System
From your **host computer's terminal**:
```bash
# Start the system in background
docker compose -f docker/docker-compose.yml up -d

# Stop the system
docker compose -f docker/docker-compose.yml down
```

### 2. Access the VNC Desktop
Open your browser at **[http://localhost:6080](http://localhost:6080)**.
*   **Username**: `ubuntu`
*   **Password**: `roser` (if prompted)

---

## 🚀 Navigation & Simulation (Inside Container)

Once inside the container's terminal (via the web browser), use these commands:

### 2.1 Start Full Navigation Stack
```bash
# Option A: Map Localization (uses existing map)
ros2 launch home_robot main_nav.launch.py

# Option B: SLAM Mapping (builds new map from scratch)
ros2 launch home_robot main_nav.launch.py use_slam:=true

# Use the provided launch script for nav2 and rviz cleanup if needed
./scripts/launch_navigation.sh
```

> [!TIP]
> This launch includes: Gazebo simulation, Nav2 stack, SLAM (if enabled), RViz2 visualization with pre-loaded map, and the tb3-burger model with a "few" modifications.

---

## 🎤 Voice Control & Patrols

You can control the robot using your voice (Microphone required) or automated patrols.

### 3.1 Launch Voice Controller
```bash
ros2 run home_robot voice_controller

# Use the provided launch script for audio cleanup if needed
./scripts/launch_voice.sh
```
*   **Commands**: *"Ve al sofá"*, *"Patrulla"*, *"Para"*, *"Guardar mapa"*.
*   **Logic**: Uses Vosk (Offline STT) and Piper (TTS).

> [!TIP]
> Say "Opciones" to let the robot list the available commands.

### 3.2 Automated Waypoint Patrol

Also, you can ask the robot to patrol through the waypoints manually.

```bash
# Sequential patrol (Sofa -> Kitchen -> ... -> Home)
ros2 run home_robot execute_patrol

# Random patrol
ros2 run home_robot execute_patrol --random
```

## 🛠️ Hardware & Tools

### 4.1 LiDAR Grid (Arduino Interface)
If you have the physical Arduino with the LED matrix:
```bash
# Identify port (usually /dev/ttyUSB0 or /dev/ttyACM0)
ros2 run home_robot lidar_grid_node --port /dev/{your_port}
```

### 4.2 Save Map
```bash
# Service call (replace 'my_map' with your name)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/ubuntu/home_robot/ros2_ws/src/home_robot/maps/my_map'}}"
```

### 4.3 Manual Control (Teleop)
If you need to drive the robot manually using your keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


