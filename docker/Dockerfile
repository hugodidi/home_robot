# Use an optimized image that already includes desktop and VNC for Jazzy
FROM tiryoh/ros2-desktop-vnc:jazzy

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Switch to root to install system dependencies
USER root

# 1. Install hardware, voice and YASMIN build dependencies
RUN apt-get update && apt-get install -y \
    alsa-utils \
    libasound2-dev \
    libportaudio2 \
    python3-pip \
    python3-venv \
    curl \
    unzip \
    tar \
    python3-serial \
    vlc-bin \
    # Project-specific ROS 2 dependencies
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-tf2-ros \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rmw-cyclonedds-cpp \
    libasound2-plugins \
    pulseaudio-utils \
    # YASMIN FSM build dependencies
    pybind11-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Configure ALSA to use PulseAudio (the bridge with the host)
RUN echo 'pcm.!default { type pulse }' > /etc/asound.conf && \
    echo 'ctl.!default { type pulse }' >> /etc/asound.conf

# 2. Setup workspace
WORKDIR /home/ubuntu/home_robot

# Cache optimization: download voice models BEFORE copying the entire code
# This way, the 150MB download is not repeated if a Python line is changed
COPY scripts/install_voice_models.sh scripts/
RUN chmod +x scripts/install_voice_models.sh && \
    ./scripts/install_voice_models.sh

# 3. Install Python dependencies (before code for better stability)
RUN pip3 install --break-system-packages --ignore-installed \
    vosk \
    sounddevice \
    pyserial \
    pyyaml \
    # YASMIN Viewer web UI dependencies
    flask \
    flask-socketio \
    waitress \
    expiringdict

# 4. Copy the rest of the project
COPY . .

# Ensure ubuntu user has audio and serial permissions
RUN usermod -aG audio,dialout ubuntu

# Ensure ubuntu user owns the workspace folder
RUN chown -R ubuntu:ubuntu /home/ubuntu/home_robot

# 5. Clone and build YASMIN in a separate workspace (/opt/yasmin_ws)
#    We do this outside of home_robot because the host mounts its own ros2_ws at runtime.
RUN mkdir -p /opt/yasmin_ws/src && \
    cd /opt/yasmin_ws/src && \
    git clone --depth 1 https://github.com/uleroboticsgroup/yasmin.git && \
    cd /opt/yasmin_ws && \
    . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 6. Build ROS 2 Workspace (home_robot + home_robot_fsm)
RUN . /opt/ros/jazzy/setup.sh && \
    . /opt/yasmin_ws/install/setup.sh && \
    cd /home/ubuntu/home_robot/ros2_ws && \
    rm -rf build install log && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 7. Environment configuration
ENV PROJECT_ROOT=/home/ubuntu/home_robot
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source setups automatically in ubuntu user's .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /opt/yasmin_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/home_robot/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export PROJECT_ROOT=/home/ubuntu/home_robot" >> /home/ubuntu/.bashrc

# Let the base image's ENTRYPOINT handle permissions and startup
# We don't force 'ubuntu' user here to avoid permission errors in logs
