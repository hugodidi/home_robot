# Use an optimized image that already includes desktop and VNC for Jazzy
FROM tiryoh/ros2-desktop-vnc:jazzy

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Switch to root to install system dependencies
USER root

# 1. Install hardware and voice dependencies
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
RUN pip3 install --break-system-packages \
    vosk \
    sounddevice \
    pyserial \
    pyyaml

# 4. Copy the rest of the project
COPY . .

# Ensure ubuntu user has audio and serial permissions
RUN usermod -aG audio,dialout ubuntu

# Ensure ubuntu user owns the workspace folder
RUN chown -R ubuntu:ubuntu /home/ubuntu/home_robot

# 5. Build ROS 2 Workspace
RUN . /opt/ros/jazzy/setup.sh && \
    cd ros2_ws && \
    rm -rf build install log && \
    colcon build --symlink-install

# 6. Environment configuration
ENV PROJECT_ROOT=/home/ubuntu/home_robot
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source setups automatically in ubuntu user's .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/home_robot/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export PROJECT_ROOT=/home/ubuntu/home_robot" >> /home/ubuntu/.bashrc

# Let the base image's ENTRYPOINT handle permissions and startup
# We don't force 'ubuntu' user here to avoid permission errors in logs
