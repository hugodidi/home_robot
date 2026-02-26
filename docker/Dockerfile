# Usar una imagen optimizada que ya incluye escritorio y VNC para Jazzy
FROM tiryoh/ros2-desktop-vnc:jazzy

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# Cambiar a root para instalar dependencias de sistema
USER root

# 1. Instalar dependencias de hardware y voz
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
    # Dependencias de ROS 2 para el proyecto
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

# Configurar ALSA para que use PulseAudio (el puente con el host)
RUN echo 'pcm.!default { type pulse }' > /etc/asound.conf && \
    echo 'ctl.!default { type pulse }' >> /etc/asound.conf

# 2. Configurar el espacio de trabajo
WORKDIR /home/ubuntu/home_robot

# Optimización de caché: descargar modelos de voz ANTES de copiar todo el código
# Así, si cambias una línea de Python, no se vuelven a bajar los 150MB
COPY scripts/install_voice_models.sh scripts/
RUN chmod +x scripts/install_voice_models.sh && \
    ./scripts/install_voice_models.sh

# 3. Instalar dependencias de Python (antes del código para mayor estabilidad)
RUN pip3 install --break-system-packages \
    vosk \
    sounddevice \
    pyserial \
    pyyaml

# 4. Copiar el resto del proyecto
COPY . .

# Asegurar que el usuario ubuntu tenga permisos de audio y serial
RUN usermod -aG audio,dialout ubuntu

# Asegurar que el usuario ubuntu sea el dueño de su carpeta de trabajo
RUN chown -R ubuntu:ubuntu /home/ubuntu/home_robot

# 5. Compilar el Workspace de ROS 2
RUN . /opt/ros/jazzy/setup.sh && \
    cd ros2_ws && \
    rm -rf build install log && \
    colcon build --symlink-install

# 6. Configuración del entorno
ENV PROJECT_ROOT=/home/ubuntu/home_robot
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Cargar los setups automáticamente en el .bashrc del usuario ubuntu
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/home_robot/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export PROJECT_ROOT=/home/ubuntu/home_robot" >> /home/ubuntu/.bashrc

# Dejamos que el ENTRYPOINT de la imagen base gestione los permisos y el arranque
# No forzamos al usuario 'ubuntu' aquí para evitar errores de permisos en los logs
