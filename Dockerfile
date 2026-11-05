FROM ros:foxy-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000

USER root

# Herramientas comunes a ambas imagenes.
RUN apt-get update && apt-get install -y \
    sudo \
    python3-pip \
    cmake \
    curl \
    wget \
    nano \
    vim \
    less \
    tree \
    htop \
    iputils-ping \
    net-tools \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Paquetes especificos de ROS Foxy y sus proyectos.
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    ros-foxy-gazebo-dev \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-nav2-bringup \
    ros-foxy-joint-state-publisher \
    ros-foxy-slam-toolbox \
    ros-foxy-depth-image-proc \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-xacro \
    libusb-1.0-0-dev \
    python2.7 \
    python3-serial \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Herramientas y cabeceras para reconstruir los paquetes desde un clon limpio.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    patch \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-yaml \
    python3-pytest \
    libboost-system-dev \
    libboost-thread-dev \
    libudev-dev \
    && rm -rf /var/lib/apt/lists/*

# SDK de Claude para rosbot_telegram_ai. Con Python 3.8 pip instala la última versión compatible.
RUN python3 -m pip install --no-cache-dir anthropic

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd \
        --uid ${USER_UID} \
        --gid ${USER_GID} \
        --create-home \
        --shell /bin/bash \
        ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
        > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /usr/share/gazebo/setup.bash" \
    >> /home/${USERNAME}/.bashrc \
    && echo "source /opt/ros/foxy/setup.bash" \
    >> /home/${USERNAME}/.bashrc

USER ${USERNAME}

# Claude Code para rosbot_telegram_ai (backend claude_code). Se inicia sesión
# una vez dentro del contenedor ejecutando `claude`.
RUN curl -fsSL https://claude.ai/install.sh | bash \
    && echo 'export PATH="$HOME/.local/bin:$PATH"' >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/carpeta_compartida

CMD ["bash"]
