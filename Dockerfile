FROM ros:foxy-ros-base-focal

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=helio
ARG USER_UID=1000
ARG USER_GID=1000

USER root

RUN apt-get update && apt-get install -y \
    sudo \
    ros-foxy-desktop \
    ros-foxy-gazebo-dev \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-nav2-bringup \
    ros-foxy-joint-state-publisher \
    ros-foxy-slam-toolbox \
    ros-foxy-rmw-cyclonedds-cpp \
    ros-foxy-xacro \
    libusb-1.0-0-dev \
    python2.7 \
    python3-pip \
    python3-serial \
    mesa-utils \
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

WORKDIR /home/${USERNAME}/carpeta_compartida

CMD ["bash"]
