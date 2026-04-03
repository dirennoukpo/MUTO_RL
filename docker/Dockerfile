FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive

# ────────────────────────────────────────────────────────────────
# Locale UTF-8 (requis pour ROS 2 dans un container minimal)
# ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# ────────────────────────────────────────────────────────────────
# 1. Base ROS 2 + outils de développement
# ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-argcomplete \
    python3-rosinstall-generator \
    \
# ────────────────────────────────────────────────────────────────
# 2. Navigation / Localisation / SLAM
# ────────────────────────────────────────────────────────────────
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    \
# ────────────────────────────────────────────────────────────────
# 3. Description robot / TF / diagnostics
# ────────────────────────────────────────────────────────────────
    ros-humble-xacro \
    ros-humble-urdf \
    ros-humble-urdfdom-py \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-tools \
    ros-humble-diagnostic-updater \
    \
# ────────────────────────────────────────────────────────────────
# 4. Contrôle robot (ros2_control)
# ────────────────────────────────────────────────────────────────
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-transmission-interface \
    \
# ────────────────────────────────────────────────────────────────
# 5. Perception / Vision / Pointcloud
# ────────────────────────────────────────────────────────────────
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-pipeline \
    ros-humble-compressed-image-transport \
    ros-humble-depthimage-to-laserscan \
    ros-humble-laser-filters \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-perception-pcl \
    ros-humble-sensor-msgs-py \
    \
# ────────────────────────────────────────────────────────────────
# 6. Simulation — Gazebo Fortress (Gazebo Classic EOL jan. 2025)
# ────────────────────────────────────────────────────────────────
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-gz-ros2-control \
    \
# ────────────────────────────────────────────────────────────────
# 7. Visualisation
# ────────────────────────────────────────────────────────────────
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-robot-monitor \
    ros-humble-rqt-image-view \
    \
# ────────────────────────────────────────────────────────────────
# 8. Toolchain C / C++
# ────────────────────────────────────────────────────────────────
    build-essential \
    cmake \
    ninja-build \
    ccache \
    pkg-config \
    gdb \
    gdbserver \
    clang \
    clangd \
    clang-format \
    clang-tidy \
    cppcheck \
    lcov \
    \
# ────────────────────────────────────────────────────────────────
# 9. Python — runtime, venv et outils système
# ────────────────────────────────────────────────────────────────
    python3-pip \
    python3-venv \
    python3-dev \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-pandas \
    python3-opencv \
    python3-pytest \
    python3-coverage \
    python3-flake8 \
    \
# ────────────────────────────────────────────────────────────────
# 10. Bibliothèques natives robotique
# ────────────────────────────────────────────────────────────────
    libasio-dev \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libtinyxml2-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libboost-all-dev \
    can-utils \
    i2c-tools \
    v4l-utils \
    \
# ────────────────────────────────────────────────────────────────
# 11. Utilitaires système / réseau / debug
# ────────────────────────────────────────────────────────────────
    bash-completion \
    git \
    git-lfs \
    tmux \
    vim \
    nano \
    less \
    htop \
    tree \
    iproute2 \
    iputils-ping \
    net-tools \
    usbutils \
    pciutils \
    lsb-release \
    gnupg2 \
    software-properties-common \
    x11-apps \
    netcat-openbsd \
    curl \
    wget \
    rsync \
    iptables \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# ────────────────────────────────────────────────────────────────
# 12. Tailscale — installation via le dépôt officiel
# ────────────────────────────────────────────────────────────────
RUN curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.noarmor.gpg \
        | tee /usr/share/keyrings/tailscale-archive-keyring.gpg > /dev/null \
    && curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/jammy.tailscale-keyring.list \
        | tee /etc/apt/sources.list.d/tailscale.list \
    && apt-get update \
    && apt-get install -y tailscale \
    && rm -rf /var/lib/apt/lists/*

# ────────────────────────────────────────────────────────────────
# 13. Venv dédié au projet
# ────────────────────────────────────────────────────────────────
ENV VENV_PATH=/opt/tekbot_venv
RUN python3 -m venv --system-site-packages "$VENV_PATH" \
    && touch "$VENV_PATH/COLCON_IGNORE"

ENV PATH="$VENV_PATH/bin:$PATH"

RUN pip install --no-cache-dir --upgrade pip setuptools wheel

# empy 3.3.4 OBLIGATOIRE pour colcon/ament Humble
RUN pip install --no-cache-dir \
    "empy==3.3.4" \
    pytest-repeat \
    pytest-rerunfailures \
    catkin-pkg \
    lark \
    transforms3d \
    pyserial \
    pyyaml \
    requests

# ────────────────────────────────────────────────────────────────
# 14. Initialisation rosdep
# ────────────────────────────────────────────────────────────────
RUN rosdep init || true && rosdep update --rosdistro humble

# ────────────────────────────────────────────────────────────────
# 15. Dossier de travail
# ────────────────────────────────────────────────────────────────
WORKDIR /home/pi/TEKBOT-ROBOTICS

# ────────────────────────────────────────────────────────────────
# 16. Configuration automatique du shell (Bash)
# ────────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f install/setup.bash ]; then source install/setup.bash; fi" >> ~/.bashrc && \
    echo "source $VENV_PATH/bin/activate" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc

# ────────────────────────────────────────────────────────────────
# 17. Script d'entrée
# ────────────────────────────────────────────────────────────────
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]