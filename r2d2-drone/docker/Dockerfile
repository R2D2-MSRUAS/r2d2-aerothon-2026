FROM osrf/ros:jazzy-desktop-full

ENV LIBGL_ALWAYS_INDIRECT=0
ENV QT_X11_NO_MITSHM=1
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

###########################################################
# Update
###########################################################

RUN apt-get update && apt-get upgrade -y

###########################################################
# ROS Core
###########################################################

RUN apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-localization \
    ros-jazzy-tf2-tools \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-image-view

###########################################################
# Gazebo Harmonic
###########################################################

RUN apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control

###########################################################
# Vision
###########################################################

RUN apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-pipeline \
    ros-jazzy-camera-info-manager \
    ros-jazzy-depth-image-proc \
    ros-jazzy-vision-opencv

###########################################################
# Point Cloud
###########################################################

RUN apt-get install -y \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions

###########################################################
# Mapping
###########################################################

RUN apt-get install -y \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-ros

###########################################################
# MAVROS (ArduPilot)
###########################################################

RUN apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras

###########################################################
# GeographicLib
###########################################################

RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

###########################################################
# Development
###########################################################

RUN apt-get install -y \
    build-essential \
    cmake \
    git \
    gdb \
    clang \
    clang-format \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-dev \
    python3-setuptools \
    python3-wheel

###########################################################
# Utilities
###########################################################

RUN apt-get install -y \
    nano \
    vim \
    tree \
    htop \
    curl \
    wget \
    unzip \
    zip \
    tmux \
    xterm \
    net-tools \
    iputils-ping \
    usbutils \
    lsb-release

###########################################################
# Python
###########################################################

RUN apt-get install -y \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-pandas \
    python3-opencv \
    python3-pyzbar \
    python3-yaml

RUN pip3 install --no-cache-dir --break-system-packages \
    transforms3d \
    pyquaternion \
    qrcode

###########################################################
# ROSDEP
###########################################################

RUN rosdep init || true && rosdep update

###########################################################
# Workspace
###########################################################

RUN mkdir -p /root/drone_ws/src

WORKDIR /root/drone_ws

###########################################################
# Bash
###########################################################

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

RUN echo '[ -f /root/drone_ws/install/setup.bash ] && source /root/drone_ws/install/setup.bash' >> /root/.bashrc

RUN echo "alias cw='cd /root/drone_ws'" >> /root/.bashrc
RUN echo "alias cs='cd /root/drone_ws/src'" >> /root/.bashrc
RUN echo "alias cb='cd /root/drone_ws && colcon build --symlink-install'" >> /root/.bashrc
RUN echo "alias sb='source install/setup.bash'" >> /root/.bashrc

###########################################################
# Clean
###########################################################

RUN apt-get clean && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]
