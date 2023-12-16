FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y curl lsb-release gnupg2 software-properties-common && \
    add-apt-repository universe && \
    apt clean

# ROS 2 Foxy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt upgrade -y && \
    apt install -y ros-foxy-ros-base python3-argcomplete ros-dev-tools ros-foxy-image-transport libopencv-dev ros-foxy-cv-bridge && \
    apt clean

ENV ROS2_PATH=/opt/ros/foxy/

WORKDIR /root/workspace
