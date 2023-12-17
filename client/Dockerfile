FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    net-tools \
    iputils-ping \
    python3-pip \
    libgl1-mesa-glx \
    libqt5x11extras5 \
    && apt clean

RUN pip3 install opencv-python

WORKDIR /root/workspace
