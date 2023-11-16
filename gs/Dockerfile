FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    net-tools \
    iputils-ping \
    && apt clean

WORKDIR /root/workspace
