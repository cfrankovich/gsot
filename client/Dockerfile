FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    curl \
    gnupg2 \
    software-properties-common \
    && apt clean

RUN curl -sL https://deb.nodesource.com/setup_20.x | bash -

RUN apt install -y nodejs nginx

WORKDIR /root/workspace

EXPOSE 3000 
