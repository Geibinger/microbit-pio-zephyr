FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-venv python3-dev \
    python3-distutils python3-setuptools \
    git curl unzip wget cmake ninja-build g++ \
    build-essential pkg-config libffi-dev libssl-dev \
    udev usbutils clang-format \
  && pip3 install --upgrade platformio \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

VOLUME ["/dev"]
