FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-venv python3-dev \
    python3-distutils python3-setuptools \
    git curl unzip wget cmake ninja-build g++ \
    build-essential pkg-config libffi-dev libssl-dev \
    udev usbutils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# We need pipx so that the new platformio versions for command line can be installed
RUN pip3 install --no-cache-dir pipx && \
    pipx ensurepath && \
    pipx install platformio

ENV PATH="/root/.local/bin:/root/.platformio/penv/bin:$PATH"

# Zephyr SDK tools -> not sure if we need these
RUN pipx install west && \
    west init /opt/zephyr && cd /opt/zephyr && west update && west zephyr-export

VOLUME ["/dev"]
