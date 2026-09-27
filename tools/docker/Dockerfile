FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-message-filters python3-numpy python3-scipy python3-pyproj \
    python3-matplotlib python3-pytest \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
