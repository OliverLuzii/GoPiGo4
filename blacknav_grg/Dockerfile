FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=blacknav
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

WORKDIR /home/blacknav

RUN --mount=target=/var/lib/apt/lists,type=cache,sharing=locked \
    --mount=target=/var/cache/apt,type=cache,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \
    clangd \
    wget \
    unzip \
    python3 \
    python3-dev \
    ca-certificates \
    udev \
    v4l-utils \
    ros-jazzy-realsense2-* \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/joan2937/pigpio/archive/master.zip \
    && unzip master.zip \
    && cd pigpio-master \
    && make \
    && sudo make install \
    && cd .. \
    && rm -rf pigpio-master

COPY arm64realsense_files/librealsense/bin/* /usr/local/bin/
COPY arm64realsense_files/librealsense/include/* /usr/local/include/
COPY arm64realsense_files/librealsense/lib/* /usr/local/lib/

RUN mkdir /usr/local/lib/python3.12/dist-packages/pyrealsense2/
COPY arm64realsense_files/pyrealsense2/* /usr/local/lib/python3.12/dist-packages/pyrealsense2/

COPY arm64realsense_files/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d
COPY arm64realsense_files/99-realsense-libusb.rules /etc/udev/rules.d

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN ldconfig

ENV SHELL /bin/bash
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID 1

USER $USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
