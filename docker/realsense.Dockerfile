# syntax=docker/dockerfile:1
FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
# Helpful at runtime (not strictly required once we force LIBUVC build)
ENV REALSENSE_USB_BACKEND=libusb

# ------------------------------
# Base build and ROS tools
# ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl unzip pkg-config udev \
    python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep \
    libssl-dev libusb-1.0-0-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libgtk-3-dev \
    # deps that realsense-ros needs
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-diagnostic-updater \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rclcpp-components \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------
# Build librealsense (LIBUVC backend)
# Match a known-good tag used by recent realsense-ros (adjust if needed)
# ------------------------------
ARG LRS_TAG=v2.56.4
WORKDIR /opt
RUN git clone --depth=1 --branch ${LRS_TAG} https://github.com/IntelRealSense/librealsense.git

WORKDIR /opt/librealsense
# Install udev rules inside the container (host rules still recommended)
RUN cp config/99-realsense-libusb.rules /etc/udev/rules.d/ || true && \
    cp config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/ || true

WORKDIR /opt/librealsense/build
RUN cmake .. -DFORCE_LIBUVC=ON -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && ldconfig

# ------------------------------
# Build realsense-ros from source
# ------------------------------
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws/src
# Correct branch (contains realsense2_camera)
RUN git clone --depth=1 -b ros2-master https://github.com/IntelRealSense/realsense-ros.git

WORKDIR /opt/ros_ws
# If you prefer rosdep, uncomment the lines below:
# RUN rosdep init || true && rosdep update
# RUN . /opt/ros/humble/setup.sh && rosdep install --rosdistro humble --from-paths src -i -y --skip-keys=librealsense2

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --event-handlers console_direct+

# Source overlays for interactive shells
RUN echo '. /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo '. /opt/ros_ws/install/setup.bash' >> /root/.bashrc

# Use the ROS entrypoint so /opt/ros/humble is sourced
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
