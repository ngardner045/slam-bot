FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONPATH=/workspace/src

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    htop \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python \
    pyserial \
    pynput \
    matplotlib \
    scipy

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-urdf \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-robot-nav-tutorials \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Install additional tools
RUN apt-get update && apt-get install -y \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-plot \
    ros-humble-rqt-console \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Source ROS2 setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set default command
CMD ["/bin/bash"]
