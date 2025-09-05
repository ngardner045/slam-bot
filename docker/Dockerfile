# ROS 2 Humble base (arm64 works on Pi 5)
FROM ros:humble-ros-base

# For Python serial and TF broadcaster deps already in ros-base
RUN apt-get update && apt-get install -y \
        python3-pip \
        ros-humble-joy \
        ros-humble-teleop-twist-joy \
        ros-humble-cv-bridge \
        python3-opencv \
        ros-humble-image-transport \
        && rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial

# Copy your node
WORKDIR /app

# Run the node (you can override params with ROS args)
CMD ["python3", "/app/roboclaw_base_node.py"]

SHELL ["/bin/bash","-lc"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
