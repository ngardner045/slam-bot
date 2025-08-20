# 📦 ROS2 Package Availability on Raspberry Pi (ARM64)

## 🚨 **Important Note for ARM64 Architecture**

Some ROS2 packages are not available for ARM64 architecture on Raspberry Pi. This document explains what's available and provides alternatives.

## ✅ **Available Packages (ARM64 Compatible)**

### **Core ROS2 Packages**
- `ros-humble-ros-base` ✅
- `ros-humble-robot-state-publisher` ✅
- `ros-humble-joint-state-publisher` ✅
- `ros-humble-urdf` ✅
- `ros-humble-xacro` ✅
- `ros-humble-robot-localization` ✅
- `ros-humble-navigation2` ✅
- `ros-humble-nav2-bringup` ✅
- `ros-humble-teleop-twist-keyboard` ✅
- `ros-humble-rviz2` ✅
- `ros-humble-rqt` ✅
- `ros-humble-rqt-graph` ✅
- `ros-humble-rqt-plot` ✅
- `ros-humble-rqt-console` ✅

## ❌ **Unavailable Packages (ARM64)**

### **Missing Packages**
- `ros-humble-robot-nav-tutorials` ❌
- `ros-humble-rtabmap-ros` ❌
- `ros-humble-realsense2-camera` ❌
- `ros-humble-realsense2-description` ❌
- `ros-humble-gazebo-ros-pkgs` ❌
- `ros-humble-gazebo-ros2-control` ❌
- `ros-humble-plotjuggler` ❌
- `ros-humble-plotjuggler-ros` ❌

## 🔧 **Alternatives and Solutions**

### **1. RTAB-Map Alternative**
Since `rtabmap-ros` isn't available for ARM64, you can:

#### **Option A: Build from Source**
```bash
# Inside the Docker container
cd /tmp
git clone https://github.com/introlab/rtabmap.git
cd rtabmap
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
make install
```

#### **Option B: Use Basic SLAM with Navigation2**
```bash
# Navigation2 includes basic SLAM capabilities
ros2 launch nav2_bringup bringup_launch.py
```

### **2. RealSense Camera Alternative**
Since `realsense2_camera` isn't available for ARM64:

#### **Option A: Build RealSense SDK from Source**
The Dockerfile already includes this:
```dockerfile
# Build and install RealSense SDK from source for ARM64
RUN cd /tmp && \
    git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd / && rm -rf /tmp/librealsense
```

#### **Option B: Use OpenCV for Camera Access**
```python
import cv2
import numpy as np

# Access camera directly
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
```

### **3. Gazebo Alternative**
Since Gazebo packages aren't available for ARM64:

#### **Option A: Use RViz for Visualization**
```bash
# RViz provides excellent visualization capabilities
ros2 run rviz2 rviz2
```

#### **Option B: Use Web-based Simulators**
- **Webots**: Has ARM64 support
- **CoppeliaSim**: Can run headless on Pi

## 🚀 **Updated Docker Build**

The Dockerfiles have been updated to only install available packages. To rebuild:

```bash
# Stop existing containers
docker-compose -f docker-compose.pi.yml down

# Rebuild with updated Dockerfiles
docker-compose -f docker-compose.pi.yml up -d --build
```

## 📋 **What Still Works**

### **Core Functionality**
- ✅ **Robot Description**: URDF/XACRO loading
- ✅ **Motor Control**: RoboClaw interface
- ✅ **Navigation**: Nav2 stack
- ✅ **Visualization**: RViz2
- ✅ **Basic SLAM**: Navigation2 built-in capabilities

### **Missing Functionality**
- ❌ **Advanced SLAM**: RTAB-Map features
- ❌ **Simulation**: Gazebo integration
- ❌ **Camera Integration**: RealSense ROS2 packages

## 🔄 **Workarounds**

### **1. For SLAM**
Use Navigation2's built-in SLAM capabilities:
```bash
# Basic SLAM with Navigation2
ros2 launch nav2_bringup bringup_launch.py slam:=True
```

### **2. For Camera Data**
Access camera directly and publish to ROS2 topics:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        
    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(msg)
```

### **3. For Advanced Features**
Consider using a development machine (x86_64) for:
- Advanced SLAM algorithms
- Simulation and testing
- Package development

## 📚 **Resources**

### **ARM64 Compatible Packages**
- [ROS2 Humble ARM64 Packages](https://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/)
- [Navigation2 ARM64 Support](https://navigation.ros.org/)

### **Building from Source**
- [RTAB-Map Build Instructions](https://github.com/introlab/rtabmap/wiki/Installation)
- [RealSense SDK Build](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

## 🎯 **Recommendations**

1. **Start with basic functionality** using available packages
2. **Build missing packages from source** if needed
3. **Use alternatives** where possible
4. **Consider hybrid approach**: Pi for control, x86_64 for advanced features

## 🔍 **Checking Package Availability**

To check if a package is available for ARM64:
```bash
# Check package availability
apt-cache search ros-humble-package-name

# Check specific architecture
apt-cache show ros-humble-package-name | grep Architecture
```

## 📞 **Getting Help**

If you need specific functionality that's not available:
1. **Check package alternatives**
2. **Build from source**
3. **Use development machine for testing**
4. **Consider different approaches**

The SLAM Bot will still work with the available packages, providing core robot control and navigation capabilities!
