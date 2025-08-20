#!/bin/bash

echo "Setting up SLAM Bot ROS2 project..."

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo "Error: Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Create necessary directories
echo "Creating project directories..."
mkdir -p maps
mkdir -p config
mkdir -p launch

# Set proper permissions for USB devices (Linux only)
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Setting USB device permissions..."
    sudo usermod -a -G dialout $USER
    echo "Added user to dialout group. You may need to log out and back in."
fi

# Build Docker images
echo "Building Docker images..."
docker-compose build

echo ""
echo "Setup complete! To start the robot:"
echo "1. Connect your hardware (RoboClaw, RealSense camera)"
echo "2. Run: docker-compose up"
echo "3. In another terminal: docker exec -it slam-bot-ros bash"
echo "4. Inside container: ros2 launch slam_bot_bringup robot.launch.py"
echo ""
echo "For SLAM: ros2 launch slam_bot_navigation slam.launch.py"
echo "For navigation: ros2 launch slam_bot_navigation navigation.launch.py"
