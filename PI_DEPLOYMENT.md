# SLAM Bot Raspberry Pi Deployment Guide

This guide will help you deploy the SLAM Bot to your Raspberry Pi 5 using Docker.

## Prerequisites

- Raspberry Pi 5 with at least 8GB RAM (16GB recommended)
- MicroSD card with Ubuntu 22.04 LTS or Raspberry Pi OS
- Network connection (WiFi or Ethernet)
- RoboClaw 2x15A motor controller
- RealSense D455 camera
- 2x DC motors with encoders
- Differential drive robot chassis

## Quick Deployment

### 1. From Your Development Machine

```bash
# Make the deployment script executable
chmod +x deploy_to_pi.sh

# Deploy to Pi (replace with your Pi's IP address)
./deploy_to_pi.sh pi@192.168.1.100

# Or use hostname if available
./deploy_to_pi.sh pi@raspberrypi.local
```

### 2. Manual Deployment

If you prefer to deploy manually:

```bash
# Copy files to Pi
scp -r . pi@raspberrypi.local:~/slam-bot-deploy/

# SSH into Pi
ssh pi@raspberrypi.local

# Navigate to project directory
cd ~/slam-bot-deploy

# Install Docker and dependencies
sudo apt update
sudo apt install -y docker.io docker-compose

# Add user to docker group
sudo usermod -a -G docker $USER

# Log out and back in for group changes
exit
ssh pi@raspberrypi.local

# Build and run
docker-compose -f docker-compose.pi.yml build
docker-compose -f docker-compose.pi.yml up -d
```

## Hardware Setup

### 1. RoboClaw Motor Controller

- Connect RoboClaw to Pi via USB
- Connect left motor to M1, right motor to M2
- Connect encoders to respective channels
- Power the RoboClaw with appropriate voltage (check motor specs)

### 2. RealSense D455 Camera

- Connect camera to USB 3.0 port
- Ensure stable mounting on robot chassis
- Check camera orientation (should face forward)

### 3. Power Supply

- Use a high-quality power supply (5V/3A minimum)
- Consider using a power distribution board
- Ensure stable voltage for motors and Pi

## Configuration

### 1. Motor Parameters

Edit `config/motor_params.yaml` to match your hardware:

```yaml
motor_controller:
  ros__parameters:
    wheel_diameter: 0.065  # meters
    wheel_separation: 0.25  # meters
    encoder_ticks_per_rev: 1440  # adjust for your encoders
    max_linear_velocity: 1.0  # m/s
    max_angular_velocity: 2.0  # rad/s
```

### 2. Camera Parameters

The RealSense camera will auto-detect, but you can adjust settings in the launch files if needed.

## Usage

### 1. Start the Robot

```bash
# Access the ROS container
docker exec -it slam-bot-ros bash

# Inside container, start the robot
ros2 launch slam_bot_bringup robot.launch.py
```

### 2. Start SLAM

```bash
# In another terminal, access container
docker exec -it slam-bot-ros bash

# Start SLAM
ros2 launch slam_bot_navigation slam.launch.py
```

### 3. Start Navigation

```bash
# In another terminal, access container
docker exec -it slam-bot-ros bash

# Start navigation (requires a map first)
ros2 launch slam_bot_navigation navigation.launch.py
```

### 4. Teleop Control

```bash
# In another terminal, access container
docker exec -it slam-bot-ros bash

# Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Docker Commands

### View Logs
```bash
# View all container logs
docker-compose -f docker-compose.pi.yml logs -f

# View specific service logs
docker-compose -f docker-compose.pi.yml logs -f slam-bot-ros
```

### Container Management
```bash
# Stop containers
docker-compose -f docker-compose.pi.yml down

# Restart containers
docker-compose -f docker-compose.pi.yml restart

# Rebuild and restart
docker-compose -f docker-compose.pi.yml up -d --build
```

### Access Containers
```bash
# Access ROS container
docker exec -it slam-bot-ros bash

# Access GUI container
docker exec -it slam-bot-gui bash
```

## Troubleshooting

### 1. Permission Issues

```bash
# Fix USB permissions
sudo usermod -a -G dialout,video $USER
# Log out and back in
```

### 2. Camera Not Detected

```bash
# Check USB devices
lsusb

# Check video devices
ls /dev/video*

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. RoboClaw Not Responding

```bash
# Check serial devices
ls /dev/ttyUSB*

# Check permissions
ls -la /dev/ttyUSB*

# Test serial connection
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### 4. Docker Issues

```bash
# Check Docker status
sudo systemctl status docker

# Restart Docker
sudo systemctl restart docker

# Check container status
docker ps -a
```

### 5. Performance Issues

- Ensure Pi has adequate cooling
- Close unnecessary applications
- Consider using a faster microSD card
- Monitor system resources: `htop`

## Auto-start on Boot

To enable the SLAM Bot to start automatically on boot:

```bash
# Enable the service
sudo systemctl enable slam-bot-docker.service

# Start the service
sudo systemctl start slam-bot-docker.service

# Check status
sudo systemctl status slam-bot-docker.service
```

## Development Workflow

### 1. Code Changes

```bash
# On your development machine
# Make changes to source code

# Deploy changes to Pi
./deploy_to_pi.sh pi@raspberrypi.local

# On Pi, rebuild and restart
docker-compose -f docker-compose.pi.yml up -d --build
```

### 2. Testing

```bash
# Test individual components
docker exec -it slam-bot-ros bash
ros2 topic list
ros2 topic echo /cmd_vel
```

## Monitoring

### 1. System Resources
```bash
# Monitor system resources
htop

# Monitor disk usage
df -h

# Monitor memory
free -h
```

### 2. ROS Topics
```bash
# List all topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /wheel_odom
ros2 topic echo /battery_voltage
ros2 topic echo /scan
```

### 3. TF Tree
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check TF transforms
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Support

If you encounter issues:

1. Check the logs: `docker-compose -f docker-compose.pi.yml logs -f`
2. Verify hardware connections
3. Check system resources
4. Ensure all dependencies are installed
5. Review the configuration files

## Performance Tips

- Use a high-quality microSD card (Class 10 or better)
- Ensure adequate power supply
- Keep the Pi cool with proper ventilation
- Close unnecessary background processes
- Consider using an external SSD for better I/O performance
