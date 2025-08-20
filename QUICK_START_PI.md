# 🚀 Quick Start on Raspberry Pi

This guide will get you up and running with the SLAM Bot directly on your Pi.

## 📋 Prerequisites

- Raspberry Pi 5 with Ubuntu 22.04 or Raspberry Pi OS
- At least 8GB RAM (16GB recommended)
- MicroSD card with at least 32GB storage
- Network connection (WiFi or Ethernet)
- RoboClaw 2x15A motor controller
- RealSense D455 camera
- 2x DC motors with encoders

## 🚨 **Important: Package Availability**

Some ROS2 packages are not available for ARM64 architecture on Raspberry Pi. The Dockerfiles have been updated to only install available packages. See `PI_PACKAGE_NOTES.md` for details.

## 🎯 Quick Setup (3 Steps)

### **Step 1: Clone and Setup**
```bash
# Clone the repository (if you haven't already)
git clone <your-repo-url>
cd slam-bot

# Make the setup script executable
chmod +x setup_on_pi.sh

# Run the setup script
./setup_on_pi.sh
```

**⚠️  If you get package errors, try the simple setup instead:**
```bash
# Alternative: Use the simple setup script
chmod +x simple_setup_pi.sh
./simple_setup_pi.sh
```

### **Step 2: Log Out and Back In**
After the setup script completes:
```bash
# Log out to apply group changes
exit

# SSH back in
ssh pi@raspberrypi.local
```

### **Step 3: Start the Robot**
```bash
# Navigate to project directory
cd ~/slam-bot

# Start the SLAM Bot
~/start_slam_bot.sh
```

## 🔧 What the Setup Scripts Do

### **Main Setup Script (`setup_on_pi.sh`)**
Automatically:
1. **Updates system packages**
2. **Installs Docker and Docker Compose**
3. **Sets up hardware permissions** (RoboClaw, RealSense)
4. **Creates udev rules** for USB devices
5. **Creates startup script** (`~/start_slam_bot.sh`)
6. **Creates systemd service** for auto-start

### **Simple Setup Script (`simple_setup_pi.sh`)**
Alternative approach that:
1. **Installs essential packages only**
2. **Uses pip for Python packages** (more reliable)
3. **Creates basic udev rules**
4. **Focuses on core functionality**

## 🚨 Troubleshooting Package Installation

### **Common Package Issues**

#### **"Unable to locate package python3-pyserial"**
This is common on Raspberry Pi. The setup scripts now handle this automatically by:
1. **Trying different package names** (`python3-pyserial`, `python3-serial`)
2. **Falling back to pip installation** if packages aren't found
3. **Using `--break-system-packages`** flag for pip (required on newer Pi OS versions)

#### **If you still have issues, install manually:**
```bash
# Install Python packages manually via pip
sudo pip3 install --break-system-packages pyserial numpy opencv-python matplotlib scipy

# Or try system packages with different names
sudo apt install python3-serial python3-numpy python3-opencv python3-matplotlib python3-scipy
```

#### **Check your Pi OS version:**
```bash
# Check OS version
cat /etc/os-release

# Check architecture
uname -m

# Check available Python packages
apt-cache search python3-pyserial
apt-cache search python3-serial
```

## 🔄 **Rebuilding After Package Fixes**

If you encountered the ROS2 package errors, rebuild the containers:

```bash
# Stop existing containers
docker-compose -f docker-compose.pi.yml down

# Rebuild with updated Dockerfiles
docker-compose -f docker-compose.pi.yml up -d --build
```

## 🎮 Using the Robot

### **Start the Robot Base**
```bash
# Access the ROS container
docker exec -it slam-bot-ros bash

# Inside container, start the robot
ros2 launch slam_bot_bringup robot.launch.py
```

### **Start SLAM (Mapping)**
```bash
# In another terminal, access container
docker exec -it slam-bot-ros bash

# Start SLAM (using Navigation2 built-in capabilities)
ros2 launch nav2_bringup bringup_launch.py slam:=True
```

### **Start Navigation**
```bash
# In another terminal, access container
docker exec -it slam-bot-ros bash

# Start navigation (requires a map first)
ros2 launch slam_bot_navigation navigation.launch.py
```

### **Teleop Control**
```bash
# In another terminal, access container
docker exec -it slam-bot-ros bash

# Start teleop keyboard control
ros2 run teleop_twist-keyboard teleop_twist_keyboard
```

## 📊 Monitor and Debug

### **View Container Logs**
```bash
# View all logs
docker-compose -f docker-compose.pi.yml logs -f

# View specific service logs
docker-compose -f docker-compose.pi.yml logs -f slam-bot-ros
```

### **Check Container Status**
```bash
# See running containers
docker ps

# See all containers (including stopped)
docker ps -a
```

### **Access Containers**
```bash
# Access ROS container
docker exec -it slam-bot-ros bash

# Access GUI container
docker exec -it slam-bot-gui bash
```

## 🛠️ Common Commands

### **Container Management**
```bash
# Stop containers
docker-compose -f docker-compose.pi.yml down

# Restart containers
docker-compose -f docker-compose.pi.yml restart

# Rebuild and restart
docker-compose -f docker-compose.pi.yml up -d --build
```

### **System Management**
```bash
# Enable auto-start on boot
sudo systemctl enable slam-bot-docker.service

# Check service status
sudo systemctl status slam-bot-docker.service

# Start service manually
sudo systemctl start slam-bot-docker.service
```

## 🔍 Troubleshooting

### **Permission Issues**
```bash
# Fix USB permissions
sudo usermod -a -G dialout,video $USER
# Log out and back in
```

### **Camera Not Detected**
```bash
# Check USB devices
lsusb

# Check video devices
ls /dev/video*

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### **RoboClaw Not Responding**
```bash
# Check serial devices
ls /dev/ttyUSB*

# Check permissions
ls -la /dev/ttyUSB*

# Test serial connection
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### **Docker Issues**
```bash
# Check Docker status
sudo systemctl status docker

# Restart Docker
sudo systemctl restart docker

# Check container status
docker ps -a
```

### **Python Package Issues**
```bash
# Check pip installation
pip3 list | grep pyserial

# Reinstall packages
sudo pip3 install --force-reinstall --break-system-packages pyserial

# Check Python path
python3 -c "import serial; print('pyserial working')"
```

## 📁 Project Structure on Pi

```
~/slam-bot/
├── docker/                 # Docker configuration files
├── src/                    # ROS packages source code
├── config/                 # Configuration files
├── launch/                 # Launch files
├── maps/                   # Generated maps
├── docker-compose.pi.yml   # Pi Docker services
├── setup_on_pi.sh         # Main Pi setup script
├── simple_setup_pi.sh     # Simple Pi setup script (alternative)
├── start_slam_bot.sh      # Startup script (created by setup)
├── PI_PACKAGE_NOTES.md    # Package availability information
└── README.md              # Project documentation
```

## 🚨 Important Notes

1. **First build takes time**: Docker images need to be built from source for ARM64
2. **Hardware connections**: Ensure RoboClaw and RealSense are properly connected
3. **Power supply**: Use stable 5V/3A power supply
4. **Cooling**: Ensure Pi has adequate ventilation
5. **Storage**: Use high-quality microSD card (Class 10+)
6. **Package issues**: Use the simple setup script if you encounter package problems
7. **ROS2 packages**: Some advanced packages aren't available for ARM64 (see `PI_PACKAGE_NOTES.md`)

## 🔄 Development Workflow

### **Update Code**
```bash
# Pull latest changes
git pull origin main

# Rebuild and restart
docker-compose -f docker-compose.pi.yml up -d --build
```

### **Test Changes**
```bash
# Access container
docker exec -it slam-bot-ros bash

# Test individual components
ros2 topic list
ros2 topic echo /cmd_vel
```

## 📞 Getting Help

If you encounter issues:

1. **Check the logs**: `docker-compose -f docker-compose.pi.yml logs -f`
2. **Verify hardware connections**
3. **Check system resources**: `htop`, `df -h`, `free -h`
4. **Review configuration files**
5. **Try the simple setup script** if packages fail
6. **Check package availability**: `PI_PACKAGE_NOTES.md`
7. **Check the full deployment guide**: `PI_DEPLOYMENT.md`

## 🎉 Success Indicators

You'll know everything is working when:

- ✅ Docker containers are running (`docker ps` shows containers)
- ✅ Robot launches without errors
- ✅ Motor controller responds (`ros2 topic echo /wheel_odom`)
- ✅ RViz shows robot model
- ✅ Navigation2 launches successfully

## 🆘 Still Having Issues?

If you continue to have problems:

1. **Try the simple setup script**: `./simple_setup_pi.sh`
2. **Check your Pi OS version**: `cat /etc/os-release`
3. **Verify architecture**: `uname -m`
4. **Check available packages**: `apt-cache search python3-pyserial`
5. **Install packages manually**: Use the pip commands above
6. **Review package notes**: `PI_PACKAGE_NOTES.md`

## 📚 **Additional Resources**

- **Package Availability**: `PI_PACKAGE_NOTES.md` - Details on ARM64 package compatibility
- **Full Deployment Guide**: `PI_DEPLOYMENT.md` - Comprehensive deployment information
- **Troubleshooting**: See troubleshooting sections above

Happy robot building! 🤖
