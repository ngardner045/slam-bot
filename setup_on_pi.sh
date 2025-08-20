#!/bin/bash

# SLAM Bot Setup Script for Raspberry Pi
# Run this script directly on the Pi after cloning the repo

set -e

echo "🚀 Setting up SLAM Bot on Raspberry Pi..."

# Check if running on Pi
if ! uname -m | grep -q "aarch64\|arm64"; then
    echo "⚠️  Warning: This script is designed for ARM64 architecture (Raspberry Pi)"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "📦 Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install required packages with fallbacks
echo "🔧 Installing required packages..."
sudo apt install -y \
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
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release

# Install Python packages with fallbacks
echo "🐍 Installing Python packages..."
# Try different package names for pyserial
if apt-cache search python3-pyserial | grep -q python3-pyserial; then
    sudo apt install -y python3-pyserial
elif apt-cache search python3-serial | grep -q python3-serial; then
    sudo apt install -y python3-serial
else
    echo "⚠️  python3-pyserial not found in repos, will install via pip"
fi

# Try to install other packages
if apt-cache search python3-numpy | grep -q python3-numpy; then
    sudo apt install -y python3-numpy
fi

if apt-cache search python3-opencv | grep -q python3-opencv; then
    sudo apt install -y python3-opencv
fi

if apt-cache search python3-matplotlib | grep -q python3-matplotlib; then
    sudo apt install -y python3-matplotlib
fi

if apt-cache search python3-scipy | grep -q python3-scipy; then
    sudo apt install -y python3-scipy
fi

# Install Docker
echo "🐳 Installing Docker..."
if ! command -v docker > /dev/null; then
    # Add Docker's official GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # Add Docker repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Install Docker
    sudo apt update
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
    
    # Add user to docker group
    sudo usermod -a -G docker $USER
    echo "✅ Docker installed! You may need to log out and back in for group changes to take effect."
else
    echo "✅ Docker already installed"
fi

# Install Docker Compose if not present
if ! command -v docker-compose > /dev/null; then
    echo "🔧 Installing Docker Compose..."
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    echo "✅ Docker Compose installed"
else
    echo "✅ Docker Compose already installed"
fi

# Install missing Python packages via pip
echo "🐍 Installing Python packages via pip..."
sudo pip3 install --break-system-packages \
    pyserial \
    numpy \
    opencv-python \
    matplotlib \
    scipy

# Set up hardware permissions
echo "🔐 Setting up hardware permissions..."
sudo usermod -a -G dialout,video $USER

# Create udev rules for RealSense camera
echo "📷 Creating udev rules for RealSense camera..."
sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null << 'UDEV_RULES'
# RealSense camera
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5d", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", MODE="0666"
UDEV_RULES

# Create udev rules for RoboClaw
echo "⚙️  Creating udev rules for RoboClaw..."
sudo tee /etc/udev/rules.d/99-roboclaw.rules > /dev/null << 'UDEV_RULES'
# RoboClaw motor controller
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"
UDEV_RULES

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create startup script
echo "📝 Creating startup script..."
cat > ~/start_slam_bot.sh << 'STARTUP_SCRIPT'
#!/bin/bash

# Start SLAM Bot on Raspberry Pi with Docker
cd ~/slam-bot

# Check if containers are running
if docker-compose -f docker-compose.pi.yml ps | grep -q "Up"; then
    echo "🔄 Restarting SLAM Bot containers..."
    docker-compose -f docker-compose.pi.yml down
fi

# Build Docker images (if needed)
echo "🔨 Building Docker images for Pi..."
docker-compose -f docker-compose.pi.yml build

# Start containers
echo "🚀 Starting SLAM Bot containers..."
docker-compose -f docker-compose.pi.yml up -d

echo ""
echo "✅ SLAM Bot containers started!"
echo ""
echo "🔧 Useful commands:"
echo "  View logs: docker-compose -f docker-compose.pi.yml logs -f"
echo "  Stop containers: docker-compose -f docker-compose.pi.yml down"
echo "  Access ROS container: docker exec -it slam-bot-ros bash"
echo "  Access GUI container: docker exec -it slam-bot-gui bash"
echo ""
echo "🎮 To start the robot:"
echo "  docker exec -it slam-bot-ros bash"
echo "  ros2 launch slam_bot_bringup robot.launch.py"
STARTUP_SCRIPT

chmod +x ~/start_slam_bot.sh

# Create systemd service for auto-start
echo "⚙️  Creating systemd service..."
sudo tee /etc/systemd/system/slam-bot-docker.service > /dev/null << 'SERVICE_FILE'
[Unit]
Description=SLAM Bot Docker Service
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
User=pi
WorkingDirectory=/home/pi/slam-bot
ExecStart=/bin/bash -c 'docker-compose -f docker-compose.pi.yml up -d'
ExecStop=/bin/bash -c 'docker-compose -f docker-compose.pi.yml down'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE_FILE

echo ""
echo "🎉 Setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Log out and back in (for group permissions to take effect)"
echo "2. Connect your hardware:"
echo "   - RoboClaw to USB port"
echo "   - RealSense D455 camera to USB port"
echo "3. Start the robot: ~/start_slam_bot.sh"
echo ""
echo "🔧 To enable auto-start on boot:"
echo "  sudo systemctl enable slam-bot-docker.service"
echo ""
echo "📚 For more information, see PI_DEPLOYMENT.md"
