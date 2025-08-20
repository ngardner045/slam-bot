#!/bin/bash

# Simple SLAM Bot Setup Script for Raspberry Pi
# Alternative setup if the main script has issues

set -e

echo "🚀 Simple SLAM Bot Setup on Raspberry Pi..."

# Update system
echo "📦 Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Install essential packages only
echo "🔧 Installing essential packages..."
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

# Install Python packages via pip (more reliable)
echo "🐍 Installing Python packages via pip..."
sudo pip3 install --break-system-packages \
    pyserial \
    numpy \
    opencv-python \
    matplotlib \
    scipy

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

# Set up basic hardware permissions
echo "🔐 Setting up basic hardware permissions..."
sudo usermod -a -G dialout,video $USER

# Create basic udev rules
echo "📷 Creating basic udev rules..."
sudo tee /etc/udev/rules.d/99-slam-bot.rules > /dev/null << 'UDEV_RULES'
# RealSense camera
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", MODE="0666"
# RoboClaw motor controller
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", MODE="0666"
UDEV_RULES

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create simple startup script
echo "📝 Creating startup script..."
cat > ~/start_slam_bot.sh << 'STARTUP_SCRIPT'
#!/bin/bash

# Start SLAM Bot on Raspberry Pi with Docker
cd ~/slam-bot

echo "🔨 Building Docker images for Pi..."
docker-compose -f docker-compose.pi.yml build

echo "🚀 Starting SLAM Bot containers..."
docker-compose -f docker-compose.pi.yml up -d

echo ""
echo "✅ SLAM Bot containers started!"
echo ""
echo "🔧 Useful commands:"
echo "  View logs: docker-compose -f docker-compose.pi.yml logs -f"
echo "  Stop containers: docker-compose -f docker-compose.pi.yml down"
echo "  Access ROS container: docker exec -it slam-bot-ros bash"
echo ""
echo "🎮 To start the robot:"
echo "  docker exec -it slam-bot-ros bash"
echo "  ros2 launch slam_bot_bringup robot.launch.py"
STARTUP_SCRIPT

chmod +x ~/start_slam_bot.sh

echo ""
echo "🎉 Simple setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Log out and back in (for group permissions to take effect)"
echo "2. Connect your hardware:"
echo "   - RoboClaw to USB port"
echo "   - RealSense D455 camera to USB port"
echo "3. Start the robot: ~/start_slam_bot.sh"
echo ""
echo "📚 For more information, see QUICK_START_PI.md"
