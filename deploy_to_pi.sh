#!/bin/bash

# Deploy SLAM Bot to Raspberry Pi with Docker
# Usage: ./deploy_to_pi.sh [pi_username@pi_ip]

set -e

# Default Pi connection (modify as needed)
PI_USER=${1:-"pi@raspberrypi.local"}
PI_IP=$(echo $PI_USER | cut -d@ -f2)
PI_USERNAME=$(echo $PI_USER | cut -d@ -f1)

echo "Deploying SLAM Bot to Raspberry Pi at $PI_IP..."

# Check if we can reach the Pi
echo "Checking connection to Pi..."
if ! ping -c 1 $PI_IP > /dev/null 2>&1; then
    echo "Error: Cannot reach Raspberry Pi at $PI_IP"
    echo "Please check:"
    echo "1. Pi is powered on and connected to network"
    echo "2. Pi IP address is correct"
    echo "3. Pi is accessible from this machine"
    exit 1
fi

# Create deployment directory on Pi
echo "Creating deployment directory on Pi..."
ssh $PI_USER "mkdir -p ~/slam-bot-deploy"

# Copy project files to Pi
echo "Copying project files to Pi..."
rsync -av --exclude='.git' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='maps/*.db' \
    --exclude='maps/*.yaml' \
    --exclude='maps/*.pgm' \
    ./ $PI_USER:~/slam-bot-deploy/

# Create Pi-specific configuration
echo "Creating Pi-specific configuration..."
cat > pi_config.sh << 'EOF'
#!/bin/bash

# Pi-specific configuration for SLAM Bot with Docker

# Update system
echo "Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Install required packages
echo "Installing required packages..."
sudo apt-get install -y \
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
    python3-pyserial \
    python3-numpy \
    python3-opencv \
    python3-matplotlib \
    python3-scipy \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release

# Install Docker
echo "Installing Docker..."
if ! command -v docker > /dev/null; then
    # Add Docker's official GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # Add Docker repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Install Docker
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
    
    # Add user to docker group
    sudo usermod -a -G docker $USER
fi

# Install Docker Compose if not present
if ! command -v docker-compose > /dev/null; then
    echo "Installing Docker Compose..."
    sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.0/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
fi

# Set up USB permissions for RoboClaw
echo "Setting up USB permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Create udev rules for RealSense camera
echo "Creating udev rules for RealSense camera..."
sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null << 'UDEV_RULES'
# RealSense camera
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5d", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b4d", MODE="0666"
UDEV_RULES

# Create udev rules for RoboClaw
echo "Creating udev rules for RoboClaw..."
sudo tee /etc/udev/rules.d/99-roboclaw.rules > /dev/null << 'UDEV_RULES'
# RoboClaw motor controller
SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"
UDEV_RULES

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Create startup script
echo "Creating startup script..."
cat > ~/start_slam_bot.sh << 'STARTUP_SCRIPT'
#!/bin/bash

# Start SLAM Bot on Raspberry Pi with Docker
cd ~/slam-bot-deploy

# Build Docker images
echo "Building Docker images for Pi..."
docker-compose -f docker-compose.pi.yml build

# Start containers
echo "Starting SLAM Bot containers..."
docker-compose -f docker-compose.pi.yml up -d

echo "SLAM Bot containers started!"
echo "To access ROS container: docker exec -it slam-bot-ros bash"
echo "To view logs: docker-compose -f docker-compose.pi.yml logs -f"
echo "To stop: docker-compose -f docker-compose.pi.yml down"
STARTUP_SCRIPT

chmod +x ~/start_slam_bot.sh

# Create systemd service for auto-start
echo "Creating systemd service..."
sudo tee /etc/systemd/system/slam-bot-docker.service > /dev/null << 'SERVICE_FILE'
[Unit]
Description=SLAM Bot Docker Service
After=docker.service
Requires=docker.service

[Service]
Type=oneshot
User=pi
WorkingDirectory=/home/pi/slam-bot-deploy
ExecStart=/bin/bash -c 'docker-compose -f docker-compose.pi.yml up -d'
ExecStop=/bin/bash -c 'docker-compose -f docker-compose.pi.yml down'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
SERVICE_FILE

# Enable service (optional - uncomment to auto-start on boot)
# sudo systemctl enable slam-bot-docker.service

echo "Configuration complete!"
echo "You may need to log out and back in for group changes to take effect."
echo "To start manually: ~/start_slam_bot.sh"
echo "To enable auto-start: sudo systemctl enable slam-bot-docker.service"
EOF

# Copy and run configuration script on Pi
echo "Running configuration on Pi..."
scp pi_config.sh $PI_USER:~/
ssh $PI_USER "chmod +x ~/pi_config.sh && ~/pi_config.sh"

# Clean up local config file
rm pi_config.sh

echo ""
echo "Deployment complete!"
echo ""
echo "Next steps on the Pi:"
echo "1. Log out and back in (for group permissions)"
echo "2. Connect your hardware:"
echo "   - RoboClaw to USB port"
echo "   - RealSense D455 camera to USB port"
echo "3. Run: ~/start_slam_bot.sh"
echo "4. Access ROS container: docker exec -it slam-bot-ros bash"
echo "5. Inside container: ros2 launch slam_bot_bringup robot.launch.py"
echo ""
echo "To enable auto-start on boot:"
echo "sudo systemctl enable slam-bot-docker.service"
echo ""
echo "Useful Docker commands:"
echo "  View logs: docker-compose -f docker-compose.pi.yml logs -f"
echo "  Stop containers: docker-compose -f docker-compose.pi.yml down"
echo "  Restart containers: docker-compose -f docker-compose.pi.yml restart"
