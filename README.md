# SLAM Bot - ROS-based Differential Drive Robot

A ROS-based differential drive robot with SLAM capabilities, designed for Raspberry Pi 5 with RealSense D455 camera and RoboClaw motor controller.

## Hardware Components

- **Raspberry Pi 5** - Main computing platform
- **RoboClaw 2x15A** - Dual motor controller for differential drive
- **2x DC Motors** - Differential drive system
- **RealSense D455** - RGB-D camera for SLAM and navigation
- **Differential Drive Chassis** - Robot base

## Features

- **SLAM (Simultaneous Localization and Mapping)** using RTAB-Map
- **Navigation Stack** with move_base
- **Motor Control** via RoboClaw interface
- **Camera Integration** with RealSense D455
- **Docker-based Development** for consistent environment
- **ROS2 Humble** support
- **Raspberry Pi 5 optimized** with ARM64 Docker images

## Quick Start

### Development Machine (x86_64)

1. **Clone the repository:**
   ```bash
   git clone <your-repo-url>
   cd slam-bot
   ```

2. **Build and run with Docker:**
   ```bash
   docker-compose up --build
   ```

3. **Access ROS tools:**
   ```bash
   # In another terminal
   docker exec -it slam-bot-ros bash
   ```

### Raspberry Pi 5 Deployment

1. **Deploy to Pi:**
   ```bash
   chmod +x deploy_to_pi.sh
   ./deploy_to_pi.sh pi@raspberrypi.local
   ```

2. **On Pi, start the robot:**
   ```bash
   ~/start_slam_bot.sh
   ```

3. **Access ROS container:**
   ```bash
   docker exec -it slam-bot-ros bash
   ros2 launch slam_bot_bringup robot.launch.py
   ```

## Project Structure

```
slam-bot/
├── docker/                 # Docker configuration files
│   ├── Dockerfile         # x86_64 development image
│   ├── Dockerfile.pi      # ARM64 Pi image
│   └── Dockerfile.gui.pi  # ARM64 Pi GUI image
├── src/                    # ROS packages source code
│   ├── slam_bot_bringup/   # Launch files and configuration
│   ├── slam_bot_control/   # Motor control and robot interface
│   ├── slam_bot_description/ # URDF robot description
│   └── slam_bot_navigation/ # Navigation and SLAM configuration
├── config/                  # Configuration files
├── launch/                  # Launch files
├── maps/                    # Generated maps
├── docker-compose.yml       # x86_64 Docker services
├── docker-compose.pi.yml    # Pi Docker services
├── deploy_to_pi.sh         # Pi deployment script
├── PI_DEPLOYMENT.md        # Pi deployment guide
└── README.md               # This file
```

## Development

### Building ROS Packages

```bash
# Inside the Docker container
cd /workspace
colcon build
source install/setup.bash
```

### Running Individual Components

```bash
# Launch robot base
ros2 launch slam_bot_bringup robot.launch.py

# Launch SLAM
ros2 launch slam_bot_navigation slam.launch.py

# Launch navigation
ros2 launch slam_bot_navigation navigation.launch.py
```

## Configuration

### Motor Parameters

Edit `config/motor_params.yaml` to configure:
- Motor encoder counts per revolution
- Wheel diameter and track width
- PID parameters for motor control

### Camera Parameters

Edit `config/camera_params.yaml` to configure:
- RealSense D455 parameters
- Image processing settings

## Docker Images

### Development (x86_64)
- `docker-compose.yml` - Uses standard ROS2 Humble image
- Optimized for development and testing

### Raspberry Pi 5 (ARM64)
- `docker-compose.pi.yml` - Uses ARM64 Ubuntu 22.04 base
- Optimized for Pi 5 performance
- Includes RealSense SDK compiled for ARM64

## Troubleshooting

### Common Issues

1. **Permission denied errors**: Ensure Docker has proper permissions
2. **Camera not detected**: Check USB connections and permissions
3. **Motor control issues**: Verify RoboClaw connections and power
4. **Pi performance issues**: Monitor temperature and system resources

### Debug Mode

Run with debug logging:
```bash
docker-compose -f docker-compose.debug.yml up
```

## Performance Considerations

### Raspberry Pi 5
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: Use high-quality microSD card (Class 10+)
- **Cooling**: Ensure adequate ventilation
- **Power**: Stable 5V/3A power supply

### Docker Performance
- Use `--privileged` for hardware access
- Mount `/dev` for device access
- Use `host` networking for ROS communication

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS2 community
- Intel RealSense team
- Pololu RoboClaw documentation
- Raspberry Pi Foundation
