# SLAM-Bot: Differential Drive Robot with RoboClaw + ROS 2

> A Raspberry Pi 5–powered robot using a RoboClaw 2x15A motor controller, wheel encoders, and ROS 2 Humble for control and odometry.  
> Features include joystick teleoperation via an Xbox controller and Dockerized ROS 2 setup for portability.

---

## 🚀 Features
- **Motor control**: RoboClaw 2x15A, packet-serial over USB
- **Odometry**: Wheel encoders → `/odom` (with optional TF publishing)
- **Teleoperation**: Xbox controller via `joy` + `teleop_twist_joy`
- **Dockerized**: Bring up the full stack with one command
- **Safety**: Deadman switch (LB), turbo button (RB), automatic stop on command timeout

---

## 🖼️ Demo
<!-- Replace with your own image or GIF -->
![Robot demo](docs/demo.gif)

---

## 📦 Quickstart

### 1. Clone the repo
```bash
git clone git@github.com:ngardner045/slam-bot.git
cd slam-bot/docker
```

### 2. Build and run
```bash
docker compose up -d --build
```

This launches:
- `roboclaw_base_node.py`: ROS 2 node for motor control + odometry  
- `joy_node`: reads controller events from `/dev/input/eventX`  
- `teleop_twist_joy`: maps joystick → `/cmd_vel` → motors  

### 3. Drive
- Hold **LB** (deadman)  
- Push **left stick** forward/back (linear velocity)  
- Move **left stick left/right** (angular velocity)  
- Hold **RB** for turbo mode  

---

## 🎮 Controller mapping
| Control      | Function         |
|--------------|------------------|
| **LB (6)**   | Enable (deadman) |
| **RB (7)**   | Turbo mode       |
| **Axis 1**   | Linear x (fwd/back) |
| **Axis 0**   | Angular yaw (turn) |

---

## 📂 Project structure
```
.
├── docker/
│   ├── Dockerfile           # Base ROS + deps
│   ├── docker-compose.yml   # Bring up robot + teleop
│   └── teleop.yaml          # Optional teleop config
├── src/
│   └── roboclaw_base_node.py # Main ROS 2 driver node
├── docs/
│   └── demo.gif             # Demo media (optional)
├── LICENSE
└── README.md
```

---

## ⚙️ Configuration
- **Port**: `/dev/ttyACM0` (mapped via Docker)
- **Address**: `0x80` (packet-serial mode)
- **Geometry params**: wheel radius, track width, gear ratio in node params
- **Timeout**: `cmd_timeout:=2.0` (motors stop if no command)

---

## 🛡️ Safety notes
- RoboClaw red LED may blink when motors are commanded to stop — this is normal (status event, not a fault).
- Ensure batteries are properly rated for your motor current.
- Always test with wheels off the ground first.

---

## 📫 Contact
- **Author**: Nathan Gardner  
- **GitHub**: [@ngardner045](https://github.com/ngardner045)


## License
No license granted. © ngardner045, 2025.  
Viewing is permitted; any use, copying, modification, or distribution requires prior written permission.
