# Wits RoboMaster S1 Platform

A Raspberry Pi-focused robotics platform for the DJI RoboMaster S1, providing full ROS 2-based control, sensing, vision, and safety infrastructure. Forked from the [Cambridge RoboMaster](https://github.com/proroklab/cambridge-robomaster) project by the [Prorok Lab](https://proroklab.github.io/cambridge-robomaster/) and adapted for deployment on Raspberry Pi hardware with planned extension to NVIDIA Jetson platforms.

> **Platform Status:** Raspberry Pi is the primary supported target. NVIDIA Jetson support is planned for future development.

---

## Origins & Relationship to Cambridge RoboMaster

This repository is forked from the [Prorok Lab's Cambridge RoboMaster](https://github.com/proroklab/cambridge-robomaster) platform developed at the University of Cambridge. The original project provided a multi-robot research platform built around the DJI RoboMaster S1, with support for both NVIDIA Jetson and Raspberry Pi compute modules.

**What this fork changes:**

- **RPi-first development** — The codebase has been refactored to target Raspberry Pi as the primary compute platform.
- **IR camera + AprilTag pipeline** — Added a Docker-based OpenNI2 IR camera module with AprilTag detection infrastructure for vision-based localization (`rpi_ir_apriltag/`).
- **NVIDIA Jetson (planned)** — Future work will extend the platform back to NVIDIA Jetson boards for GPU-accelerated perception workloads.

### Key Upstream Sources

| Component | Source |
|---|---|
| CAN ROS 2 bridge & message definitions | [proroklab/robomaster_ros2_can](https://github.com/proroklab/robomaster_ros2_can) |
| Original platform documentation | [proroklab.github.io/cambridge-robomaster](https://proroklab.github.io/cambridge-robomaster/) |
| CAN protocol implementation | [janblumenkamp/robomaster_sdk_can](https://github.com/janblumenkamp/robomaster_sdk_can) |
| ROS 2 message definitions | [proroklab/ros2_robomaster_msgs](https://github.com/proroklab/ros2_robomaster_msgs) |

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Raspberry Pi                            │
│                                                             │
│  ┌──────────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ robomaster_bridge│  │ ir_cam_ros   │  │ emergency    │  │
│  │ (Docker)         │  │ (Docker)     │  │ _stop        │  │
│  │                  │  │              │  │ (native)     │  │
│  │ • controlling    │  │ • openni2    │  │              │  │
│  │ • sensing        │  │ • apriltag   │  │ • GPIO btn   │  │
│  │                  │  │ • image_proc │  │ • NeoPixel   │  │
│  └───────┬──────────┘  └──────────────┘  └──────────────┘  │
│          │ CAN bus                                          │
│  ┌───────┴──────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │ MCP2515 CAN HAT  │  │ USB IR Cam   │  │ LCD HATs     │  │
│  └───────┬──────────┘  └──────────────┘  └──────────────┘  │
└──────────┼──────────────────────────────────────────────────┘
           │
    ┌──────┴──────┐
    │ RoboMaster  │
    │ S1 Base     │
    └─────────────┘
```
> Note - the camera node is used as the form of localization for now but may be extended to FoV cameras for each RoboMaster. Camera stream processing (April tag detection) is to be processed on a PC with sufficient compute.

All ROS 2 nodes communicate over **CycloneDDS** with host networking. Docker containers share the host network stack for seamless DDS discovery across the robot fleet.

---

## Repository Structure

```
s1-robomaster/
├── robomaster_bridge/       # CAN bridge — Docker container for S1 motor/LED control
│   ├── Dockerfile           # ROS 2 Humble + CAN utilities
│   ├── install.bash         # Systemd service, kernel modules, CAN network config
│   └── run_docker.sh        # Container launch script
├── rpi_ir_apriltag/         # IR camera + AprilTag detection (Docker)
│   └── ros_ws/src/
│       └── rpi_ir_bringup/  # Launch files, camera & AprilTag configs
├── emergency_stop/          # GPIO emergency stop + NeoPixel LED feedback
├── joycon/                  # Game controller mapping (SDL + ROS 2 joy)
├── rviz_desktop/            # Docker-based RViz2 visualization client
├── setup_rasppi/            # RPi initial setup scripts (apt, sudo, etc.)
├── setup_LCD_screens/       # Waveshare SPI LCD HAT setup
├── docs/                    # Setup documentation
│   └── software_setup_raspi.md
└── copy_to_raspberry_pi.bash
```

---

## ROS 2 Interface

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | Velocity commands (linear + angular) |
| `/cmd_wheels` | `robomaster_msgs/WheelSpeed` | Subscribe | Direct wheel speed control (FR/FL/RL/RR) |
| `/vel` | `geometry_msgs/TwistStamped` | Publish | Velocity feedback |
| `/wheel_speed` | `robomaster_msgs/WheelSpeed` | Publish | Wheel encoder readings |
| `/battery_state` | `sensor_msgs/BatteryState` | Publish | Battery voltage and status |
| `/attitude` | `geometry_msgs/QuaternionStamped` | Publish | IMU orientation |
| `/ir/image` | `sensor_msgs/Image` | Publish | IR camera feed |
| `/ir/camera_info` | `sensor_msgs/CameraInfo` | Publish | Camera intrinsics |

### Services

| Service | Type | Description |
|---|---|---|
| `/led` | `robomaster_msgs/LED` | RGB LED control (solid/flashing modes) |
| `/emergency_stop` | `emergency_stop_msgs/EmergencyStop` | Distributed emergency stop broadcast |

---

## Hardware Requirements

| Component | Details |
|---|---|
| Compute | Raspberry Pi 4/5 (primary), NVIDIA Jetson (planned) |
| Robot base | DJI RoboMaster S1 |
| CAN interface | Waveshare MCP2515 CAN HAT |
| Camera | OpenNI2-compatible USB IR camera (e.g. PrimeSense, ASUS Xtion) |
| Emergency stop | Physical button (GPIO 17) + 39-LED NeoPixel ring |
| Display | Waveshare Zero LCD HAT (A) — 1x 1.3" + 2x 0.96" SPI screens |
| Controller | Any SDL-compatible gamepad (optional) |

---

## Setup & Installation

### Prerequisites

- Raspberry Pi OS (64-bit recommended)
- Docker & Docker Compose
- SSH access to the Pi

### Quick Start

**1. Flash and connect**
```bash
# Flash RPi OS, set credentials (user: rasppiuser, pass: rasppiuser)
# Enable SSH, connect to local network
ssh-copy-id rasppiuser@<RPI_IP>
```

**2. Copy repository to Pi**
```bash
./copy_to_raspberry_pi.bash
```

**3. Initial setup (on the Pi)**
```bash
sudo ./setup_rasppi/sudocfg.bash
sudo vi /etc/hostname   # Set to e.g. robomaster-1
sudo reboot

sudo ./setup_rasppi/apt.bash
# Reboot after package installation
```

**4. Docker setup**
```bash
sudo ./docker/add_group.bash
sudo ./docker/setup_docker_compose.bash
sudo cp docker/daemon.json /etc/docker
sudo service docker restart
```

**5. CAN bridge**
```bash
cd robomaster_bridge
sudo ./install.bash
```

This configures:
- Kernel modules (`can`, `can_raw`, `mcp251x`)
- systemd-networkd CAN interface (1 Mbit/s)
- Systemd service for auto-start on boot

**6. IR camera (optional)**
```bash
cd rpi_ir_apriltag
docker-compose up
```

> **Note:** AprilTag detection requires camera calibration before enabling. See config files in `rpi_ir_apriltag/ros_ws/src/rpi_ir_bringup/config/`.

For detailed setup instructions, see [docs/software_setup_raspi.md](docs/software_setup_raspi.md).

---

## Running

### Start the CAN bridge
```bash
# Via systemd (auto-starts on boot after install)
sudo systemctl start robomaster_bridge

# Or manually
cd robomaster_bridge && ./run_docker.sh
```

### Teleoperation with a gamepad
```bash
SDL_GAMECONTROLLERCONFIG=$(cat joycon/gamecontrollerdb.txt) \
  ros2 run joy game_controller_node --ros-args -r __ns:=/robomaster_1/joy -p device_id:=0
```

### Visualization (from a desktop machine)
```bash
cd rviz_desktop
docker-compose up
```

### Monitor robot state
```bash
ros2 topic echo /vel             # Velocity feedback
ros2 topic echo /battery_state   # Battery level
ros2 topic echo /wheel_speed     # Wheel encoders
```

---

## Development Status

| Module | Status |
|---|---|
| CAN bridge (control + sensing) | Production-ready |
| Emergency stop system | Production-ready |
| RPi setup & deployment | Production-ready |
| LCD status displays | Working |
| IR camera driver | Working (streaming only) |
| AprilTag detection | Infrastructure in place, needs calibration |
| Gamepad integration | Basic support |
| NVIDIA Jetson support | Planned |

---

## Contributing

This project is developed by the Wits Robots team. Contributions should target Raspberry Pi compatibility first. When adding new modules:

- Use Docker containers with `ros:humble-ros-base` as the base image
- Use host networking and CycloneDDS for ROS 2 communication
- Provide systemd service files for boot persistence where appropriate

---

## Acknowledgements

This project builds on the excellent work of the [Prorok Lab](https://www.proroklab.org/) at the University of Cambridge. The original Cambridge RoboMaster platform and its associated ROS 2 CAN bridge were developed by Jan Blumenkamp and the Prorok Lab team.

- **Cambridge RoboMaster platform:** [proroklab/cambridge-robomaster](https://github.com/proroklab/cambridge-robomaster)
- **ROS 2 CAN bridge:** [proroklab/robomaster_ros2_can](https://github.com/proroklab/robomaster_ros2_can)
- **CAN SDK:** [janblumenkamp/robomaster_sdk_can](https://github.com/janblumenkamp/robomaster_sdk_can)
- **Platform website:** [proroklab.github.io/cambridge-robomaster](https://proroklab.github.io/cambridge-robomaster/)

---

## License

See the upstream [Cambridge RoboMaster](https://github.com/proroklab/cambridge-robomaster) repository for license information.
