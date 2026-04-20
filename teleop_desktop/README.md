# Teleop Desktop

A Docker-based module for manually controlling a RoboMaster S1 via GUI or keyboard. Runs on any machine with a display (desktop, laptop, or SSH-forwarded X11) and publishes `Twist` messages to the robot's `cmd_vel` topic.

## Prerequisites

- Docker and Docker Compose
- An X server on the local machine (desktop Linux) or SSH-forwarded X11 (`ssh -X`)
- Network access to the robot on `ROS_DOMAIN_ID=0`
- The robot's `robomaster_bridge` and `robomaster_domain_bridge` services running

## Services

Three modes are available in [docker-compose.yml](docker-compose.yml):

| Service | Description |
|---|---|
| `teleop` | Default — `rqt_robot_steering` with slider-based virtual joystick |
| `rqt` | Full rqt dashboard (graph, topic viewer, publisher, etc.) |
| `keyboard` | Terminal-based `teleop_twist_keyboard` |

## Usage

### Slider GUI (default)

```bash
./run_teleop.sh              # x86/standard host
./run_teleop.sh --nvidia     # NVIDIA Jetson host
```

This runs `xhost +local:docker` then `docker compose up --build teleop` (or `teleop-jetson`). In the GUI that opens, set the **Topic** field to `/robot_<ID>/cmd_vel` (e.g. `/robot_4/cmd_vel`) and drag the sliders to drive.

The `--nvidia` / `--jetson` flag selects the L4T-compatible base image (`dustynv/ros:humble-pytorch-l4t-r35.3.1`) and runs the container with `--runtime nvidia`. Requires `nvidia-container-runtime` installed on the Jetson host (included in stock JetPack).

### Full rqt dashboard

```bash
xhost +local:docker
docker compose run --rm rqt
```

Open any rqt plugin from the menu — graph, topic monitor, manual publisher, etc.

### Keyboard teleop

```bash
docker compose run --rm keyboard
```

Use the keys shown on screen (`i` forward, `,` back, `j`/`l` turn, etc.). By default it publishes to `/cmd_vel` — to target a specific robot, use:

```bash
docker compose run --rm keyboard \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot_4/cmd_vel
```

## Running over SSH

From your workstation:

```bash
ssh -X user@<machine_with_display>
cd teleop_desktop
./run_teleop.sh
```

The `-X` flag forwards X11 so GUIs from the remote Docker container render on your local screen.

> Note: X11 forwarding over SSH is slower than local. For remote control over a network, consider **Foxglove Studio** with `foxglove_bridge` for a browser-based virtual joystick instead.

## Network Topology

This module assumes the `domain_bridge` gateway (installed by `robomaster_bridge/install.bash`) is exposing `cmd_vel` on `ROS_DOMAIN_ID=0`. The container is configured accordingly:

- `ROS_DOMAIN_ID=0` — public network domain
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=/etc/cyclone_dds.xml` — mounted from `../cyclone_dds.xml`

## Troubleshooting

**"cannot open display"**
- On the host, run `xhost +local:docker` before `docker compose up`
- If over SSH, ensure you connected with `ssh -X` and `echo $DISPLAY` is set

**GUI opens but no robot responds**
- Check `ros2 topic list` includes `/robot_<ID>/cmd_vel`
- Confirm the domain_bridge gateway is running on the robot: `sudo systemctl status robomaster_domain_bridge`
- Confirm the topic field in the GUI matches your `ROBOT_ID`

**"Qt xcb connection failed"**
- Usually an X auth issue — ensure `$HOME/.Xauthority` exists and is being mounted
