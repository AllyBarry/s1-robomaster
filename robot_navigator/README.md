# Robot Navigator

Per-robot navigation node for the RoboMaster S1 fleet. Each instance controls
a single robot and accepts goals on a topic namespace dedicated to that robot,
so multiple robots can be commanded independently from rviz (or a script).

Camera-based localization from `apriltag_detector` arrives with variable
latency. The navigator bridges that gap by dead-reckoning from the robot's
own onboard sensors (IMU attitude and body-frame velocity) and fuses fresh
camera poses into its running estimate with a complementary filter, which
prevents the robot from snapping back to stale positions while still
correcting for drift.

**Pose-source priority (per integration step):**
1. `/robot_{id}/attitude` (QuaternionStamped, IMU) — yaw
2. `/robot_{id}/vel` (TwistStamped, body frame) — linear velocity
3. Last commanded `cmd_vel` — fallback when the onboard topics are stale

## Node: robot_navigator

**Subscriptions:**
| Topic | Type | Description |
|---|---|---|
| `/field/robot_{id}/pose` | `geometry_msgs/PoseStamped` | Camera pose (field frame) |
| `/robot_{id}/goal_pose` | `geometry_msgs/PoseStamped` | Target pose (x, y, and optionally yaw) |
| `/robot_{id}/waypoint` | `geometry_msgs/PointStamped` | Target position only |
| `/robot_{id}/vel` | `geometry_msgs/TwistStamped` | Onboard body-frame velocity (optional) |
| `/robot_{id}/attitude` | `geometry_msgs/QuaternionStamped` | Onboard IMU orientation (optional) |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/robot_{id}/cmd_vel` | `geometry_msgs/Twist` | Velocity command to the robot |
| `/robot_{id}/estimated_pose` | `geometry_msgs/PoseStamped` | Dead-reckoned + fused pose estimate |
| `/robot_{id}/current_goal` | `geometry_msgs/PoseStamped` | Echo of the active goal for rviz |
| `/robot_{id}/goal_marker` | `visualization_msgs/Marker` | Arrow/sphere marker at the goal |

**Parameters** (see [config/navigator.yaml](ros_ws/src/robot_navigator/config/navigator.yaml)):
| Parameter | Default | Description |
|---|---|---|
| `robot_id` | `0` | Robot to control |
| `linear_gain` | `0.8` | P-gain on position error |
| `angular_gain` | `1.2` | P-gain on heading error |
| `max_linear_speed` | `0.5` | Clamp (m/s) |
| `max_angular_speed` | `1.5` | Clamp (rad/s) |
| `goal_tolerance` | `0.05` | Position deadband (m) |
| `angular_tolerance` | `0.15` | Heading deadband (rad) |
| `align_heading` | `false` | Rotate to match goal yaw as well as position |
| `control_rate_hz` | `20.0` | Control + estimator tick rate |
| `camera_pose_timeout` | `1.0` | Stop if no camera pose for this long |
| `camera_trust` | `0.35` | Complementary filter weight on fresh camera poses |
| `onboard_sensor_timeout` | `0.3` | Fall back to command integration if `/vel` or `/attitude` silent for this long |
| `vel_y_flip` | `true` | Flip sign of `/vel.linear.y` (wire is y-right, integrator is y-left) |
| `attitude_yaw_offset` | `0.0` | IMU-yaw → field-yaw offset (auto-seeded from first camera fix) |
| `attitude_yaw_bias_trust` | `0.02` | How fast camera yaw corrects IMU drift; 0 to freeze the offset |
| `field_frame` | `"field"` | Expected frame for pose/goal messages |

## Prerequisites

- **apriltag_detector** publishing `/field/robot_{id}/pose`
- **robomaster_bridge** subscribing to `/robot_{id}/cmd_vel`

For the navigator to use onboard sensors, `/robot_{id}/vel` and
`/robot_{id}/attitude` must reach its ROS domain. They are published on the
robot's private domain (10) and **not bridged by default** — to enable them,
uncomment the corresponding blocks in
[robomaster_bridge/domain_bridge.yaml.template](../robomaster_bridge/domain_bridge.yaml.template)
and re-run `install.bash`, or run the navigator directly in domain 10.
When these topics are silent the navigator falls back to integrating its
last issued `cmd_vel`, which still works but drifts faster.

## Usage

```bash
cd s1-robomaster/robot_navigator
docker compose build
docker compose run --rm robot_navigator \
  ros2 launch robot_navigator navigator.launch.py robot_id:=0
```

Run one instance per robot:

```bash
./run_navigator.sh 0 1 2
```

### Running on NVIDIA Jetson

The `ros:humble-ros-base-jammy` base image is multi-arch and runs on arm64
without modification. The build step, however, often fails on L4T kernels
with:

```
iptables v1.8.7 (legacy): can't initialize iptables table `raw':
Table does not exist (do you need to insmod?)
```

L4T's kernel ships without `iptable_raw`, so Docker can't configure its
default bridge network during build. Two options:

1. **Host-networked build** (already set in
   [docker-compose.yml](docker-compose.yml) under `build.network: host`):

   ```bash
   docker compose build
   ```

   If that still fails (older docker-compose ignores the field), fall
   back to a direct build:

   ```bash
   DOCKER_BUILDKIT=0 docker build --network=host -t robot_navigator-robot_navigator .
   ```

2. **Load the missing module** (works on some JetPack revisions):

   ```bash
   sudo modprobe iptable_raw
   docker compose build
   ```

At runtime the service already uses `network_mode: host`, which also
bypasses the broken bridge, so no runtime changes are needed.

### Publishing goals

From rviz (Fixed Frame = `field`) or the command line:

```bash
# Full pose (drives to x, y; ignores yaw unless align_heading:=true)
ros2 topic pub --once /robot_0/goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "field"}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}'

# Or just a point
ros2 topic pub --once /robot_0/waypoint geometry_msgs/PointStamped \
  '{header: {frame_id: "field"}, point: {x: 1.0, y: 0.5}}'
```

### rviz displays

With Fixed Frame set to `field`:
- **Marker** on `/robot_{id}/goal_marker` — green arrow/sphere at the current target
- **Pose** on `/robot_{id}/current_goal` — pose-axes view of the target
- **Pose** on `/robot_{id}/estimated_pose` — the navigator's fused estimate (useful for debugging latency compensation)
- **MarkerArray** on `/field/markers` — field boundary from apriltag_detector

## Dead reckoning notes

Each integration step picks the best available velocity source and steps
the field-frame position estimate forward. When the IMU is live the yaw
estimate comes straight from `/attitude + attitude_yaw_offset` (no
integration drift); when `/vel` is live it drives position integration;
otherwise the last issued `cmd_vel` is used.

The IMU-to-field yaw offset auto-seeds from the first camera fix and then
drifts slowly toward camera yaw at rate `attitude_yaw_bias_trust`. Setting
that to zero freezes the offset after the initial seed.

Body-frame conventions: the RoboMaster wire format uses y-right for both
`cmd_vel` and `/vel`. The integrator works internally in y-left, so
`/vel.linear.y` is flipped on ingest (`vel_y_flip: true`) and `cmd_vel.linear.y`
is flipped on egress. If you see the robot strafe the wrong way, toggle
`vel_y_flip`.

If the robot is repeatedly overshooting, lower `camera_trust` (more smoothing,
more reliance on dead reckoning). If it jitters visibly when a camera frame
arrives, also lower `camera_trust`. If it lags behind its true position,
raise `camera_trust`.
