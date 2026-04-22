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

## Node: follower

Follower node that generates a waypoint for `robot_id` based on the tracked
position of another robot. The waypoint sits `follow_distance` metres from
the leader along the line between leader and follower, so the follower
naturally approaches to the configured separation and holds.

It publishes to `/robot_{id}/waypoint`, so the robot's own `robot_navigator`
instance must be running to act on the waypoint.

**Subscriptions:**
| Topic | Type | Description |
|---|---|---|
| `/field/robot_{robot_id}/pose` | `geometry_msgs/PoseStamped` | Follower's own pose |
| `/field/robot_{target_robot_id}/pose` | `geometry_msgs/PoseStamped` | Leader pose to track |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/robot_{robot_id}/waypoint` | `geometry_msgs/PointStamped` | Computed follow target |

**Parameters** (see [config/follower.yaml](ros_ws/src/robot_navigator/config/follower.yaml)):
| Parameter | Default | Description |
|---|---|---|
| `robot_id` | `0` | Follower's robot ID |
| `target_robot_id` | `1` | Leader's robot ID |
| `follow_distance` | `0.5` | Target separation (m) |
| `publish_rate_hz` | `5.0` | Waypoint publish rate |
| `stale_pose_timeout` | `1.0` | Pause if either pose is this old (s) |

### Launching a follower

```bash
docker compose run --rm robot_navigator \
  ros2 launch robot_navigator follower.launch.py \
    robot_id:=2 target_robot_id:=1 follow_distance:=0.4
```

Run alongside a `navigator` instance for the same `robot_id` — the navigator
consumes the waypoint and drives the robot.

## Node: global_feedback

Publishes a scalar signal describing how well a team of robots matches a
pre-configured formation in the field. Mirrors the simulator's
`global_reward_node` but against `/field/robot_{id}/pose` and rendered in
the `field` frame.

The reward is `-sum(distance(robot, target))` — zero when every target has a
robot on it, more negative as robots drift away. Higher is better.

**Subscriptions:**
| Topic | Type | Description |
|---|---|---|
| `/field/robot_{id}/pose` | `geometry_msgs/PoseStamped` | Live pose per configured robot |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/global_reward` | `std_msgs/Float32` | Negative total distance from targets |
| `/target_markers` | `visualization_msgs/MarkerArray` | Green cylinders + labels at each target (display in rviz) |

**Parameters** (see [config/global_feedback.yaml](ros_ws/src/robot_navigator/config/global_feedback.yaml)):
| Parameter | Default | Description |
|---|---|---|
| `robot_ids` | `[0, 1, 2]` | Robots included in the formation |
| `formation` | `"triangle"` | `line` \| `triangle` \| `circle` \| `custom` |
| `formation_center_x` / `_y` | `0.0` | Centre of the formation (field coords) |
| `formation_spacing` | `0.5` | Line step size, or ring radius |
| `formation_yaw` | `0.0` | Rotate the whole formation (rad) |
| `targets_flat` | — | Flat `[x0, y0, x1, y1, ...]` for `formation: custom` |
| `assignment` | `"ordered"` | `ordered` = target i ↔ `robot_ids[i]`; `nearest` = each target takes its closest robot |
| `publish_rate_hz` | `10.0` | Reward publish rate |
| `marker_rate_hz` | `1.0` | Target-marker refresh rate |
| `stale_pose_timeout` | `2.0` | Skip publish if any pose is this old (s) |

### Launching

```bash
# Triangle formation, ordered assignment (default)
docker compose run --rm robot_navigator \
  ros2 launch robot_navigator global_feedback.launch.py \
    formation:=triangle formation_spacing:=0.6

# Line of 3 robots
docker compose run --rm robot_navigator \
  ros2 launch robot_navigator global_feedback.launch.py \
    formation:=line formation_spacing:=0.4
```

### Live monitoring

```bash
ros2 topic echo /global_reward
```

In rviz add a **MarkerArray** display on `/target_markers` with Fixed Frame
`field` to see the configured positions.

## Node: belief

Per-robot Bayesian belief over the `/global_reward` landscape using Random
Fourier Features + online Bayesian Linear Regression. Pure-NumPy port of
the sim's `belief_node` — no torch, runs fine on Pi/Jetson.

Each time the robot has moved `displacement_threshold` metres, the node
decomposes the change in `/global_reward` along the displacement vector
into a 2-D gradient observation and feeds it into two independent BLRs
(x- and y-components) at the source grid cell.

**Subscriptions:**
| Topic | Type | Description |
|---|---|---|
| `/field/robot_{id}/pose` | `geometry_msgs/PoseStamped` | Robot's position in the field |
| `/global_reward` | `std_msgs/Float32` | Scalar feedback from `global_feedback` |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/robot_{id}/belief/uncertainty` | `nav_msgs/OccupancyGrid` | σ heatmap across the grid |
| `/robot_{id}/belief/gradient_mag` | `nav_msgs/OccupancyGrid` | ‖∇reward‖ heatmap |
| `/robot_{id}/belief/gradients` | `visualization_msgs/MarkerArray` | Arrow field of mean gradients |
| `/robot_{id}/waypoint` | `geometry_msgs/PointStamped` | UCB-selected target cell (only when `publish_waypoints:=true`) |

**Parameters** (see [config/belief.yaml](ros_ws/src/robot_navigator/config/belief.yaml)):
| Parameter | Default | Description |
|---|---|---|
| `robot_id` | `0` | Robot being tracked |
| `grid_resolution` | `0.2` | Cell size (m) |
| `grid_origin_x` / `_y` | `-1.5` | Bottom-left corner of grid in field coords |
| `grid_width` / `_height` | `15` | Cells per side (15×15 × 0.2 m = 3 m × 3 m) |
| `displacement_threshold` | `0.25` | Move this far before adding a sample (m) |
| `num_features` | `256` | RFF basis size |
| `lengthscale` | `0.6` | RBF kernel length scale |
| `prior_std`, `noise_std` | `1.0`, `0.5` | Prior / observation noise std dev |
| `forgetting` | `0.999` | BLR forgetting factor (<1 = adapt to drifting reward) |
| `publish_waypoints` | `false` | Emit UCB-chosen waypoints for the navigator |
| `ucb_alpha`, `ucb_beta` | `1.0`, `10.0` | UCB exploit/explore weights |

### Run for a single robot (test mode)

```bash
./run_belief.sh 0                   # belief visualisation only
./run_belief.sh 0 --with-waypoints  # belief + UCB waypoints
./run_belief.sh --rebuild 0         # force clean image rebuild
```

The belief node needs `global_feedback` running to receive `/global_reward`.
With `--with-waypoints`, also run a navigator for the same robot so the
waypoints get executed:

```bash
./run_global_feedback.sh            # in one terminal
./run_navigator.sh 0                # in another
./run_belief.sh 0 --with-waypoints  # in a third
```

### rviz displays (Fixed Frame = `field`)

- **Map** on `/robot_0/belief/uncertainty` — σ heatmap. Fresh areas stay
  bright; explored areas darken.
- **Map** on `/robot_0/belief/gradient_mag` — reward-gradient magnitude.
  Peaks indicate "reward slope" hotspots.
- **MarkerArray** on `/robot_0/belief/gradients` — mean-gradient arrow
  field, coloured red→green by per-cell uncertainty.

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
