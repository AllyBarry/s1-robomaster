# Navigation Demo

Demo ROS 2 package that drives a RoboMaster S1 to a target point using vision-based localization from the `apriltag_detector` module.

## Node: go_to_point

Proportional controller that navigates the robot toward a clicked point. The S1's mecanum wheels allow holonomic movement, so the robot drives directly toward the goal in both X and Y.

**Subscriptions:**
| Topic | Type | Description |
|---|---|---|
| `/field/robot_{id}/pose` | `geometry_msgs/PoseStamped` | Robot position from field_localizer |
| `/clicked_point` | `geometry_msgs/PointStamped` | Target from rviz "Publish Point" tool |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/robot_{id}/cmd_vel` | `geometry_msgs/Twist` | Velocity command to the robot |

**Parameters:**
| Parameter | Default | Description |
|---|---|---|
| `robot_id` | `4` | AprilTag ID of the robot to control |
| `linear_gain` | `0.8` | Proportional gain |
| `max_linear_speed` | `0.5` | Maximum velocity (m/s) |
| `goal_tolerance` | `0.05` | Stop distance from goal (m) |

## Prerequisites

The following modules must be running:
- **rpi_camera_streamer** — publishes camera images
- **apriltag_detector** — detects tags, publishes `/field/robot_{id}/pose`, and visualizes the field on `/field/markers`
- **robomaster_bridge** — subscribes to `/cmd_vel` and drives the motors

## Usage

### Build and run

```bash
cd s1-robomaster/navigation_demo
docker compose build
docker compose up
```

### Override the robot ID

```bash
docker compose run navigation_demo \
  ros2 launch navigation_demo navigation.launch.py robot_id:=5
```

### Set a goal

In rviz, use the **Publish Point** tool (toolbar) and click anywhere on the field. The robot will drive to that point and stop.

### Visualize in rviz

Add the following displays:
- **MarkerArray** on `/field/markers` — shows the field boundary and corner tags (published by apriltag_detector)
- **PoseArray** on `/field/robot_poses` — shows detected robot positions
- Set the **Fixed Frame** to `field`

## Configuration

Edit `config/navigation.yaml` to tune the controller.
