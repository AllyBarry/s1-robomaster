# Navigation Demo

Demo ROS 2 package that drives a RoboMaster S1 to a target point using vision-based localization from the `apriltag_detector` module.

## Nodes

### go_to_point

Proportional controller that navigates the robot toward a clicked point. The S1's mecanum wheels allow holonomic movement, so the robot drives directly toward the goal in both X and Y.

**Subscriptions:**
| Topic | Type | Description |
|---|---|---|
| `/field/robot_{id}/pose` | `geometry_msgs/PoseStamped` | Robot position from field_localizer |
| `/clicked_point` | `geometry_msgs/PointStamped` | Target from rviz "Publish Point" tool |

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command to the robot |

**Parameters:**
| Parameter | Default | Description |
|---|---|---|
| `robot_id` | `4` | AprilTag ID of the robot to control |
| `linear_gain` | `0.8` | Proportional gain |
| `max_linear_speed` | `0.5` | Maximum velocity (m/s) |
| `goal_tolerance` | `0.05` | Stop distance from goal (m) |

### field_visualizer

Publishes rviz markers showing the field boundary and corner tag positions. Uses the same `field.yaml` configuration as the apriltag_detector.

**Publications:**
| Topic | Type | Description |
|---|---|---|
| `/field/markers` | `visualization_msgs/MarkerArray` | Field corners (orange cylinders), labels, and boundary outline (green) |

## Prerequisites

The following modules must be running:
- **rpi_camera_streamer** — publishes camera images
- **apriltag_detector** — detects tags and publishes `/field/robot_{id}/pose`
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
- **MarkerArray** on `/field/markers` — shows the field boundary and corner tags
- **PoseArray** on `/field/robot_poses` — shows detected robot positions
- Set the **Fixed Frame** to `field`

## Configuration

Edit `config/navigation.yaml` to tune the controller, or `config/field.yaml` to match your physical field layout. If you change the field dimensions or corner tag IDs, update `field.yaml` in both this module and `apriltag_detector`.
