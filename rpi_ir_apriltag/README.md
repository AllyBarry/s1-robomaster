# rpi_ir_apriltag

ROS 2 (Humble) bringup package for an IR camera with AprilTag detection, designed to run on a Raspberry Pi inside a Docker container.

The pipeline streams raw IR images from an OpenNI2-compatible USB camera, rectifies them, and runs AprilTag pose estimation — enabling 6-DOF marker detection for robot localisation or docking tasks.

---

## Architecture

```
USB IR Camera (OpenNI2)
        │
        ▼
 /ir/image  +  /ir/camera_info
        │
        ▼
  [ir_rectify node]          ← currently disabled
        │
        ▼
    /ir/image_rect
        │
        ▼
  [apriltag node]            ← currently disabled
        │
        ▼
  AprilTag detections (pose + ID)
```

---

## ROS Nodes

### `openni2_camera` — IR Camera Driver

| | |
|---|---|
| **Package** | `openni2_camera` |
| **Plugin** | `openni2_wrapper::OpenNI2Driver` |
| **Type** | Composable node (runs in `camera_container`) |
| **Status** | **Active** |

Streams raw IR frames from an OpenNI2-compatible USB camera (Kinect v1, ASUS Xtion, PrimeSense, etc.). RGB and depth streams are disabled; only IR is enabled.

**Published topics:**

| Topic | Type | Description |
|---|---|---|
| `/ir/image` | `sensor_msgs/Image` | Raw IR image stream |
| `/ir/camera_info` | `sensor_msgs/CameraInfo` | IR camera calibration data |

**Key parameters:**

| Parameter | Value | Description |
|---|---|---|
| `enable_ir` | `true` | Enable IR stream |
| `enable_color` | `false` | RGB stream disabled |
| `enable_depth` | `false` | Depth stream disabled |
| `depth_registration` | `false` | Depth-to-colour alignment off |
| `use_device_time` | `true` | Use camera hardware timestamps |

---

### `ir_rectify` — Image Rectification

| | |
|---|---|
| **Package** | `image_proc` |
| **Executable** | `rectify_node` |
| **Type** | Standalone node |
| **Status** | **Disabled** (commented out in launch file) |

Undistorts raw IR images using the camera calibration parameters, producing a geometrically corrected image ready for AprilTag detection.

**Subscribed topics:**

| Topic | Remapped from | Description |
|---|---|---|
| `image` | `/ir/image` | Raw IR image |
| `camera_info` | `/ir/camera_info` | Calibration data |

**Published topics:**

| Topic | Remapped to | Description |
|---|---|---|
| `image_rect` | `/ir/image_rect` | Rectified IR image |

---

### `apriltag` — AprilTag Detection

| | |
|---|---|
| **Package** | `apriltag_ros` |
| **Executable** | `apriltag_node` |
| **Status** | **Disabled** (commented out in launch file) |

Detects AprilTag markers in the rectified IR image stream and estimates their 6-DOF pose (position + orientation) relative to the camera frame.

**Subscribed topics:**

| Topic | Remapped from | Description |
|---|---|---|
| `image_rect` | `/ir/image_rect` | Rectified IR image |
| `camera_info` | `/ir/camera_info` | Calibration data |

**Detection parameters** (from [config/apriltag.yaml](ros_ws/src/rpi_ir_bringup/config/apriltag.yaml)):

| Parameter | Value | Description |
|---|---|---|
| `family` | `36h11` | AprilTag family |
| `size` | `0.162` m | Physical tag side length |
| `max_hamming` | `0` | Maximum bit errors allowed |
| `threads` | `2` | Detector threads |
| `decimate` | `2.0` | Image downscaling factor |
| `blur` | `0.0` | Pre-detection Gaussian blur |
| `refine` | `true` | Sub-pixel corner refinement |
| `sharpening` | `0.25` | Pre-detection sharpening |
| `pose_estimation_method` | `pnp` | 3D pose via Perspective-n-Point |

---

## Configuration Files

| File | Purpose |
|---|---|
| [config/camera.yaml](ros_ws/src/rpi_ir_bringup/config/camera.yaml) | v4l2 USB camera settings (device, resolution, pixel format) |
| [config/cam_calibration.yaml](ros_ws/src/rpi_ir_bringup/config/cam_calibration.yaml) | Camera intrinsics (focal length, principal point, distortion) |
| [config/apriltag.yaml](ros_ws/src/rpi_ir_bringup/config/apriltag.yaml) | AprilTag detector parameters |
| [rgb_PS1080_PrimeSense.yaml](ros_ws/src/rpi_ir_bringup/rgb_PS1080_PrimeSense.yaml) | PrimeSense RGB calibration (loaded into Docker container) |
| [depth_PS1080_PrimeSense.yaml](ros_ws/src/rpi_ir_bringup/depth_PS1080_PrimeSense.yaml) | PrimeSense depth calibration (loaded into Docker container) |

> **Note:** `cam_calibration.yaml` ships with placeholder intrinsics (all-zero distortion, approximate focal lengths). Replace with values from a proper camera calibration run before using the rectification or AprilTag nodes.

---

## Launch File

**[launch/camera_apriltag.launch.py](ros_ws/src/rpi_ir_bringup/launch/camera_apriltag.launch.py)**

Starts a `rclcpp_components` container (`camera_container`) and loads the OpenNI2 camera driver into it as a composable node. The rectification and AprilTag nodes are present in the file but commented out — uncomment them to enable the full pipeline.

---

## Docker Deployment

The package is containerised for easy deployment on a Raspberry Pi.

### Build & Run

```bash
# Build image
docker compose build

# Run (attaches USB bus and uses host networking)
docker compose up
```

### What the container does

- Base image: `ros:humble-ros-base-jammy` (Ubuntu 22.04)
- Installs `openni2-camera`, `image-proc`, `apriltag-ros`, and system dependencies
- Copies PrimeSense calibration files into `/root/.ros/camera_info/`
- Builds the workspace with `colcon build --symlink-install`
- On startup: sources ROS and workspace, then launches `camera_apriltag.launch.py`

### docker-compose settings

| Setting | Value | Reason |
|---|---|---|
| `network_mode` | host | ROS 2 DDS discovery across host network |
| `ipc` | host | Shared memory transport for zero-copy topics |
| `privileged` | true | Required for USB device access |
| `ROS_DOMAIN_ID` | 0 | ROS 2 domain |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | CycloneDDS middleware |

---

## Enabling the Full Pipeline

To activate rectification and AprilTag detection, open [launch/camera_apriltag.launch.py](ros_ws/src/rpi_ir_bringup/launch/camera_apriltag.launch.py) and uncomment the `ir_rectify` and `apriltag` node definitions. Then rebuild and relaunch:

```bash
# Inside the container or after sourcing the workspace
ros2 launch rpi_ir_bringup camera_apriltag.launch.py
```

Before enabling the rectify node, calibrate the camera and update `cam_calibration.yaml` with real intrinsic values.

---

## Dependencies

- ROS 2 Humble
- `openni2_camera`
- `image_proc`
- `apriltag_ros`
- `rmw-cyclonedds-cpp`
