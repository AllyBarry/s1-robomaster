## Camera Streaming and Calibration README

### Overview

This setup runs a ROS 2 camera streamer in Docker on the Raspberry Pi and supports camera calibration using a separate Docker Compose service.

The main points were:

* expose host devices so the camera is visible in the container
* forward X11 so GUI tools like `cameracalibrator` can open
* verify GUI works with a small OpenCV test
* verify the camera topic is actually streaming
* then run calibration

---

## Docker Compose layout

There are two services:

* `camera`
  starts the webcam ROS node and publishes topics

* `calibrate`
  runs the ROS calibration GUI and subscribes to the camera topics

Both use the same image, but run as separate containers.

---

## Compose file structure

Shared requirements:

* `/dev:/dev` so camera devices are visible
* X11 mounts for GUI
* camera info folder mounted so calibration persists
* `network_mode: host` so ROS 2 discovery works

---

## Start camera streaming

```bash
docker compose up camera
```

This should start the ROS camera node.

---

## Run calibration

In another terminal:

```bash
docker compose run --rm calibrate
```

This opens the calibration GUI.

Use a checkerboard, then click:

* **CALIBRATE**
* **SAVE**

---

## Test commands

### 1. Check camera device on host

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

Use this to confirm which `/dev/videoN` is the real camera.

---

### 2. Check camera stream on host with ffmpeg

MJPG:

```bash
ffmpeg -f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 -frames 1 mjpg_test.jpg
```

YUYV:

```bash
ffmpeg -f v4l2 -input_format yuyv422 -video_size 640x480 -i /dev/video0 -frames 1 yuyv_test.jpg
```

This confirms the camera can capture outside Docker.

---

### 3. Check GUI forwarding inside container

Test OpenCV GUI:

```bash
python3 -c "import cv2, numpy as np; img=np.zeros((300,300,3),dtype=np.uint8); cv2.imshow('test', img); cv2.waitKey(0)"
```

If this fails, calibration GUI will also fail.

You can also test:

```bash
xclock
```

---

### 4. Check ROS topics are publishing

```bash
ros2 topic list
ros2 topic hz /webcam/image_raw
ros2 topic echo /webcam/camera_info --once
```

Expected topics:

```text
/webcam/image_raw
/webcam/camera_info
```

If `/webcam/camera_info` is all zeros, calibration has not been loaded yet.

---

### 5. Run calibration manually

If needed directly inside the container:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 4x6 \
  --square 0.04 \
  --ros-args \
  -r image:=/webcam/image_raw \
  -r camera:=/webcam
```

Adjust:

* `4x6` = inner checkerboard corners
* `0.04` = square size in meters

---

### 6. Check saved calibration file

Calibration should save to:

```bash
ls -l /root/.ros/camera_info/
```

Expected file:

```text
logitech_brio.yaml
```

Because `./camera_info` is mounted into `/root/.ros/camera_info`, it should also appear on the host in:

```text
./camera_info/
```

---

### 7. Verify calibration loads

Restart the camera service:

```bash
docker compose restart camera
```

Then check:

```bash
ros2 topic echo /webcam/camera_info --once
```

Now the output should contain:

* nonzero `k`
* nonzero `d`
* nonzero `p`
* valid distortion model

---

## Troubleshooting

### Camera not found

Check:

```bash
lsusb
ls -l /dev/video*
v4l2-ctl --list-devices
```

If `/dev/video0` does not exist, the camera is not available on the host.

---

### GUI not opening

Check:

```bash
echo $DISPLAY
xclock
```

If GUI does not work, calibration GUI will fail too.

---

### Camera publishes but calibration window is black

Usually means the calibrator is subscribed to the wrong topic.

Use:

```bash
-r image:=/webcam/image_raw -r camera:=/webcam
```

not `/ir/image` unless that is really your topic.

---

### Old containers piling up

Clean them with:

```bash
docker compose down --remove-orphans
```

---

## Typical workflow

### Start streaming

```bash
docker compose up camera
```

### Confirm topics

```bash
ros2 topic list
ros2 topic hz /webcam/image_raw
```

### Run calibration

```bash
docker compose run --rm calibrate
```

### Save calibration

Then restart camera:

```bash
docker compose restart camera
```

### Confirm calibration loaded

```bash
ros2 topic echo /webcam/camera_info --once
```

---

## Final architecture

```text
BRIO
  ↓
camera container
  ↓
/webcam/image_raw
/webcam/camera_info
  ↓
calibration container
  ↓
saved YAML
  ↓
camera node reloads calibration
```

If you want, I can also turn this into a polished markdown `README.md` file block you can paste directly into your repo.

`xhost +local:…` is needed because Docker containers are **not trusted by the X server** by default.

When you forward X11, three things must happen:

1. container must know which display to use (`DISPLAY`)
2. container must access the X11 socket (`/tmp/.X11-unix`)
3. X server must **allow** the container to connect → this is what `xhost` does

Without step 3, you get errors like:

* cannot open display
* GTK backend failed
* OpenCV window not showing
* connection refused

## What `xhost +local:root` does

It tells the X server:

> allow local processes running as root to connect to this display

Your container runs as `root`, so this allows GUI apps inside Docker to open windows.

## Why Docker needs it

From X11’s perspective:

* your desktop session = trusted
* Docker container = unknown client
* unknown clients are blocked

So you explicitly allow:

```bash
xhost +local:root
```

Now container GUI apps work.

## What happens without it

You typically see:

```text
cannot open display
GTK backend error
connection refused
cv2.namedWindow failed
```

Exactly what you hit.

## What `local` means

`local:` = UNIX socket clients (like Docker containers)

not remote network hosts.

So this is safer than:

```bash
xhost +
```

which allows everything.

## Safer versions

Allow only root local:

```bash
xhost +local:root
```

Allow only current user:

```bash
xhost +SI:localuser:$(whoami)
```

Remove permission later:

```bash
xhost -local:root
```

## Full pipeline

You need all three:

Docker compose:

```yaml
environment:
  - DISPLAY=${DISPLAY}

volumes:
  - /tmp/.X11-unix:/tmp/.X11-unix
  - ${HOME}/.Xauthority:/root/.Xauthority
```

Host:

```bash
xhost +local:root
```

Then GUI works.

## Summary

You need `xhost` because:

* container is not trusted by X server
* GUI apps need permission to open windows
* `xhost +local:root` grants that permission

- After calibration you will find in `/tmp/calibrationdata.tar` `ost.yaml` which you must uncompress and put in the `.../config` 
in `ros_ws`. Remember the name of the camera must match the config 




# Camera
h BRIO Camera Controls (/dev/video1)

Use:

export CAM=/dev/video1

This is the main controllable video device for the Logitech Brio on your Jetson.

⸻

Why /dev/video1

The Brio exposes multiple device nodes:

/dev/video1
/dev/video2
/dev/video3
/dev/video4

We use /dev/video1 because it contains the camera controls:

* focus
* exposure
* gain
* white balance
* zoom
* pan / tilt

Check controls:

v4l2-ctl -d /dev/video1 --list-ctrls

⸻

Focus

Controls image sharpness.

Disable autofocus

sudo v4l2-ctl -d $CAM -c focus_automatic_continuous=0

Set manual focus

sudo v4l2-ctl -d $CAM -c focus_absolute=40

Use for AprilTags to stop focus hunting.

⸻

Exposure

Controls brightness by changing sensor shutter time.

Enable manual exposure

sudo v4l2-ctl -d $CAM -c auto_exposure=1

Set exposure time

sudo v4l2-ctl -d $CAM -c exposure_time_absolute=80

Lower = darker but sharper
Higher = brighter but blurrier

Use lower values for moving robots / sharp tags.

⸻

Gain

Electronic brightness amplification.

sudo v4l2-ctl -d $CAM -c gain=15

Higher = brighter but noisier.

Use only if image is too dark after exposure tuning.

⸻

White Balance

Controls image color temperature.

Disable auto white balance

sudo v4l2-ctl -d $CAM -c white_balance_automatic=0

Set temperature

sudo v4l2-ctl -d $CAM -c white_balance_temperature=4500

Mostly useful for color consistency.

⸻

Sharpness

Digital edge enhancement.

sudo v4l2-ctl -d $CAM -c sharpness=140

Useful for AprilTag edges in moderation.

⸻

Backlight Compensation

Adjusts for bright background scenes.

sudo v4l2-ctl -d $CAM -c backlight_compensation=0

Usually disable for controlled indoor robotics.

⸻

Recommended AprilTag Settings

udo v4l2-ctl -d $CAM \
-c auto_exposure=1 \
-c focus_automatic_continuous=0 \
-c white_balance_automatic=0 \
-c exposure_time_absolute=350 \
-c exposure_dynamic_framerate=0 \
-c pan_absolute=0 \
-c tilt_absolute=0 \
-c focus_absolute=0 \
-c gain=15 \
-c white_balance_temperature=4500 \
-c brightness=128 \
-c contrast=145 \
-c saturation=100 \
-c sharpness=255 \
-c backlight_compensation=0 \
-c power_line_frequency=2
⸻

View Camera Without ROS

ffplay $CAM

or

guvcview -d $CAM

⸻

Show Current Settings

v4l2-ctl -d $CAM --all

