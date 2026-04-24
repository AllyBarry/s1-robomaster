# camera_broker

Single-source v4l2 relay so multiple containers can open the webcam at once.

Reads the real camera (`/dev/video1`, Logitech BRIO) and writes to a
v4l2loopback virtual node (`/dev/video20`). Both `rpi_camera_streamer` and
`apriltag_detector` default to `/dev/video20`, so neither fights the other
for the real device.

## One-time host setup (Jetson)

```bash
sudo apt install -y v4l2loopback-dkms v4l2loopback-utils linux-headers-$(uname -r)

# Persist module + options across reboots
echo v4l2loopback | sudo tee /etc/modules-load.d/v4l2loopback.conf
echo 'options v4l2loopback video_nr=20 card_label="camera_broker" exclusive_caps=1' \
  | sudo tee /etc/modprobe.d/v4l2loopback.conf

# Load now
sudo modprobe -r v4l2loopback 2>/dev/null || true
sudo modprobe v4l2loopback

ls -l /dev/video20   # should exist
```

If `modprobe` fails with "module not found", install matching kernel headers
and re-run `sudo dpkg-reconfigure v4l2loopback-dkms`.

## Startup order

Bring the broker up first; it owns `/dev/video1`:

```bash
cd rpi_ir_apriltag/camera_broker
docker compose up -d
docker compose logs -f broker   # should show "relaying ..." with no errors
```

Then either or both consumers:

```bash
cd ../rpi_camera_streamer && docker compose up -d camera
cd ../apriltag_detector  && docker compose up -d apriltag_detector
```

Both now read `/dev/video20` independently. Consumers can still be pointed at
the real device for debugging with `video_device:=/dev/video1` at launch time.

## Camera controls

`v4l2-ctl` controls (focus, exposure, white balance, etc.) apply to the real
device `/dev/video1`, not the loopback. Apply them from the host while the
broker is up:

```bash
sudo v4l2-ctl -d /dev/video1 \
  -c focus_automatic_continuous=0 \
  -c focus_absolute=40 \
  -c auto_exposure=1 \
  -c exposure_time_absolute=350
```

Full recommended AprilTag settings are in `../rpi_camera_streamer/README.md`.

## Tuning resolution / FPS

Override via env in `docker-compose.yml` — defaults are `640x480 @ 30 YUYV` to
match the consumers' current launch parameters. Changing them here without
changing the consumers' `image_size` launch args will break capture.
