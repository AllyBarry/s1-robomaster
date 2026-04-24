#!/bin/bash
set -e

SRC_DEVICE="${SRC_DEVICE:-/dev/video1}"
DST_DEVICE="${DST_DEVICE:-/dev/video20}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"

echo "camera_broker: waiting for $SRC_DEVICE and $DST_DEVICE ..."
for _ in $(seq 1 30); do
  if [ -e "$SRC_DEVICE" ] && [ -e "$DST_DEVICE" ]; then
    break
  fi
  sleep 1
done

if [ ! -e "$SRC_DEVICE" ]; then
  echo "camera_broker: source $SRC_DEVICE not present on host" >&2
  exit 1
fi
if [ ! -e "$DST_DEVICE" ]; then
  echo "camera_broker: loopback $DST_DEVICE not present - is v4l2loopback loaded on the host?" >&2
  exit 1
fi

echo "camera_broker: relaying $SRC_DEVICE -> $DST_DEVICE (${WIDTH}x${HEIGHT} @ ${FPS}fps YUYV)"
exec ffmpeg -nostdin -hide_banner -loglevel warning \
  -f v4l2 -input_format yuyv422 -framerate "$FPS" -video_size "${WIDTH}x${HEIGHT}" \
  -i "$SRC_DEVICE" \
  -f v4l2 -vcodec rawvideo -pix_fmt yuyv422 "$DST_DEVICE"
