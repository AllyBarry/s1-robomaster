import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamWatchdog(Node):
    def __init__(self):
        super().__init__("webcam_watchdog")

        self.declare_parameter("image_topic", "/webcam/image_raw")
        self.declare_parameter("startup_timeout_sec", 15.0)
        self.declare_parameter("stale_timeout_sec", 5.0)
        self.declare_parameter("check_period_sec", 1.0)

        image_topic = str(self.get_parameter("image_topic").value)
        self.startup_timeout_sec = float(
            self.get_parameter("startup_timeout_sec").value
        )
        self.stale_timeout_sec = float(
            self.get_parameter("stale_timeout_sec").value
        )
        check_period_sec = float(self.get_parameter("check_period_sec").value)

        self.start_time = self.get_clock().now()
        self.last_frame_time = None

        self.sub = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )
        self.timer = self.create_timer(check_period_sec, self._check_stream)

        self.get_logger().info(
            f"Watching {image_topic} "
            f"(startup_timeout_sec={self.startup_timeout_sec:.1f}, "
            f"stale_timeout_sec={self.stale_timeout_sec:.1f})"
        )

    def _on_image(self, _msg: Image):
        self.last_frame_time = self.get_clock().now()

    def _check_stream(self):
        now = self.get_clock().now()

        if self.last_frame_time is None:
            startup_age = (now - self.start_time).nanoseconds / 1e9
            if startup_age > self.startup_timeout_sec:
                self.get_logger().error(
                    "No frames received before startup timeout; "
                    "exiting so the container can restart."
                )
                raise RuntimeError("camera stream never became active")
            return

        frame_age = (now - self.last_frame_time).nanoseconds / 1e9
        if frame_age > self.stale_timeout_sec:
            self.get_logger().error(
                f"Camera stream stale for {frame_age:.1f}s; "
                "exiting so the container can restart."
            )
            raise RuntimeError("camera stream became stale")


def main(args=None):
    rclpy.init(args=args)
    node = WebcamWatchdog()
    try:
        rclpy.spin(node)
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
