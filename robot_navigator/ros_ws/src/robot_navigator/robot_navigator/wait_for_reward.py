"""One-shot helper for staged experiments: subscribe to `/global_reward`
and exit 0 once the value stays above THRESHOLD for HOLD_SEC seconds, or
exit 1 if TIMEOUT_SEC elapses first.

Used to auto-detect "formation reached" so the experiment harness can
transition phases as soon as the team has converged, rather than on a
fixed timer.

Usage (inside the docker image, after build/install):
    ros2 run robot_navigator wait_for_reward THRESHOLD HOLD_SEC TIMEOUT_SEC

Reward semantics: global_feedback publishes -Σ(distance_to_target). Higher
is better; 0 means every robot is exactly on its target. A threshold like
-0.2 corresponds to roughly 7 cm average error per robot in a 3-robot
team — tight enough to call "converged" without demanding pixel-perfect
positions.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RewardWaiter(Node):
    def __init__(self, threshold: float, hold_sec: float, timeout_sec: float):
        super().__init__("reward_waiter")
        self.threshold = threshold
        self.hold_sec = hold_sec
        self.timeout_sec = timeout_sec
        self.start = time.monotonic()
        self.first_above: float | None = None
        self.last_reward: float = float("-inf")
        self.received_any = False
        self.silent_warned = False
        self.exit_code = 1  # default to "timed out" until proven otherwise
        self.create_subscription(Float32, "/global_reward", self._on_reward, 10)
        self.create_timer(0.5, self._check_timeout)
        self.get_logger().info(
            f"Waiting for /global_reward > {threshold} held for {hold_sec:.1f}s "
            f"(timeout {timeout_sec:.1f}s)."
        )

    def _on_reward(self, msg: Float32) -> None:
        now = time.monotonic()
        self.last_reward = float(msg.data)
        self.received_any = True
        if self.last_reward > self.threshold:
            if self.first_above is None:
                self.first_above = now
                self.get_logger().info(
                    f"Reward {self.last_reward:.3f} crossed threshold; "
                    f"holding for {self.hold_sec:.1f}s..."
                )
            elif now - self.first_above >= self.hold_sec:
                self.get_logger().info(
                    f"Held above {self.threshold} for {self.hold_sec:.1f}s "
                    f"(reward {self.last_reward:.3f}). Transitioning."
                )
                self.exit_code = 0
                rclpy.shutdown()
        else:
            if self.first_above is not None:
                self.get_logger().info(
                    f"Reward {self.last_reward:.3f} dropped below "
                    f"{self.threshold}; resetting hold counter."
                )
            self.first_above = None

    def _check_timeout(self) -> None:
        elapsed = time.monotonic() - self.start
        # Flag a silent publisher early — if no /global_reward message has
        # arrived after 5 s, something upstream is wrong. One-shot via
        # `silent_warned` so timer jitter can't cause us to miss a narrow
        # time window (or fire repeatedly).
        if not self.received_any and not self.silent_warned and elapsed > 5.0:
            self.silent_warned = True
            self.get_logger().warn(
                "No /global_reward messages yet — is global_feedback "
                "running and seeing all robot poses?"
            )
        if elapsed > self.timeout_sec:
            self.get_logger().warn(
                f"Timeout after {self.timeout_sec:.1f}s — last reward "
                f"{self.last_reward:.3f}. Proceeding anyway."
            )
            self.exit_code = 1
            rclpy.shutdown()


def main():
    if len(sys.argv) < 4:
        print(
            "Usage: wait_for_reward THRESHOLD HOLD_SEC TIMEOUT_SEC",
            file=sys.stderr,
        )
        sys.exit(2)
    threshold = float(sys.argv[1])
    hold_sec = float(sys.argv[2])
    timeout_sec = float(sys.argv[3])

    rclpy.init()
    node = RewardWaiter(threshold, hold_sec, timeout_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_code = 130
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
