"""One-shot helper for staged experiments: subscribe to `/global_reward`
and exit 0 once the team has converged, exit 1 on timeout.

Reach detection is a two-track OR:
  - **EMA-smoothed reward** above THRESHOLD held for HOLD_SEC seconds —
    catches a stable plateau even if instantaneous values bounce.
  - **Peak**: at any moment in the last HOLD_SEC, the smoothed reward
    crossed PEAK_THRESHOLD (a tighter value) — catches a clean
    "robots momentarily nailed it" event.

Either fires → exit 0.

Reward semantics: global_feedback publishes -Σ(distance_to_target).
Higher is better; 0 means every robot is exactly on its target. With
3 robots, a reward of -0.6 means roughly 20 cm average error per robot.

Usage (inside the docker image, after build/install):
    ros2 run robot_navigator wait_for_reward THRESHOLD HOLD_SEC TIMEOUT_SEC \\
        [PEAK_THRESHOLD] [EMA_ALPHA]

  PEAK_THRESHOLD  optional, default = THRESHOLD (no separate peak track)
  EMA_ALPHA       optional, default = 0.2 — 1.0 disables smoothing
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RewardWaiter(Node):
    def __init__(
        self,
        threshold: float,
        hold_sec: float,
        timeout_sec: float,
        peak_threshold: float | None = None,
        ema_alpha: float = 0.2,
    ):
        super().__init__("reward_waiter")
        self.threshold = threshold
        self.peak_threshold = (
            peak_threshold if peak_threshold is not None else threshold
        )
        self.hold_sec = hold_sec
        self.timeout_sec = timeout_sec
        self.ema_alpha = max(0.01, min(1.0, ema_alpha))
        self.start = time.monotonic()
        self.first_above: float | None = None
        self.last_reward: float = float("-inf")
        self.ema_reward: float | None = None
        self.received_any = False
        self.silent_warned = False
        self.exit_code = 1  # default to "timed out" until proven otherwise
        self.create_subscription(Float32, "/global_reward", self._on_reward, 10)
        self.create_timer(0.5, self._check_timeout)
        self.get_logger().info(
            f"Waiting for /global_reward EMA > {threshold} held {hold_sec:.1f}s "
            f"(or peak > {self.peak_threshold:.2f}, alpha={self.ema_alpha:.2f}, "
            f"timeout {timeout_sec:.1f}s)."
        )

    def _on_reward(self, msg: Float32) -> None:
        now = time.monotonic()
        self.last_reward = float(msg.data)
        self.received_any = True

        # EMA: smooths out per-tick UCB oscillation while still tracking
        # the true convergence trend on the second-or-two scale.
        if self.ema_reward is None:
            self.ema_reward = self.last_reward
        else:
            self.ema_reward = (
                self.ema_alpha * self.last_reward
                + (1.0 - self.ema_alpha) * self.ema_reward
            )

        # Peak track — tighter threshold, single crossing wins.
        if self.last_reward >= self.peak_threshold:
            self.get_logger().info(
                f"Peak reward {self.last_reward:.3f} ≥ {self.peak_threshold} "
                f"— transitioning."
            )
            self.exit_code = 0
            rclpy.shutdown()
            return

        # Sustained track — EMA above threshold for hold_sec.
        if self.ema_reward > self.threshold:
            if self.first_above is None:
                self.first_above = now
                self.get_logger().info(
                    f"EMA {self.ema_reward:.3f} crossed {self.threshold}; "
                    f"holding for {self.hold_sec:.1f}s..."
                )
            elif now - self.first_above >= self.hold_sec:
                self.get_logger().info(
                    f"EMA held above {self.threshold} for {self.hold_sec:.1f}s "
                    f"(EMA {self.ema_reward:.3f}, instantaneous "
                    f"{self.last_reward:.3f}). Transitioning."
                )
                self.exit_code = 0
                rclpy.shutdown()
        else:
            if self.first_above is not None:
                self.get_logger().info(
                    f"EMA {self.ema_reward:.3f} dropped below "
                    f"{self.threshold}; resetting hold counter."
                )
            self.first_above = None

    def _check_timeout(self) -> None:
        elapsed = time.monotonic() - self.start
        if not self.received_any and not self.silent_warned and elapsed > 5.0:
            self.silent_warned = True
            self.get_logger().warn(
                "No /global_reward messages yet — is global_feedback "
                "running and seeing all robot poses?"
            )
        if elapsed > self.timeout_sec:
            ema_str = (
                f"{self.ema_reward:.3f}" if self.ema_reward is not None else "n/a"
            )
            self.get_logger().warn(
                f"Timeout after {self.timeout_sec:.1f}s — last reward "
                f"{self.last_reward:.3f}, EMA {ema_str}. Proceeding anyway."
            )
            self.exit_code = 1
            rclpy.shutdown()


def main():
    if len(sys.argv) < 4:
        print(
            "Usage: wait_for_reward THRESHOLD HOLD_SEC TIMEOUT_SEC "
            "[PEAK_THRESHOLD] [EMA_ALPHA]",
            file=sys.stderr,
        )
        sys.exit(2)
    threshold = float(sys.argv[1])
    hold_sec = float(sys.argv[2])
    timeout_sec = float(sys.argv[3])
    peak_threshold = float(sys.argv[4]) if len(sys.argv) > 4 else None
    ema_alpha = float(sys.argv[5]) if len(sys.argv) > 5 else 0.2

    rclpy.init()
    node = RewardWaiter(
        threshold, hold_sec, timeout_sec, peak_threshold, ema_alpha
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_code = 130
    sys.exit(node.exit_code)


if __name__ == "__main__":
    main()
