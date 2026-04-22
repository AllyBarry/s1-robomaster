"""
Per-robot GP belief node — NumPy RFF+BLR port of the sim's belief_node,
adapted for the field-frame tag-camera setup.

Subscribes to:
    /field/robot_{id}/pose   geometry_msgs/PoseStamped
    /global_reward           std_msgs/Float32

Publishes:
    /robot_{id}/belief/uncertainty   nav_msgs/OccupancyGrid   (σ heatmap)
    /robot_{id}/belief/gradient_mag  nav_msgs/OccupancyGrid   (|∇reward| heatmap)
    /robot_{id}/belief/gradients     visualization_msgs/MarkerArray (arrows)
    /robot_{id}/waypoint             geometry_msgs/PointStamped   (optional)

Update rule: cache (position, reward). Once the robot has moved
`displacement_threshold` metres, decompose Δreward along the displacement
into a 2-D gradient observation at the *source* grid cell and feed it into
two independent Bayesian linear regressions (x- and y-components) over
Random Fourier Features.

If `publish_waypoints` is true, the node also runs UCB on its current
gradient belief and publishes a waypoint for the `robot_navigator` to
drive to. Otherwise it's purely a belief-visualisation node.
"""

import math
import random

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker, MarkerArray


# ------------------------------------------------------------------ #
#  RFF + BLR (NumPy)
# ------------------------------------------------------------------ #

class RFFFeatureMap:
    def __init__(self, in_dim=2, num_features=256, lengthscale=0.5,
                 rng: np.random.Generator | None = None):
        rng = rng or np.random.default_rng()
        self.D = num_features
        self.W = rng.standard_normal((num_features, in_dim)) / lengthscale
        self.b = rng.uniform(0, 2 * math.pi, size=num_features)
        self.scale = math.sqrt(2.0 / num_features)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        return self.scale * np.cos(x @ self.W.T + self.b)


class BayesianLinearRFF:
    def __init__(self, num_features, prior_var=1.0, noise_var=0.25,
                 noise_floor=0.05, forgetting=1.0):
        self.D = num_features
        self.noise_var = noise_var
        self.noise_floor = noise_floor
        self.forgetting = forgetting
        self.m = np.zeros(self.D)
        self.S = np.eye(self.D) * prior_var

    def _noise(self):
        return max(self.noise_var, self.noise_floor ** 2)

    def update(self, phi, y):
        if self.forgetting < 1.0:
            self.S /= self.forgetting
        nv = self._noise()
        Sph = self.S @ phi
        denom = nv + phi @ Sph
        pred = phi @ self.m
        K = Sph / denom
        self.m += K * (y - pred)
        self.S -= np.outer(Sph, Sph) / denom

    def predict(self, Phi):
        mu = Phi @ self.m
        SPhiT = self.S @ Phi.T
        var = np.sum(Phi * SPhiT.T, axis=1) + self._noise()
        std = np.sqrt(np.maximum(var, 1e-12))
        return mu, std


class GradientFieldRFFBLR:
    """Two independent BLRs modelling ∂r/∂x and ∂r/∂y over an HxW grid."""

    def __init__(self, H, W, num_features=256, lengthscale=0.5,
                 prior_std=1.0, noise_std=0.5, noise_floor=0.1,
                 forgetting=1.0, seed=42):
        self.H, self.W = H, W
        rng = np.random.default_rng(seed)

        xs = np.linspace(-1.0, 1.0, W)
        ys = np.linspace(-1.0, 1.0, H)
        Y, X = np.meshgrid(ys, xs, indexing="ij")
        self.coords = np.stack([X.ravel(), Y.ravel()], axis=-1)

        self.rff = RFFFeatureMap(2, num_features, lengthscale, rng)
        self.Phi_grid = self.rff(self.coords)

        noise_var = noise_std ** 2
        prior_var = max(prior_std ** 2 - noise_var, 1e-6)
        self.blr_x = BayesianLinearRFF(num_features, prior_var, noise_var,
                                       noise_floor, forgetting)
        self.blr_y = BayesianLinearRFF(num_features, prior_var, noise_var,
                                       noise_floor, forgetting)

        self.mu_x = np.zeros(H * W)
        self.sig_x = np.full(H * W, prior_std)
        self.mu_y = np.zeros(H * W)
        self.sig_y = np.full(H * W, prior_std)

    def add_sample(self, row, col, gx_obs, gy_obs):
        phi = self.Phi_grid[row * self.W + col]
        self.blr_x.update(phi, gx_obs)
        self.blr_y.update(phi, gy_obs)

    def update_predictions(self):
        self.mu_x, self.sig_x = self.blr_x.predict(self.Phi_grid)
        self.mu_y, self.sig_y = self.blr_y.predict(self.Phi_grid)

    def uncertainty_grid(self):
        return np.sqrt(self.sig_x ** 2 + self.sig_y ** 2).reshape(self.H, self.W)

    def gradient_grids(self):
        return (self.mu_x.reshape(self.H, self.W),
                self.mu_y.reshape(self.H, self.W))

    def sigma_grids(self):
        return (self.sig_x.reshape(self.H, self.W),
                self.sig_y.reshape(self.H, self.W))


# ------------------------------------------------------------------ #
#  UCB action selection (4-connected)
# ------------------------------------------------------------------ #

ACTION_DROW = [-1, 1, 0, 0]
ACTION_DCOL = [0, 0, -1, 1]
INVALID = -1e9


def ucb_select(dx_mu, dx_sig, dy_mu, dy_sig, row, col, alpha=1.0, beta=1.0):
    H, W = dx_mu.shape
    ucb = np.full(4, INVALID)
    if row > 0:
        ucb[0] = alpha * (-dy_mu[row - 1, col]) + beta * dy_sig[row - 1, col]
    if row < H - 1:
        ucb[1] = alpha * dy_mu[row, col] + beta * dy_sig[row, col]
    if col > 0:
        ucb[2] = alpha * (-dx_mu[row, col - 1]) + beta * dx_sig[row, col - 1]
    if col < W - 1:
        ucb[3] = alpha * dx_mu[row, col] + beta * dx_sig[row, col]
    max_val = np.max(ucb)
    candidates = np.flatnonzero(ucb == max_val)
    return int(random.choice(candidates)), ucb


# ------------------------------------------------------------------ #
#  Coordinate helpers
# ------------------------------------------------------------------ #

def world_to_grid(wx, wy, ox, oy, res, H, W):
    col = int(math.floor((wx - ox) / res))
    row = int(math.floor((wy - oy) / res))
    if 0 <= row < H and 0 <= col < W:
        return row, col
    return None


def grid_to_world(row, col, ox, oy, res):
    return ox + (col + 0.5) * res, oy + (row + 0.5) * res


# ------------------------------------------------------------------ #
#  Node
# ------------------------------------------------------------------ #

class BeliefNode(Node):
    def __init__(self):
        super().__init__("belief_node")

        self.declare_parameter("robot_id", 0)
        self.declare_parameter("grid_resolution", 0.2)
        self.declare_parameter("grid_origin_x", -1.5)
        self.declare_parameter("grid_origin_y", -1.5)
        self.declare_parameter("grid_width", 15)
        self.declare_parameter("grid_height", 15)
        self.declare_parameter("displacement_threshold", 0.25)
        self.declare_parameter("num_features", 256)
        self.declare_parameter("lengthscale", 0.6)
        self.declare_parameter("prior_std", 1.0)
        self.declare_parameter("noise_std", 0.5)
        self.declare_parameter("forgetting", 0.999)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("field_frame", "field")
        # Optional UCB-driven waypoint publishing.
        self.declare_parameter("publish_waypoints", False)
        self.declare_parameter("ucb_alpha", 1.0)
        self.declare_parameter("ucb_beta", 10.0)
        self.declare_parameter("waypoint_tolerance", 0.15)
        self.declare_parameter("gradient_subsample", 1)
        self.declare_parameter("random_seed", 42)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.res = float(self.get_parameter("grid_resolution").value)
        self.ox = float(self.get_parameter("grid_origin_x").value)
        self.oy = float(self.get_parameter("grid_origin_y").value)
        self.W = int(self.get_parameter("grid_width").value)
        self.H = int(self.get_parameter("grid_height").value)
        self.disp_thresh = float(self.get_parameter("displacement_threshold").value)
        self.field_frame = str(self.get_parameter("field_frame").value)
        self.publish_waypoints = bool(self.get_parameter("publish_waypoints").value)
        self.ucb_alpha = float(self.get_parameter("ucb_alpha").value)
        self.ucb_beta = float(self.get_parameter("ucb_beta").value)
        self.waypoint_tol = float(self.get_parameter("waypoint_tolerance").value)
        self.gradient_subsample = max(
            1, int(self.get_parameter("gradient_subsample").value)
        )

        self.model = GradientFieldRFFBLR(
            H=self.H, W=self.W,
            num_features=int(self.get_parameter("num_features").value),
            lengthscale=float(self.get_parameter("lengthscale").value),
            prior_std=float(self.get_parameter("prior_std").value),
            noise_std=float(self.get_parameter("noise_std").value),
            noise_floor=0.1,
            forgetting=float(self.get_parameter("forgetting").value),
            seed=int(self.get_parameter("random_seed").value),
        )
        self.model.update_predictions()

        self.pos: np.ndarray | None = None
        self.prev_pos: np.ndarray | None = None
        self.current_reward: float | None = None
        self.prev_reward: float | None = None

        self.waypoint_cell: tuple[int, int] | None = None

        self.create_subscription(
            PoseStamped, f"/field/robot_{self.robot_id}/pose", self._on_pose, 10
        )
        self.create_subscription(Float32, "/global_reward", self._on_reward, 10)

        self.unc_pub = self.create_publisher(
            OccupancyGrid, f"/robot_{self.robot_id}/belief/uncertainty", 1
        )
        self.mag_pub = self.create_publisher(
            OccupancyGrid, f"/robot_{self.robot_id}/belief/gradient_mag", 1
        )
        self.grad_pub = self.create_publisher(
            MarkerArray, f"/robot_{self.robot_id}/belief/gradients", 10
        )
        self.waypoint_pub = self.create_publisher(
            PointStamped, f"/robot_{self.robot_id}/waypoint", 10
        )

        self.create_timer(0.1, self._belief_tick)
        pub_period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(pub_period, self._publish_viz)
        if self.publish_waypoints:
            self.create_timer(0.2, self._waypoint_tick)

        self.get_logger().info(
            f"belief_node[robot_{self.robot_id}] — grid {self.H}×{self.W} "
            f"@ {self.res}m, origin ({self.ox},{self.oy}), "
            f"waypoints={'on' if self.publish_waypoints else 'off'}"
        )

    # ------------------------------------------------------------------ #
    #  Inputs
    # ------------------------------------------------------------------ #
    def _on_pose(self, msg: PoseStamped):
        self.pos = np.array([msg.pose.position.x, msg.pose.position.y])

    def _on_reward(self, msg: Float32):
        self.current_reward = float(msg.data)

    # ------------------------------------------------------------------ #
    #  Belief update
    # ------------------------------------------------------------------ #
    def _belief_tick(self):
        if self.pos is None or self.current_reward is None:
            return

        if self.prev_pos is None:
            self.prev_pos = self.pos.copy()
            self.prev_reward = self.current_reward
            return

        disp = self.pos - self.prev_pos
        dist = float(np.linalg.norm(disp))
        if dist < self.disp_thresh:
            return

        reward_delta = self.current_reward - self.prev_reward
        # Gradient observation: Δr along unit displacement, resolved onto x and y.
        inv_dist = 1.0 / dist
        gx_obs = reward_delta * disp[0] * inv_dist
        gy_obs = reward_delta * disp[1] * inv_dist

        cell = world_to_grid(
            self.prev_pos[0], self.prev_pos[1],
            self.ox, self.oy, self.res, self.H, self.W
        )
        if cell is not None:
            row, col = cell
            self.model.add_sample(row, col, gx_obs, gy_obs)
            self.model.update_predictions()

        self.prev_pos = self.pos.copy()
        self.prev_reward = self.current_reward

    # ------------------------------------------------------------------ #
    #  Waypoint selection (optional)
    # ------------------------------------------------------------------ #
    def _waypoint_tick(self):
        if self.pos is None:
            return

        cell = world_to_grid(
            self.pos[0], self.pos[1], self.ox, self.oy, self.res, self.H, self.W
        )
        if cell is None:
            return
        row, col = cell

        # Replan only if we've reached (or have no) target cell.
        if self.waypoint_cell is not None:
            tx, ty = grid_to_world(*self.waypoint_cell, self.ox, self.oy, self.res)
            if math.hypot(tx - self.pos[0], ty - self.pos[1]) > self.waypoint_tol:
                return  # still pursuing current target

        dx_mu, dy_mu = self.model.gradient_grids()
        dx_sig, dy_sig = self.model.sigma_grids()
        action, _ = ucb_select(
            dx_mu, dx_sig, dy_mu, dy_sig, row, col,
            alpha=self.ucb_alpha, beta=self.ucb_beta,
        )
        new_row = max(0, min(self.H - 1, row + ACTION_DROW[action]))
        new_col = max(0, min(self.W - 1, col + ACTION_DCOL[action]))
        self.waypoint_cell = (new_row, new_col)

        wx, wy = grid_to_world(new_row, new_col, self.ox, self.oy, self.res)
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.field_frame
        msg.point.x = wx
        msg.point.y = wy
        self.waypoint_pub.publish(msg)

    # ------------------------------------------------------------------ #
    #  Visualisation
    # ------------------------------------------------------------------ #
    def _publish_viz(self):
        stamp = self.get_clock().now().to_msg()
        self._publish_occupancy(
            self.unc_pub, stamp, self.model.uncertainty_grid()
        )

        mu_x, mu_y = self.model.gradient_grids()
        self._publish_occupancy(
            self.mag_pub, stamp, np.hypot(mu_x, mu_y)
        )

        self._publish_gradient_arrows(stamp)

    def _publish_occupancy(self, pub, stamp, grid: np.ndarray):
        og = OccupancyGrid()
        og.header.stamp = stamp
        og.header.frame_id = self.field_frame
        og.info.resolution = float(self.res)
        og.info.width = int(self.W)
        og.info.height = int(self.H)
        og.info.origin.position.x = float(self.ox)
        og.info.origin.position.y = float(self.oy)
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0

        gmin = float(grid.min())
        gmax = float(grid.max())
        if gmax - gmin > 1e-9:
            norm = (grid - gmin) / (gmax - gmin)
        else:
            norm = np.zeros_like(grid)
        # OccupancyGrid is row-major, row 0 at origin (bottom-left) — matches
        # our (row=0 → y=origin_y) convention exactly.
        data = (norm * 100.0).astype(np.int8).flatten().tolist()
        og.data = data
        pub.publish(og)

    def _publish_gradient_arrows(self, stamp):
        mu_x, mu_y = self.model.gradient_grids()
        unc = self.model.uncertainty_grid()
        unc_min, unc_max = float(unc.min()), float(unc.max())
        if unc_max - unc_min > 1e-9:
            unc_norm = (unc - unc_min) / (unc_max - unc_min)
        else:
            unc_norm = np.zeros_like(unc)

        step = max(1, self.gradient_subsample)
        arrow_scale = self.res * 0.8
        markers = MarkerArray()
        mid = 0
        for r in range(0, self.H, step):
            for c in range(0, self.W, step):
                gx = float(mu_x[r, c])
                gy = float(mu_y[r, c])
                mag = math.hypot(gx, gy)
                if mag < 1e-6:
                    continue
                wx, wy = grid_to_world(r, c, self.ox, self.oy, self.res)
                ux, uy = gx / mag * arrow_scale, gy / mag * arrow_scale

                m = Marker()
                m.header.frame_id = self.field_frame
                m.header.stamp = stamp
                m.ns = f"robot_{self.robot_id}_grad"
                m.id = mid
                mid += 1
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.points = [
                    Point(x=wx, y=wy, z=0.08),
                    Point(x=wx + ux, y=wy + uy, z=0.08),
                ]
                m.scale = Vector3(x=0.02, y=0.05, z=0.0)
                u = float(unc_norm[r, c])
                m.color = ColorRGBA(r=u, g=1.0 - u, b=0.0, a=0.9)
                m.lifetime.sec = 2
                markers.markers.append(m)

        self.grad_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = BeliefNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
