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

Update rule: cache (position, reward). When the robot crosses into a new
grid cell (and has moved at least `min_displacement` metres to filter
pose noise), decompose Δreward along the displacement into a 2-D gradient
observation at the *source* grid cell and feed it into two independent
Bayesian linear regressions (x- and y-components) over Random Fourier
Features.

If `publish_waypoints` is true, the node also runs UCB on its current
gradient belief and publishes a waypoint for the `robot_navigator` to
drive to. Otherwise it's purely a belief-visualisation node.
"""

import math
import random
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, PoseStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Bool, ColorRGBA, Float32
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


def ucb_select(dx_mu, dx_sig, dy_mu, dy_sig, row, col, alpha=1.0, beta=1.0,
               penalty=None, forbidden=None):
    """
    4-connected UCB action selection.

    `penalty`, if given, is an HxW array of non-negative costs subtracted
    from each action's score at the *destination* cell — the soft layer.

    `forbidden`, if given, is an HxW bool array: destinations flagged True
    are hard-rejected (score stays INVALID, same treatment as off-grid).
    Use this for peer-mask collision avoidance that must never be
    overridden by a strong gradient.
    """
    H, W = dx_mu.shape
    ucb = np.full(4, INVALID)
    if row > 0 and (forbidden is None or not forbidden[row - 1, col]):
        s = alpha * (-dy_mu[row - 1, col]) + beta * dy_sig[row - 1, col]
        if penalty is not None:
            s -= penalty[row - 1, col]
        ucb[0] = s
    if row < H - 1 and (forbidden is None or not forbidden[row + 1, col]):
        s = alpha * dy_mu[row, col] + beta * dy_sig[row, col]
        if penalty is not None:
            s -= penalty[row + 1, col]
        ucb[1] = s
    if col > 0 and (forbidden is None or not forbidden[row, col - 1]):
        s = alpha * (-dx_mu[row, col - 1]) + beta * dx_sig[row, col - 1]
        if penalty is not None:
            s -= penalty[row, col - 1]
        ucb[2] = s
    if col < W - 1 and (forbidden is None or not forbidden[row, col + 1]):
        s = alpha * dx_mu[row, col] + beta * dx_sig[row, col]
        if penalty is not None:
            s -= penalty[row, col + 1]
        ucb[3] = s
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
        self.declare_parameter("grid_origin_x", 0.0)
        self.declare_parameter("grid_origin_y", 0.0)
        self.declare_parameter("grid_width", 15)
        self.declare_parameter("grid_height", 15)
        # Minimum displacement (m) to accept a sample, on top of the
        # cell-change requirement. Pure pose-noise jitter should stay below
        # this. Keep it well under grid_resolution.
        self.declare_parameter("min_displacement", 0.05)
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
        # Peer-aware collision avoidance at the planner layer: each peer's
        # position contributes a Gaussian penalty to candidate cells.
        self.declare_parameter(
            "peer_robot_ids", [],
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("repulsion_weight", 5.0)
        self.declare_parameter("repulsion_radius", 0.4)
        self.declare_parameter("peer_pose_timeout", 1.0)
        # Hard action mask: forbid UCB candidates whose destination lies
        # within this radius (m) of any peer. 0 disables the mask, leaving
        # only the soft Gaussian penalty active.
        self.declare_parameter("peer_mask_radius", 0.0)
        # Runtime kill-switch for the planner-layer repulsion penalty.
        self.declare_parameter("repulsion_enabled", True)
        self.declare_parameter("avoidance_toggle_topic", "/collision_avoidance_enabled")
        # Edge-margin penalty: discourages UCB from picking waypoints near
        # the grid boundary (under-sampled corners otherwise attract the
        # explore bonus). 0 or non-positive values disable.
        self.declare_parameter("edge_margin_cells", 2)
        self.declare_parameter("edge_margin_weight", 5.0)

        self.robot_id = int(self.get_parameter("robot_id").value)
        self.res = float(self.get_parameter("grid_resolution").value)
        self.ox = float(self.get_parameter("grid_origin_x").value)
        self.oy = float(self.get_parameter("grid_origin_y").value)
        self.W = int(self.get_parameter("grid_width").value)
        self.H = int(self.get_parameter("grid_height").value)
        self.min_disp = float(self.get_parameter("min_displacement").value)
        self.field_frame = str(self.get_parameter("field_frame").value)
        self.publish_waypoints = bool(self.get_parameter("publish_waypoints").value)
        self.ucb_alpha = float(self.get_parameter("ucb_alpha").value)
        self.ucb_beta = float(self.get_parameter("ucb_beta").value)
        self.waypoint_tol = float(self.get_parameter("waypoint_tolerance").value)
        self.gradient_subsample = max(
            1, int(self.get_parameter("gradient_subsample").value)
        )
        peer_ids_raw = self.get_parameter("peer_robot_ids").value or []
        self.peer_ids = [int(v) for v in peer_ids_raw if int(v) != self.robot_id]
        self.rep_weight = float(self.get_parameter("repulsion_weight").value)
        self.rep_radius = float(self.get_parameter("repulsion_radius").value)
        self.peer_timeout = float(self.get_parameter("peer_pose_timeout").value)
        self.mask_radius = float(self.get_parameter("peer_mask_radius").value)
        self.rep_enabled = bool(self.get_parameter("repulsion_enabled").value)
        avoid_toggle_topic = str(self.get_parameter("avoidance_toggle_topic").value)
        self.edge_margin_cells = int(self.get_parameter("edge_margin_cells").value)
        self.edge_margin_weight = float(self.get_parameter("edge_margin_weight").value)

        # Precomputed world-coord mesh for fast penalty evaluation.
        xs = self.ox + (np.arange(self.W) + 0.5) * self.res
        ys = self.oy + (np.arange(self.H) + 0.5) * self.res
        self._grid_X, self._grid_Y = np.meshgrid(xs, ys)

        # Static edge-margin penalty: linear falloff from the boundary.
        # Cells ≥ edge_margin_cells from every edge get 0.
        self._edge_penalty = self._compute_edge_penalty()

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
        self.prev_cell: tuple[int, int] | None = None
        self.current_reward: float | None = None
        self.prev_reward: float | None = None

        self.waypoint_cell: tuple[int, int] | None = None

        # Peer position cache: {robot_id: (pos, stamp_monotonic)}.
        self.peer_positions: dict[int, tuple[np.ndarray, float]] = {}
        # Peer waypoint cache — used to avoid argmax'ing onto a cell a
        # peer has already committed to. Only populated for lower-ID
        # peers (priority rule: higher-ID robots yield, lower-IDs don't).
        self.peer_waypoints: dict[int, tuple[np.ndarray, float]] = {}

        self.create_subscription(
            PoseStamped, f"/field/robot_{self.robot_id}/pose", self._on_pose, 10
        )
        self.create_subscription(Float32, "/global_reward", self._on_reward, 10)

        for pid in self.peer_ids:
            self.create_subscription(
                PoseStamped,
                f"/field/robot_{pid}/pose",
                lambda msg, i=pid: self._on_peer_pose(i, msg),
                10,
            )
            # Priority tiebreak: only subscribe to waypoints of peers
            # whose IDs are lower than our own. Higher-ID peers will see
            # our waypoint and yield to us; we never yield to them.
            if pid < self.robot_id:
                self.create_subscription(
                    PointStamped,
                    f"/robot_{pid}/waypoint",
                    lambda msg, i=pid: self._on_peer_waypoint(i, msg),
                    10,
                )

        self.create_subscription(
            Bool, avoid_toggle_topic, self._on_avoidance_toggle, 10
        )

        self.unc_pub = self.create_publisher(
            OccupancyGrid, f"/robot_{self.robot_id}/belief/uncertainty", 1
        )
        self.mag_pub = self.create_publisher(
            OccupancyGrid, f"/robot_{self.robot_id}/belief/gradient_mag", 1
        )
        self.grad_pub = self.create_publisher(
            MarkerArray, f"/robot_{self.robot_id}/belief/gradients", 10
        )
        self.rep_pub = self.create_publisher(
            OccupancyGrid, f"/robot_{self.robot_id}/belief/repulsion", 1
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

    def _on_peer_pose(self, pid: int, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y])
        self.peer_positions[pid] = (pos, time.monotonic())

    def _on_peer_waypoint(self, pid: int, msg: PointStamped):
        wp = np.array([msg.point.x, msg.point.y])
        self.peer_waypoints[pid] = (wp, time.monotonic())

    def _on_avoidance_toggle(self, msg: Bool):
        was = self.rep_enabled
        self.rep_enabled = bool(msg.data)
        if was != self.rep_enabled:
            state = "ENABLED" if self.rep_enabled else "DISABLED"
            self.get_logger().warn(f"Planner-layer repulsion {state}.")

    def _fresh_peer_positions(self) -> list[np.ndarray]:
        now = time.monotonic()
        return [p for p, t in self.peer_positions.values()
                if now - t <= self.peer_timeout]

    def _fresh_peer_waypoints(self) -> list[np.ndarray]:
        """Lower-ID peers' most recent waypoints within peer_timeout.

        Same freshness window as positions — a stale waypoint expires
        so we don't keep yielding to a muted peer indefinitely.
        """
        now = time.monotonic()
        return [wp for wp, t in self.peer_waypoints.values()
                if now - t <= self.peer_timeout]

    def _peer_penalty_grid(self) -> np.ndarray | None:
        if not self.rep_enabled:
            return None
        peers = self._fresh_peer_positions()
        if not peers or self.rep_weight <= 0.0 or self.rep_radius <= 0.0:
            return None
        denom = 2.0 * self.rep_radius * self.rep_radius
        penalty = np.zeros((self.H, self.W))
        for p in peers:
            d2 = (self._grid_X - p[0]) ** 2 + (self._grid_Y - p[1]) ** 2
            penalty += self.rep_weight * np.exp(-d2 / denom)
        return penalty

    def _peer_mask_grid(self) -> np.ndarray | None:
        """Bool HxW grid — True at cells that must not be chosen as waypoints.

        Hazard sources (union):
          - every fresh peer's current POSITION (symmetric — physical
            collision is mutual)
          - every fresh LOWER-ID peer's published WAYPOINT (asymmetric —
            higher-ID robots yield to lower-ID claims)

        Returns None when disabled or no cells are flagged.
        """
        if not self.rep_enabled or self.mask_radius <= 0.0:
            return None
        hazards = self._fresh_peer_positions() + self._fresh_peer_waypoints()
        if not hazards:
            return None
        r2 = self.mask_radius * self.mask_radius
        mask = np.zeros((self.H, self.W), dtype=bool)
        for p in hazards:
            d2 = (self._grid_X - p[0]) ** 2 + (self._grid_Y - p[1]) ** 2
            mask |= d2 <= r2
        return mask if mask.any() else None

    def _compute_edge_penalty(self) -> np.ndarray | None:
        if self.edge_margin_cells <= 0 or self.edge_margin_weight <= 0.0:
            return None
        rs = np.arange(self.H)
        cs = np.arange(self.W)
        d_row = np.minimum(rs, self.H - 1 - rs)
        d_col = np.minimum(cs, self.W - 1 - cs)
        d_edge = np.minimum(d_row[:, None], d_col[None, :])
        margin = float(self.edge_margin_cells)
        return self.edge_margin_weight * np.maximum(0.0, margin - d_edge) / margin

    def _combined_penalty(self) -> np.ndarray | None:
        peer = self._peer_penalty_grid()
        edge = self._edge_penalty
        if peer is None and edge is None:
            return None
        if peer is None:
            return edge
        if edge is None:
            return peer
        return peer + edge

    # ------------------------------------------------------------------ #
    #  Belief update
    # ------------------------------------------------------------------ #
    def _belief_tick(self):
        if self.pos is None or self.current_reward is None:
            return

        cur_cell = world_to_grid(
            self.pos[0], self.pos[1],
            self.ox, self.oy, self.res, self.H, self.W
        )
        if cur_cell is None:
            return

        if self.prev_pos is None:
            self.prev_pos = self.pos.copy()
            self.prev_cell = cur_cell
            self.prev_reward = self.current_reward
            return

        # Sample only when the robot enters a new grid cell. Filters cell
        # chatter on cell boundaries via a small metric floor.
        if cur_cell == self.prev_cell:
            return

        disp = self.pos - self.prev_pos
        dist = float(np.linalg.norm(disp))
        if dist < self.min_disp:
            return

        reward_delta = self.current_reward - self.prev_reward
        inv_dist = 1.0 / dist
        gx_obs = reward_delta * disp[0] * inv_dist
        gy_obs = reward_delta * disp[1] * inv_dist

        row, col = self.prev_cell
        self.model.add_sample(row, col, gx_obs, gy_obs)
        self.model.update_predictions()

        self.prev_pos = self.pos.copy()
        self.prev_cell = cur_cell
        self.prev_reward = self.current_reward

    # ------------------------------------------------------------------ #
    #  Waypoint selection (optional)
    # ------------------------------------------------------------------ #
    def _waypoint_tick(self):
        if self.pos is None:
            self.get_logger().warn(
                f"No pose on /field/robot_{self.robot_id}/pose yet — "
                f"waypoint tick idle.",
                throttle_duration_sec=5.0,
            )
            return

        cell = world_to_grid(
            self.pos[0], self.pos[1], self.ox, self.oy, self.res, self.H, self.W
        )
        if cell is None:
            self.get_logger().warn(
                f"Robot pose ({self.pos[0]:.2f}, {self.pos[1]:.2f}) outside "
                f"belief grid [{self.ox:.2f}, {self.ox + self.W * self.res:.2f}] × "
                f"[{self.oy:.2f}, {self.oy + self.H * self.res:.2f}] — "
                f"no waypoint will be published until pose is inside.",
                throttle_duration_sec=5.0,
            )
            return
        row, col = cell

        # Replan only if we've reached (or have no) target cell; otherwise
        # fall through and republish the existing target so downstream tools
        # see a steady waypoint stream.
        need_replan = True
        if self.waypoint_cell is not None:
            tx, ty = grid_to_world(*self.waypoint_cell, self.ox, self.oy, self.res)
            if math.hypot(tx - self.pos[0], ty - self.pos[1]) > self.waypoint_tol:
                need_replan = False

        # Force replan if the committed waypoint cell has become forbidden
        # by a peer hazard. Without this, two robots that independently
        # argmax'd onto the same cell on tick 0 would drive there together
        # (priority mask would only fire at their next legitimate replan,
        # which is too late).
        if not need_replan and self.waypoint_cell is not None:
            forbidden_check = self._peer_mask_grid()
            if forbidden_check is not None and forbidden_check[self.waypoint_cell]:
                need_replan = True
                self.get_logger().info(
                    "Current waypoint now forbidden by peer hazard — replanning.",
                    throttle_duration_sec=2.0,
                )

        if need_replan:
            dx_mu, dy_mu = self.model.gradient_grids()
            dx_sig, dy_sig = self.model.sigma_grids()
            penalty = self._combined_penalty()
            forbidden = self._peer_mask_grid()
            action, ucb_scores = ucb_select(
                dx_mu, dx_sig, dy_mu, dy_sig, row, col,
                alpha=self.ucb_alpha, beta=self.ucb_beta,
                penalty=penalty, forbidden=forbidden,
            )
            if float(np.max(ucb_scores)) <= INVALID:
                # All 4 neighbours are off-grid or mask-forbidden. Holding
                # the current cell preserves the mask's "never enter a
                # forbidden cell" guarantee — the navigator's reactive
                # layer handles any genuine close approach from here.
                self.get_logger().warn(
                    "All neighbours forbidden; holding position.",
                    throttle_duration_sec=2.0,
                )
                self.waypoint_cell = (row, col)
            else:
                new_row = max(0, min(self.H - 1, row + ACTION_DROW[action]))
                new_col = max(0, min(self.W - 1, col + ACTION_DCOL[action]))
                self.waypoint_cell = (new_row, new_col)
                self.get_logger().info(
                    f"New waypoint cell ({new_row}, {new_col}) -> "
                    f"{grid_to_world(new_row, new_col, self.ox, self.oy, self.res)}"
                )

        wx, wy = grid_to_world(*self.waypoint_cell, self.ox, self.oy, self.res)
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

        rep = self._combined_penalty()
        if rep is None:
            rep = np.zeros((self.H, self.W))
        self._publish_occupancy(self.rep_pub, stamp, rep)

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
