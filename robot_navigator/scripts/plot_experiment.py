#!/usr/bin/env python3
"""
Offline plotter for trajectory_logger output.

Expects a directory containing one or more `{scenario}.csv` files and a
matching `{scenario}.json` sidecar (target positions + robot IDs) per run.

Per-scenario plots:
  - {scen}_trajectories.png       : x/y paths + target markers
  - {scen}_reward.png             : raw global reward vs time (baseline view)
  - {scen}_formation_error.png    : −R(t) vs time — distance-to-targets the
                                    optimization is actually minimising
  - {scen}_cumulative_regret.png  : ∫ −R(τ) dτ over the run — area under
                                    the formation-error curve
  - {scen}_belief_uncertainty.png : per-robot mean posterior σ over time —
                                    "agent uncertainty in own belief" curve
  - {scen}_pose_latency.png       : per-robot pose-pipeline latency time
                                    series + distribution histogram
                                    (camera/AprilTag pipeline robustness)
  - {scen}_sample_weight.png      : per-robot soft-hold importance weight
                                    over time (only if column populated)

Cross-run comparison overlays (require ≥ 2 runs in the directory):
  - reward_comparison.png
  - formation_error_comparison.png
  - cumulative_regret_comparison.png
  - belief_uncertainty_comparison.png

New columns required for the post-trajectory plots are written by
trajectory_logger.py automatically. Older CSVs without those columns
still get the trajectory + reward + formation-error + cumulative-regret
plots — anything that needs missing columns is silently skipped.

Output layout:
  By default plots land in `{log_dir}/plots/`. Override with `--out`.

Usage:
    python3 scripts/plot_experiment.py experiment_logs/baseline_20260424_120000/
    python3 scripts/plot_experiment.py experiment_logs/run/ --out plots/
"""

import argparse
import json
import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ------------------------------------------------------------------ #
#  Loading
# ------------------------------------------------------------------ #

def _load_runs(log_dir: pathlib.Path) -> dict[str, tuple[pd.DataFrame, dict]]:
    runs: dict[str, tuple[pd.DataFrame, dict]] = {}
    for csv_path in sorted(log_dir.glob("*.csv")):
        scen = csv_path.stem
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print(f"skipping unreadable: {csv_path} ({e})", file=sys.stderr)
            continue
        if df.empty:
            print(f"skipping empty: {csv_path}", file=sys.stderr)
            continue

        sidecar = log_dir / f"{scen}.json"
        meta = {}
        if sidecar.exists():
            try:
                meta = json.loads(sidecar.read_text())
            except Exception as e:
                print(f"sidecar parse failed: {sidecar} ({e})", file=sys.stderr)
        runs[scen] = (df, meta)
    return runs


def _robot_ids_from(df: pd.DataFrame, meta: dict) -> list[int]:
    if meta.get("robot_ids"):
        return [int(r) for r in meta["robot_ids"]]
    ids: list[int] = []
    for col in df.columns:
        if col.startswith("robot_") and col.endswith("_x"):
            try:
                ids.append(int(col[len("robot_"):-len("_x")]))
            except ValueError:
                pass
    return sorted(set(ids))


def _t_and(col: pd.DataFrame, name: str):
    """Return (t, series) with NaNs masked out, or (None, None) if absent/empty."""
    if name not in col.columns:
        return None, None
    t = pd.to_numeric(col["t"], errors="coerce").to_numpy()
    s = pd.to_numeric(col[name], errors="coerce").to_numpy()
    mask = ~(np.isnan(t) | np.isnan(s))
    if not mask.any():
        return None, None
    return t[mask], s[mask]


_DEFAULT_FIELD_BOUNDS = {"x_min": 0.0, "x_max": 3.0, "y_min": 0.0, "y_max": 3.0}


# ------------------------------------------------------------------ #
#  Per-scenario plots
# ------------------------------------------------------------------ #

def _plot_trajectories(scen: str, df: pd.DataFrame, meta: dict,
                       out: pathlib.Path):
    ids = _robot_ids_from(df, meta)
    targets = meta.get("targets", [])
    bounds = meta.get("field_bounds", _DEFAULT_FIELD_BOUNDS)
    fig, ax = plt.subplots(figsize=(6.5, 5.0))
    for rid in ids:
        xs = df.get(f"robot_{rid}_x")
        ys = df.get(f"robot_{rid}_y")
        if xs is None or ys is None:
            continue
        xs = pd.to_numeric(xs, errors="coerce").to_numpy()
        ys = pd.to_numeric(ys, errors="coerce").to_numpy()
        mask = ~(np.isnan(xs) | np.isnan(ys))
        if not mask.any():
            continue
        xs, ys = xs[mask], ys[mask]
        ax.plot(xs, ys, linewidth=1.3, label=f"robot_{rid}")
        ax.scatter([xs[0]], [ys[0]], marker="o", s=40,
                   edgecolors="black", facecolors="white", zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], marker="s", s=40,
                   edgecolors="black", facecolors="none", zorder=5)
    if targets:
        tx = [t[0] for t in targets]
        ty = [t[1] for t in targets]
        ax.scatter(tx, ty, marker="x", c="k", s=80, linewidths=2,
                   label="targets", zorder=4)
    ax.set_title(f"Trajectories — {scen}")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_xlim(bounds["x_min"], bounds["x_max"])
    ax.set_ylim(bounds["y_min"], bounds["y_max"])
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_reward(scen: str, df: pd.DataFrame, out: pathlib.Path):
    t, r = _t_and(df, "reward")
    if t is None:
        return
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    ax.plot(t, r, linewidth=1.2, color="tab:blue")
    ax.set_title(f"Global reward — {scen}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("reward")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_formation_error(scen: str, df: pd.DataFrame, out: pathlib.Path):
    """Formation error = −R(t). The quantity the optimisation minimises."""
    t, r = _t_and(df, "reward")
    if t is None:
        return
    err = -r
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    ax.plot(t, err, linewidth=1.2, color="tab:red")
    ax.set_title(f"Formation error −R(t) — {scen}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("formation error  (sum of distances, m)")
    ax.grid(alpha=0.3)
    ax.axhline(0.0, color="k", linewidth=0.6, alpha=0.4)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_cumulative_regret(scen: str, df: pd.DataFrame, out: pathlib.Path):
    """Cumulative regret ≈ ∫ (R* − R(τ)) dτ with R* = 0.

    Trapezoidal integration handles uneven sample spacing (sample_hz drift).
    """
    t, r = _t_and(df, "reward")
    if t is None or len(t) < 2:
        return
    err = -r
    # cumulative trapezoid: midpoint rule on each interval.
    dt = np.diff(t)
    mid = 0.5 * (err[1:] + err[:-1])
    cum = np.concatenate([[0.0], np.cumsum(mid * dt)])
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    ax.plot(t, cum, linewidth=1.4, color="tab:purple")
    ax.set_title(f"Cumulative regret ∫−R(τ)dτ — {scen}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("cumulative regret  (m·s)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_belief_uncertainty(scen: str, df: pd.DataFrame, ids: list[int],
                             out: pathlib.Path):
    """Per-robot mean posterior σ over time — agent uncertainty in own belief."""
    series = []
    for rid in ids:
        t, u = _t_and(df, f"robot_{rid}_belief_unc")
        if t is None:
            continue
        series.append((rid, t, u))
    if not series:
        return
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    for rid, t, u in series:
        ax.plot(t, u, linewidth=1.2, label=f"robot_{rid}")
    ax.set_title(f"Posterior gradient uncertainty — {scen}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("mean σ  (gradient units)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_pose_latency(scen: str, df: pd.DataFrame, ids: list[int],
                       out: pathlib.Path):
    """Pose-pipeline latency per robot — line + histogram.

    Captures (logger receive time − message header.stamp) per pose. Spikes
    correspond to AprilTag-detector / field-localizer pipeline stalls;
    persistent offsets reveal calibration / clock issues.
    """
    series = []
    for rid in ids:
        t, lat = _t_and(df, f"robot_{rid}_pose_age_s")
        if t is None:
            continue
        # Convert s → ms for readability.
        series.append((rid, t, lat * 1000.0))
    if not series:
        return

    fig, (ax_t, ax_h) = plt.subplots(
        1, 2, figsize=(10.0, 3.5),
        gridspec_kw={"width_ratios": [3, 1]},
    )
    for rid, t, lat in series:
        ax_t.plot(t, lat, linewidth=1.0, label=f"robot_{rid}")
    ax_t.set_title(f"Pose-pipeline latency — {scen}")
    ax_t.set_xlabel("t (s)")
    ax_t.set_ylabel("latency (ms)")
    ax_t.grid(alpha=0.3)
    ax_t.legend(fontsize=8, loc="best")

    # Histogram of all robots' latency samples for the run.
    all_lats = np.concatenate([lat for _, _, lat in series])
    all_lats = all_lats[np.isfinite(all_lats)]
    if all_lats.size:
        ax_h.hist(all_lats, bins=40, color="tab:gray", edgecolor="k", linewidth=0.4)
        med = float(np.median(all_lats))
        p95 = float(np.percentile(all_lats, 95))
        ax_h.axvline(med, color="tab:blue", linewidth=1.0,
                     label=f"median {med:.0f} ms")
        ax_h.axvline(p95, color="tab:red", linewidth=1.0,
                     label=f"p95 {p95:.0f} ms")
        ax_h.legend(fontsize=8, loc="best")
    ax_h.set_title("distribution")
    ax_h.set_xlabel("latency (ms)")
    ax_h.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_sample_weight(scen: str, df: pd.DataFrame, ids: list[int],
                        out: pathlib.Path):
    """Per-robot soft-hold importance weight over time. Skipped if the
    column is empty (e.g., baseline run with the soft-hold publisher
    disabled in an older binary)."""
    series = []
    for rid in ids:
        t, w = _t_and(df, f"robot_{rid}_sample_weight")
        if t is None:
            continue
        series.append((rid, t, w))
    if not series:
        return
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    for rid, t, w in series:
        ax.plot(t, w, linewidth=1.0, label=f"robot_{rid}", marker=".",
                markersize=3, linestyle="-")
    ax.set_title(f"Soft-hold sample weight — {scen}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("weight  (1 = single-mover credit, lower = down-weighted)")
    ax.set_ylim(0.0, 1.05)
    ax.axhline(1.0, color="k", linewidth=0.6, alpha=0.4)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


# ------------------------------------------------------------------ #
#  Cross-run overlays
# ------------------------------------------------------------------ #

def _plot_reward_comparison(runs: dict, out: pathlib.Path):
    if len(runs) < 2:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.0))
    plotted = 0
    for scen, (df, _) in sorted(runs.items()):
        t, r = _t_and(df, "reward")
        if t is None:
            continue
        ax.plot(t, r, linewidth=1.1, label=scen)
        plotted += 1
    if plotted < 2:
        plt.close(fig)
        return
    ax.set_title("Global reward — all runs")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("reward")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_formation_error_comparison(runs: dict, out: pathlib.Path):
    if len(runs) < 2:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.0))
    plotted = 0
    for scen, (df, _) in sorted(runs.items()):
        t, r = _t_and(df, "reward")
        if t is None:
            continue
        ax.plot(t, -r, linewidth=1.1, label=scen)
        plotted += 1
    if plotted < 2:
        plt.close(fig)
        return
    ax.set_title("Formation error −R(t) — all runs")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("formation error (m)")
    ax.grid(alpha=0.3)
    ax.axhline(0.0, color="k", linewidth=0.6, alpha=0.4)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_cumulative_regret_comparison(runs: dict, out: pathlib.Path):
    if len(runs) < 2:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.0))
    plotted = 0
    for scen, (df, _) in sorted(runs.items()):
        t, r = _t_and(df, "reward")
        if t is None or len(t) < 2:
            continue
        err = -r
        dt = np.diff(t)
        mid = 0.5 * (err[1:] + err[:-1])
        cum = np.concatenate([[0.0], np.cumsum(mid * dt)])
        ax.plot(t, cum, linewidth=1.2, label=scen)
        plotted += 1
    if plotted < 2:
        plt.close(fig)
        return
    ax.set_title("Cumulative regret ∫−R(τ)dτ — all runs")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("cumulative regret (m·s)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_belief_uncertainty_comparison(runs: dict, out: pathlib.Path):
    """One line per run: mean across robots of the per-tick mean σ."""
    if len(runs) < 2:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.0))
    plotted = 0
    for scen, (df, meta) in sorted(runs.items()):
        ids = _robot_ids_from(df, meta)
        per_robot_t = []
        per_robot_u = []
        for rid in ids:
            t, u = _t_and(df, f"robot_{rid}_belief_unc")
            if t is None:
                continue
            per_robot_t.append(t)
            per_robot_u.append(u)
        if not per_robot_u:
            continue
        # Resample to the shortest series so we can average across robots
        # without interpolating across uneven sample-clock drift.
        n = min(len(u) for u in per_robot_u)
        stacked = np.vstack([u[:n] for u in per_robot_u])
        t_common = per_robot_t[0][:n]
        ax.plot(t_common, stacked.mean(axis=0), linewidth=1.2, label=scen)
        plotted += 1
    if plotted < 2:
        plt.close(fig)
        return
    ax.set_title("Posterior gradient uncertainty (mean across robots) — all runs")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("mean σ")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


# ------------------------------------------------------------------ #
#  Driver
# ------------------------------------------------------------------ #

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", type=pathlib.Path,
                        help="Directory containing {scenario}.csv + {scenario}.json")
    parser.add_argument("--out", type=pathlib.Path, default=None,
                        help="Output directory (default: {log_dir}/plots)")
    args = parser.parse_args()

    if not args.log_dir.is_dir():
        print(f"not a directory: {args.log_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.out or (args.log_dir / "plots")
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = _load_runs(args.log_dir)
    if not runs:
        print(f"no CSVs found in {args.log_dir}", file=sys.stderr)
        sys.exit(1)

    for scen, (df, meta) in runs.items():
        ids = _robot_ids_from(df, meta)
        _plot_trajectories(scen, df, meta, out_dir / f"{scen}_trajectories.png")
        _plot_reward(scen, df, out_dir / f"{scen}_reward.png")
        _plot_formation_error(scen, df, out_dir / f"{scen}_formation_error.png")
        _plot_cumulative_regret(scen, df, out_dir / f"{scen}_cumulative_regret.png")
        _plot_belief_uncertainty(scen, df, ids,
                                 out_dir / f"{scen}_belief_uncertainty.png")
        _plot_pose_latency(scen, df, ids,
                           out_dir / f"{scen}_pose_latency.png")
        _plot_sample_weight(scen, df, ids,
                            out_dir / f"{scen}_sample_weight.png")

    _plot_reward_comparison(runs, out_dir / "reward_comparison.png")
    _plot_formation_error_comparison(runs, out_dir / "formation_error_comparison.png")
    _plot_cumulative_regret_comparison(runs, out_dir / "cumulative_regret_comparison.png")
    _plot_belief_uncertainty_comparison(runs, out_dir / "belief_uncertainty_comparison.png")

    print("Wrote:")
    for f in sorted(out_dir.glob("*.png")):
        print(" ", f)


if __name__ == "__main__":
    main()
