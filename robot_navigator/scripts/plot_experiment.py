#!/usr/bin/env python3
"""
Offline plotter for trajectory_logger output.

Expects a directory containing one or more `{scenario}.csv` files and a
matching `{scenario}.json` sidecar (target positions + robot IDs) per run.

Renders, per scenario:
  - {scenario}_trajectories.png : x/y paths for each robot + target markers
  - {scenario}_reward.png       : global reward vs time

Plus a combined overlay across all scenarios found:
  - reward_comparison.png       : reward(t) for every run on one axis

Usage:
    python3 scripts/plot_experiment.py experiment_logs/
    python3 scripts/plot_experiment.py experiment_logs/ --out plots/
"""

import argparse
import json
import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


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
    # Fallback: infer from column names "robot_{id}_x".
    ids: list[int] = []
    for col in df.columns:
        if col.startswith("robot_") and col.endswith("_x"):
            try:
                ids.append(int(col[len("robot_"):-len("_x")]))
            except ValueError:
                pass
    return sorted(set(ids))


def _plot_trajectories(scen: str, df: pd.DataFrame, meta: dict,
                       out: pathlib.Path):
    ids = _robot_ids_from(df, meta)
    targets = meta.get("targets", [])
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
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_reward(scen: str, df: pd.DataFrame, out: pathlib.Path):
    if "reward" not in df.columns:
        return
    t = pd.to_numeric(df["t"], errors="coerce").to_numpy()
    r = pd.to_numeric(df["reward"], errors="coerce").to_numpy()
    mask = ~(np.isnan(t) | np.isnan(r))
    if not mask.any():
        return
    fig, ax = plt.subplots(figsize=(7.0, 3.5))
    ax.plot(t[mask], r[mask], linewidth=1.2, color="tab:blue")
    ax.set_title(f"Global reward — {scen}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("reward")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def _plot_reward_comparison(runs: dict, out: pathlib.Path):
    if len(runs) < 2:
        return
    fig, ax = plt.subplots(figsize=(7.5, 4.0))
    for scen, (df, _) in sorted(runs.items()):
        if "reward" not in df.columns:
            continue
        t = pd.to_numeric(df["t"], errors="coerce").to_numpy()
        r = pd.to_numeric(df["reward"], errors="coerce").to_numpy()
        mask = ~(np.isnan(t) | np.isnan(r))
        if not mask.any():
            continue
        ax.plot(t[mask], r[mask], linewidth=1.1, label=scen)
    ax.set_title("Global reward — all runs")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("reward")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", type=pathlib.Path,
                        help="Directory containing {scenario}.csv + {scenario}.json")
    parser.add_argument("--out", type=pathlib.Path, default=None,
                        help="Output directory (default: same as log_dir)")
    args = parser.parse_args()

    if not args.log_dir.is_dir():
        print(f"not a directory: {args.log_dir}", file=sys.stderr)
        sys.exit(1)
    out_dir = args.out or args.log_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = _load_runs(args.log_dir)
    if not runs:
        print(f"no CSVs found in {args.log_dir}", file=sys.stderr)
        sys.exit(1)

    for scen, (df, meta) in runs.items():
        _plot_trajectories(scen, df, meta, out_dir / f"{scen}_trajectories.png")
        _plot_reward(scen, df, out_dir / f"{scen}_reward.png")
    _plot_reward_comparison(runs, out_dir / "reward_comparison.png")

    print("Wrote:")
    for f in sorted(out_dir.glob("*.png")):
        print(" ", f)


if __name__ == "__main__":
    main()
