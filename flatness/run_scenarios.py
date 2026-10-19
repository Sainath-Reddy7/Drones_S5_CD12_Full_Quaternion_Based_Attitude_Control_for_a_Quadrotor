"""Scenario benchmark for the extension trajectories (E1 + E2).

Runs the SAME frozen double-loop LQR stack (gains untouched - the frozen-
controller discipline) over every trajectory on the paper's ideal Eq. (8)
model, plus the min-snap vs min-jerk A/B on identical waypoints, and prints
one comparison table. Plots land in results/flatness/scenarios/.

Usage:
    python -m flatness.run_scenarios [--cycles 2] [--with-mujoco]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from flatness.polytraj import PolyTrajectory
from flatness.run_paper import run
from flatness.trajectories import (
    CircleTrajectory, FigureEightTrajectory, VerticalOrbitTrajectory,
)

WAYOUT_WPS = [
    [0.0, 0.0, 0.15],    # near-ground start (pad clearance)
    [1.5, 0.0, 1.2],
    [1.5, 2.0, 1.2],
    [0.0, 2.0, 1.2],
    [0.0, 0.0, 0.8],     # return over the pad, descend
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cycles", type=float, default=2.0,
                    help="circuits for omega-based trajectories")
    ap.add_argument("--with-mujoco", action="store_true",
                    help="also run the figure-8 on MuJoCo contact physics")
    args = ap.parse_args()

    out = Path.cwd() / "results" / "flatness" / "scenarios"
    scenarios = [
        ("circle (paper)", CircleTrajectory()),
        ("figure-8", FigureEightTrajectory()),
        ("vertical orbit", VerticalOrbitTrajectory()),
        ("waypoints min-snap", PolyTrajectory.through(
            WAYOUT_WPS, speed=0.35, d=4, name="wp_minsnap")),
        ("waypoints min-jerk", PolyTrajectory.through(
            WAYOUT_WPS, speed=0.35, d=3, name="wp_minjerk")),
    ]

    rows = []
    for label, traj in scenarios:
        s = run(traj=traj, cycles=args.cycles, out_root=out)
        rows.append((label, s))

    if args.with_mujoco:
        from flatness.run_mujoco import run as run_mj
        s = run_mj(traj=FigureEightTrajectory(), out_root=out)
        rows.append(("figure-8 (MuJoCo)", s))

    print(f"\n{'scenario':<22} {'RMS x':>7} {'RMS y':>7} {'RMS z':>7} "
          f"{'RMS 3D':>8} {'peak rotor':>10} {'dur [s]':>8}")
    print("-" * 76)
    for label, s in rows:
        print(f"{label:<22} {s['rms_x_m']*100:6.1f}c {s['rms_y_m']*100:6.1f}c "
              f"{s['rms_z_m']*100:6.1f}c {s['rms_3d_m']*100:7.1f}c "
              f"{s['max_rotor_thrust_n']:9.2f}N {s['duration_s']:8.1f}")

    snap_rows = [s for (l, s) in rows if "snap" in l or "jerk" in l]
    if len(snap_rows) == 2:
        a, b = snap_rows
        print(f"\nA/B (identical waypoints & times): min-snap 3D RMS "
              f"{a['rms_3d_m']*100:.1f} cm vs min-jerk {b['rms_3d_m']*100:.1f} cm "
              f"({(1 - a['rms_3d_m']/b['rms_3d_m'])*100:+.0f}% error)")


if __name__ == "__main__":
    main()
