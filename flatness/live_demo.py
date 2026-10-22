"""Live MuJoCo demo launcher for the paper-2 stack (Choutri & Lagha 2017).

Flies any trajectory of the flatness + double-loop LQR stack in the MuJoCo
viewer (launch_passive) with the marker scene of that trajectory - the same
frozen controllers and scenes as the benchmarks, just interactive.

Usage:
    python -m flatness.live_demo                      # circle (paper study case)
    python -m flatness.live_demo --scenario figure8
    python -m flatness.live_demo --scenario orbit
    python -m flatness.live_demo --scenario waypoints
    python -m flatness.live_demo --scenario noise_circle   # paper noise 0.1
"""
from __future__ import annotations

import argparse
from pathlib import Path

from flatness.polytraj import PolyTrajectory
from flatness.run_mujoco import run
from flatness.trajectories import (
    CircleTrajectory,
    FigureEightTrajectory,
    VerticalOrbitTrajectory,
)

_SCENES = Path(__file__).resolve().parent
WAYOUT_WPS = [
    [0.0, 0.0, 0.15],
    [1.5, 0.0, 1.2],
    [1.5, 2.0, 1.2],
    [0.0, 2.0, 1.2],
    [0.0, 0.0, 0.8],
]

SCENARIOS = {
    "circle": dict(
        traj=lambda: CircleTrajectory(radius=1.0, alt=1.0, speed=0.5),
        xml=_SCENES / "circle_scene.xml",
        label="paper study case - 1 m radius circle at 1 m altitude",
    ),
    "figure8": dict(
        traj=lambda: FigureEightTrajectory(),
        xml=_SCENES / "figure8_scene.xml",
        label="extension - figure-8 (1.5 x 1.0 m, 1 m altitude)",
    ),
    "orbit": dict(
        traj=lambda: VerticalOrbitTrajectory(),
        xml=_SCENES / "circle_scene.xml",
        label="extension - vertical orbit (1 m radius, +/-0.5 m altitude)",
    ),
    "waypoints": dict(
        traj=lambda: PolyTrajectory.through(
            WAYOUT_WPS, speed=0.35, d=4, name="wp_minsnap"),
        xml=_SCENES / "waypoints_scene.xml",
        label="extension - 5-waypoint minimum-snap mission",
    ),
    "noise_circle": dict(
        traj=lambda: CircleTrajectory(radius=1.0, alt=1.0, speed=0.5),
        xml=_SCENES / "circle_scene.xml",
        label="paper study case with full sensor noise (+-0.1)",
        noise=0.1,
    ),
}


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Live MuJoCo demo - paper-2 flatness + double-loop LQR")
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="circle")
    ap.add_argument("--cycles", type=float, default=2.0,
                    help="circuits for the closed trajectories")
    ap.add_argument("--noise", type=float, default=None,
                    help="override sensor noise (default: scenario default)")
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--no-gui", action="store_true",
                    help="headless (smoke test / CI)")
    args = ap.parse_args()

    sc = SCENARIOS[args.scenario]
    noise = sc.get("noise", 0.0) if args.noise is None else args.noise
    print("=" * 62)
    print(f"  Choutri & Lagha 2017 - {args.scenario}")
    print(f"  {sc['label']}")
    print(f"  noise {noise} | controller 200 Hz ZOH | plant MuJoCo 20 kHz")
    print("=" * 62)
    print("  in the viewer: RGB triad = desired attitude q_d from the")
    print("  flatness map, yellow ball = reference point on the path,")
    print("  colored markers = the trajectory itself.")
    s = run(traj=sc["traj"](), xml_path=sc["xml"], cycles=args.cycles,
            noise=noise, speed=args.speed, show=not args.no_gui)
    print(f"\n  result: 3D RMS {s['rms_3d_m'] * 100:.1f} cm | "
          f"radial RMS {s.get('rms_radial_m', float('nan')) * 100:.1f} cm | "
          f"duration {s['duration_s']:.1f} s | "
          f"peak rotor {s['max_rotor_thrust_n']:.2f} N")


if __name__ == "__main__":
    main()
