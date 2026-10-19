"""In-engine MuJoCo recording of the flatness path-planning circle.

Runs the exact run_mujoco.run() control loop (same code path, so the
recording IS the benchmark run) inside flatness/circle_scene.xml — the
paper vehicle plus visual-only markers tracing the paper's Section V
reference circle (radius 1 m at altitude 1 m, start marker at (-1, 0)) —
and captures offscreen MuJoCo renders with a slowly orbiting chase camera,
assembled into an animated GIF with live time / radial / altitude error.

Usage:
    python -m flatness.record_mujoco [--cycles 2] [--noise 0.0]
Output:
    results/recordings/mujoco_flatness_RENDER.gif
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter

from flatness.run_mujoco import run

SCENE = Path(__file__).with_name("circle_scene.xml")
OUT = Path(__file__).resolve().parent.parent / "results" / "recordings"
W, H = 720, 540
EVERY_S = 0.2       # capture cadence [s of simulated time]
FPS = 12
R_REF = 1.0         # circle radius [m]
Z_REF = 1.0         # circle altitude [m]


def record(cycles: float = 2.0, noise: float = 0.0, speed: float = 0.5,
           traj=None, scene: str | None = None, out_name: str | None = None,
           label: str | None = None) -> dict:
    """traj: any flat-output trajectory (default: the paper's circle).
    scene: marker scene to fly in ('circle' default, 'figure8', 'waypoints');
    out_name: gif stem (defaults to the scene name)."""
    import mujoco

    from quat_sitl import quaternion as quat

    from sim.mujoco.record import make_render_pair, set_refpoint, set_triad

    scenes = {"circle": SCENE,
              "figure8": Path(__file__).with_name("figure8_scene.xml"),
              "waypoints": Path(__file__).with_name("waypoints_scene.xml")}
    scene_xml = scenes[scene or "circle"]
    if traj is None:
        from flatness.trajectories import CircleTrajectory
        traj = CircleTrajectory(radius=1.0, alt=1.0, speed=speed)
    out_stem = out_name or (scene or "circle")
    pretty = label or {"circle": "flatness + LQR circle",
                       "figure8": "flatness + LQR figure-8",
                       "waypoints": "flatness + LQR waypoint mission (min-snap)"}

    # render puppet: MuJoCo's offscreen Renderer skips the drone's
    # visual-only (contype=0) geoms — see sim/mujoco/record.make_render_pair
    r_model, r_data, opt = make_render_pair(scene_xml)
    renderer = mujoco.Renderer(r_model, height=H, width=W)
    cam = mujoco.MjvCamera()
    frames: list[np.ndarray] = []
    stats: list[tuple[float, float, float, float]] = []
    last_cap = [-1.0]

    def frame_cb(t: float, data, q_d, p_ref) -> None:
        if t - last_cap[0] < EVERY_S:
            return
        last_cap[0] = t
        p = np.array(data.qpos[0:3])
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = [p[0] * 0.6, p[1] * 0.6, 0.6]  # frame drone + path
        cam.distance = 2.6 if out_stem != "waypoints" else 3.4
        cam.azimuth = -62.0 + 8.0 * t                     # slow orbit
        cam.elevation = -16.0
        r_data.qpos[:] = data.qpos          # puppet the render copy
        r_data.qvel[:] = data.qvel
        set_triad(r_data, p, quat.to_dcm(q_d))   # desired attitude (q_d)
        set_refpoint(r_data, p_ref)              # where on the path to be now
        mujoco.mj_forward(r_model, r_data)
        renderer.update_scene(r_data, camera=cam, scene_option=opt)
        frames.append(renderer.render())
        radial = float(np.linalg.norm(p[:2]))
        track_err = float(np.linalg.norm(p - p_ref))
        stats.append((float(t), radial, float(p[2]), track_err))

    summary = run(cycles=cycles, noise=noise, speed=speed, traj=traj,
                  frame_cb=frame_cb, xml_path=scene_xml,
                  out_root=OUT.parent / "flatness_render")
    renderer.close()

    fig = plt.figure(figsize=(7.6, 6.0))
    ax = fig.add_subplot(111)
    ax.set_axis_off()
    im = ax.imshow(np.zeros((H, W, 3), np.uint8))
    title = ax.set_title("", fontsize=11)

    def frame(i: int):
        im.set_array(frames[i])
        t, radial, z, track_err = stats[i]
        title.set_text(
            f"Choutri & Lagha 2017 - {pretty}, MuJoCo contact physics\n"
            f"t = {t:5.1f} s    3D tracking err = {track_err:5.3f} m\n"
            "solid drone = actual   RGB axes = desired attitude (q_d)   "
            "yellow ball = reference point   colored markers = path")
        return [im]

    OUT.mkdir(parents=True, exist_ok=True)
    anim = FuncAnimation(fig, frame, frames=len(frames), blit=False)
    gif = OUT / f"mujoco_flatness_{out_stem}_RENDER.gif"
    anim.save(str(gif), writer=PillowWriter(fps=FPS), dpi=100)
    plt.close(fig)
    print(f"wrote {gif} ({gif.stat().st_size // 1024} KB, {len(frames)} frames)")
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cycles", type=float, default=2.0)
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--all", action="store_true",
                    help="record circle + figure-8 + waypoint mission")
    args = ap.parse_args()
    if not args.all:
        print(record(cycles=args.cycles, noise=args.noise, speed=args.speed))
    else:
        from flatness.trajectories import FigureEightTrajectory
        from flatness.polytraj import PolyTrajectory
        print(record(cycles=args.cycles, noise=args.noise, speed=args.speed))
        print(record(traj=FigureEightTrajectory(), scene="figure8", cycles=1.0))
        WPS = [[0, 0, 0.15], [1.5, 0, 1.2], [1.5, 2.0, 1.2],
               [0, 2.0, 1.2], [0, 0, 0.8]]
        print(record(traj=PolyTrajectory.through(WPS, speed=0.35, d=4),
                     scene="waypoints", cycles=None))
