"""Tests for the extension trajectories: E2 (figure-8, vertical orbit,
yaw-follow) and E1 (min-snap / min-jerk waypoint polynomials) - extension
proposals Phase A, on the frozen double-loop LQR stack (gains untouched)."""
import numpy as np
import pytest

from flatness.polytraj import PolyTrajectory
from flatness.trajectories import (
    CircleTrajectory, FigureEightTrajectory, VerticalOrbitTrajectory,
)

WPS = np.array([
    [0.0, 0.0, 0.15],
    [1.5, 0.0, 1.2],
    [1.5, 2.0, 1.2],
    [0.0, 2.0, 1.2],
    [0.0, 0.0, 0.8],
])


class TestE2Generators:
    @pytest.mark.parametrize("traj", [
        CircleTrajectory(), FigureEightTrajectory(), VerticalOrbitTrajectory()])
    def test_starts_at_rest(self, traj):
        r = traj.evaluate(0.0)
        assert np.linalg.norm(r.v) < 1e-12
        assert np.linalg.norm(r.a) < 1e-12

    def test_figure8_geometry(self):
        t = FigureEightTrajectory()
        # the 8's center crossing every half period; extents A and B
        period = 2.0 * np.pi / t.omega          # periodic after one circuit
        a = t.evaluate(t.climb_time + 1.0).p[:2]
        b = t.evaluate(t.climb_time + 1.0 + period).p[:2]
        assert np.allclose(a, b, atol=1e-9)     # closes on itself
        xs, ys = [], []
        for k in range(200):
            r = t.evaluate(t.climb_time + k / 200 * 2 * np.pi / t.omega)
            xs.append(r.p[0]); ys.append(r.p[1])
        assert max(xs) == pytest.approx(t.A, abs=0.02)
        assert max(ys) == pytest.approx(t.B, abs=0.02)

    def test_orbit_altitude_swing(self):
        t = VerticalOrbitTrajectory()
        zs = [t.evaluate(t.climb_time + k / 100 * 2 * np.pi / t.omega).p[2]
              for k in range(100)]
        assert max(zs) == pytest.approx(t.alt + t.h, abs=1e-4)
        assert min(zs) == pytest.approx(t.alt - t.h, abs=1e-4)

    def test_yaw_follow_is_tangent(self):
        t = CircleTrajectory(yaw_follow=True)
        r = t.evaluate(6.0)                     # steady section
        assert r.psi == pytest.approx(np.arctan2(r.v[1], r.v[0]), abs=1e-9)

    @pytest.mark.parametrize("traj", [
        FigureEightTrajectory(), VerticalOrbitTrajectory()])
    def test_closed_loop_tracking(self, traj):
        from flatness.run_paper import run
        s = run(traj=traj, cycles=0.75, out_root=None)
        assert s["rms_3d_m"] < 0.10, s      # aggressive paths, frozen gains
        assert s["max_rotor_thrust_n"] <= 1.95 + 1e-9


class TestE1PolyTrajectory:
    def test_waypoints_hit_exactly(self):
        t = PolyTrajectory.through(WPS, speed=0.5, d=4)
        for i, wp in enumerate(WPS):
            r = t.evaluate(float(t.times[i]))
            assert np.allclose(r.p, wp, atol=1e-8)

    def test_rest_at_both_ends(self):
        t = PolyTrajectory.through(WPS, speed=0.5, d=4)
        for tt in (0.0, t.duration):
            r = t.evaluate(tt)
            assert np.linalg.norm(r.v) < 1e-9
            assert np.linalg.norm(r.a) < 1e-9

    @pytest.mark.parametrize("d", [3, 4])
    def test_continuity_at_knots(self, d):
        # The per-segment construction pins p, v and a at every knot (C2);
        # jerk is deliberately free at knots (documented trade in
        # polytraj.py - the free coefficients buy the min-snap interior).
        t = PolyTrajectory.through(WPS, speed=0.35, d=d)
        e = 1e-7
        for i in range(1, len(WPS) - 1):
            tk = float(t.times[i])
            for order in (1, 2):
                a = t._derivs(tk - e, order)[order]
                b = t._derivs(tk + e, order)[order]
                assert np.linalg.norm(b - a) < 1e-4, (i, order)

    def test_flatref_carries_jerk_and_snap(self):
        t = PolyTrajectory.through(WPS, speed=0.5, d=4)
        r = t.evaluate(1.0)
        assert r.jerk is not None and r.snap is not None
        # numeric derivative cross-check on the snap channel
        e = 1e-4
        jerk_num = (t._derivs(1.0 + e, 3)[3] - t._derivs(1.0 - e, 3)[3]) / (2 * e)
        assert np.allclose(r.snap, jerk_num, atol=1e-2)

    def test_minsnap_beats_minjerk_on_snap_objective(self):
        snap = PolyTrajectory.through(WPS, speed=0.5, d=4)
        jerk = PolyTrajectory.through(WPS, speed=0.5, d=3)
        assert snap.snap_integral() < 0.85 * jerk.snap_integral()

    def test_closed_loop_waypoint_mission(self):
        # 0.35 m/s: at 0.5 m/s the frozen-gain stack corner-cuts the 90-deg
        # waypoint turns (curvature-limited: radius ~ v^2/a_max ~ 0.16 m) -
        # an honest boundary, documented in the README, that motivates the
        # feedforward (E3) and MPC (E7) extensions.
        from flatness.run_paper import run
        t = PolyTrajectory.through(WPS, speed=0.35, d=4)
        s = run(traj=t, out_root=None)
        assert s["rms_3d_m"] < 0.10, s
        assert s["max_rotor_thrust_n"] <= 1.95 + 1e-9
