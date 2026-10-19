"""Flat-output trajectories — Choutri & Lagha 2017, Section V study case.

Flat outputs (Eq. 13): sigma = (x, y, z, psi) plus derivatives up to the
second order, which is all the flatness map of Eqs. (15)/(19)-(21) needs.

The paper's study case: a circular path at fixed altitude 1 m starting from
p0 = (-1, 0, 0) — i.e. center (0, 0, 1), radius 1 m (the start point is on
the circle, which fixes the geometry). The paper does not give the circle
period; the default tangential speed is 0.5 m/s. The X/Y responses in the
paper's Figs. 4-5 lag about 1 s "due to the altitude settling time", i.e.
the vehicle starts on the ground and climbs onto the circle: a smooth
(minimum-jerk) altitude ramp 0 -> 1 m reproduces this.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FlatRef:
    """Flat outputs and derivatives at one time instant. jerk/snap are
    optional (provided by the polynomial trajectories; the flatness map of
    Eqs. (15)/(19)-(21) itself only needs up to acceleration)."""

    p: np.ndarray      # (3,) position [m]
    v: np.ndarray      # (3,) velocity [m/s]
    a: np.ndarray      # (3,) acceleration [m/s^2]
    psi: float         # yaw [rad]
    psi_dot: float     # yaw rate [rad/s]
    jerk: np.ndarray | None = None   # (3,) [m/s^3], optional
    snap: np.ndarray | None = None   # (3,) [m/s^4], optional


def _min_jerk(t: float, T: float, total: float) -> tuple[float, float, float]:
    """Minimum-jerk scalar profile s(t), s_dot, s_ddot over [0, T] in [0, 1]."""
    if t >= T:
        return total, 0.0, 0.0
    x = t / T
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    sd = (30 * x**2 - 60 * x**3 + 30 * x**4) / T
    sdd = (60 * x - 180 * x**2 + 120 * x**3) / T**2
    return total * s, total * sd, total * sdd


def _rate_ramp(t: float, T: float, omega: float) -> tuple[float, float, float]:
    """Angle, rate, acceleration with rate ramping 0 -> omega via the
    min-jerk profile over [0, T] (starts at rest, zero angular accel), then
    constant omega. Shared by the circle / figure-8 / orbit generators."""
    if t >= T:
        return omega * (t - T / 2.0), omega, 0.0
    x = t / T
    theta = omega * T * (2.5 * x**4 - 3.0 * x**5 + x**6)
    theta_dot = omega * (10 * x**3 - 15 * x**4 + 6 * x**5)
    theta_ddot = omega * (30 * x**2 - 60 * x**3 + 30 * x**4) / T
    return theta, theta_dot, theta_ddot


def _heading(vx: float, vy: float, ax: float, ay: float) -> tuple[float, float]:
    """yaw-follow heading along the velocity tangent (atan2 is periodic, so
    the wrapped value feeds q_zd directly with no unwrap needed); rate via
    the tangent formula, guarded at near-zero speed (undefined at rest)."""
    speed2 = vx * vx + vy * vy
    if speed2 < 1e-8:
        return 0.0, 0.0
    return float(np.arctan2(vy, vx)), float((vx * ay - vy * ax) / max(speed2, 1e-8))


class CircleTrajectory:
    """The paper's circular reference: radius 1 m at altitude 1 m from
    (-1, 0, 0), climbed onto with a smooth altitude ramp. The angular rate
    also ramps 0 -> omega with the same profile, so the reference starts
    AT REST at the start point (the paper's Figs. 4-5 show X/Y following
    with ~1 s delay from rest; a full-speed reference at t=0 would demand
    an instantaneous tilt beyond the mixer's hover headroom)."""

    def __init__(self, radius: float = 1.0, alt: float = 1.0,
                 speed: float = 0.5, climb_time: float = 2.0,
                 yaw_follow: bool = False):
        self.r = float(radius)
        self.alt = float(alt)
        self.omega = float(speed) / float(radius)   # rad/s around +z
        self.climb_time = float(climb_time)
        self.yaw_follow = yaw_follow  # heading along the path (default: hold 0)

    def _theta(self, t: float) -> tuple[float, float, float]:
        """Angle, rate, acceleration on the circle: rate ramps 0 -> omega
        with the min-jerk profile over the climb, then holds omega."""
        return _rate_ramp(t, self.climb_time, self.omega)

    def evaluate(self, t: float) -> FlatRef:
        """sigma(t) = (x, y, z, psi) with derivatives, starting at (-1, 0, 0):
        p(t) = (-r cos(th), r sin(th), z(t)) — clockwise seen from +z
        (cross(z, dp/dt) points inward)."""
        th, th_d, th_dd = self._theta(t)
        p = np.array([-self.r * np.cos(th), self.r * np.sin(th), 0.0])
        v = self.r * np.array([th_d * np.sin(th), th_d * np.cos(th), 0.0])
        a = self.r * np.array(
            [th_dd * np.sin(th) + th_d**2 * np.cos(th),
             th_dd * np.cos(th) - th_d**2 * np.sin(th), 0.0])
        z, z_dot, z_ddot = _min_jerk(t, self.climb_time, self.alt)
        p[2], v[2], a[2] = z, z_dot, z_ddot
        psi = 0.0
        psi_dot = 0.0
        if self.yaw_follow:
            psi, psi_dot = _heading(v[0], v[1], a[0], a[1])
        return FlatRef(p=p, v=v, a=a, psi=float(psi), psi_dot=float(psi_dot))

    @property
    def duration(self) -> float:
        """Two full circles — enough to show steady tracking (paper Figs. 4-7)."""
        return 2.0 * 2.0 * np.pi / self.omega + self.climb_time


class FigureEightTrajectory:
    """Lissajous figure-8: x = A sin(th), y = B sin(2 th) at altitude alt,
    climbed onto with the shared min-jerk rate ramp (starts at rest at the
    8's center, on the ground). Peak speed is max(A, 2B)*omega; pick omega
    from the desired peak tangential speed."""

    name = "figure8"

    def __init__(self, a_x: float = 1.5, b_y: float = 1.0, alt: float = 1.0,
                 speed: float = 0.6, climb_time: float = 2.0,
                 yaw_follow: bool = False):
        self.A = float(a_x)
        self.B = float(b_y)
        self.alt = float(alt)
        self.omega = float(speed) / max(self.A, 2.0 * self.B)
        self.climb_time = float(climb_time)
        self.yaw_follow = yaw_follow

    def evaluate(self, t: float) -> FlatRef:
        th, th_d, th_dd = _rate_ramp(t, self.climb_time, self.omega)
        A, B = self.A, self.B
        p = np.array([A * np.sin(th), B * np.sin(2.0 * th), 0.0])
        v = np.array([A * th_d * np.cos(th), 2.0 * B * th_d * np.cos(2.0 * th), 0.0])
        a = np.array([
            A * (th_dd * np.cos(th) - th_d**2 * np.sin(th)),
            2.0 * B * (th_dd * np.cos(2.0 * th) - 2.0 * th_d**2 * np.sin(2.0 * th)),
            0.0])
        z, z_dot, z_ddot = _min_jerk(t, self.climb_time, self.alt)
        p[2], v[2], a[2] = z, z_dot, z_ddot
        psi = psi_dot = 0.0
        if self.yaw_follow:
            psi, psi_dot = _heading(v[0], v[1], a[0], a[1])
        return FlatRef(p=p, v=v, a=a, psi=float(psi), psi_dot=float(psi_dot))

    @property
    def duration(self) -> float:
        """Two full circuits of the 8."""
        return 2.0 * 2.0 * np.pi / self.omega + self.climb_time


class VerticalOrbitTrajectory:
    """Cone/orbit: horizontal circle of radius r with z = alt + h sin(th) -
    the thrust vector sweeps a cone, stress-testing the tilt construction.
    Starts at (−r, 0, 0) at rest (sin th(0) = 0, rate ramp starts at zero)."""

    name = "orbit"

    def __init__(self, radius: float = 1.0, alt: float = 1.0, h_z: float = 0.5,
                 speed: float = 0.5, climb_time: float = 2.0,
                 yaw_follow: bool = False):
        self.r = float(radius)
        self.alt = float(alt)
        self.h = float(h_z)
        self.omega = float(speed) / float(radius)
        self.climb_time = float(climb_time)
        self.yaw_follow = yaw_follow

    def evaluate(self, t: float) -> FlatRef:
        th, th_d, th_dd = _rate_ramp(t, self.climb_time, self.omega)
        r, h = self.r, self.h
        p = np.array([-r * np.cos(th), r * np.sin(th), h * np.sin(th)])
        v = r * th_d * np.array([np.sin(th), np.cos(th), 0.0])             + np.array([0.0, 0.0, h * th_d * np.cos(th)])
        a = r * np.array([
            th_dd * np.sin(th) + th_d**2 * np.cos(th),
            th_dd * np.cos(th) - th_d**2 * np.sin(th), 0.0])             + np.array([0.0, 0.0, h * (th_dd * np.cos(th) - th_d**2 * np.sin(th))])
        z0, z0d, z0dd = _min_jerk(t, self.climb_time, self.alt)
        p[2] += z0
        v[2] += z0d
        a[2] += z0dd
        psi = psi_dot = 0.0
        if self.yaw_follow:
            psi, psi_dot = _heading(v[0], v[1], a[0], a[1])
        return FlatRef(p=p, v=v, a=a, psi=float(psi), psi_dot=float(psi_dot))

    @property
    def duration(self) -> float:
        """Two full revolutions."""
        return 2.0 * 2.0 * np.pi / self.omega + self.climb_time
