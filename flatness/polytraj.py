"""Minimum-snap / minimum-jerk waypoint trajectories (extension E1).

Mellinger & Kumar's classic result, mapped onto this repo's flatness stack:
because the flatness input depends on derivatives of the flat outputs (jerk
drives body rates, snap drives torques), the smoothest FLYABLE path through
waypoints minimizes the integral of squared snap (4th derivative) - and the
minimizer is a piecewise polynomial whose coefficients solve ONE quadratic
program with waypoint and continuity constraints:

    min_c  sum_i  int_0^{Ti} || p_i''''(t) ||^2 dt          (= c^T Q c)
    s.t.   p_i(0) = wp_i ,  p_N(T_N) = wp_N                  (waypoints)
           intermediate waypoints: v and a pinned along the corridor
           tangent (without pinning, minimizing only snap lets long
           segments loiter-then-teleport: position is four integrals away
           from the cost, so a slow huge excursion is nearly free - the
           documented failure mode this implementation fixes)
           remaining derivative orders continuous at each knot
           v(0) = a(0) = v(T) = a(T) = 0                     (rest ends)

NUMERICS: everything is solved in NORMALIZED time (t/T in [0,1]) with rows
rescaled by T^order - raw monomial bases at multi-second T span 10+ orders
of magnitude and the KKT solve returns loiter-then-teleport references
(measured). With d = 4 (snap) segments are degree 7; d = 3 (jerk) degree 5.
The A/B benchmark compares the two on identical waypoints and times.

evaluate(t) yields a FlatRef with analytic p, v, a, jerk AND snap (FlatRef's
optional fields), ready for the flatness map and for the omega_d/tau_ff
feedforward (extension E3).
"""
from __future__ import annotations

import numpy as np

from flatness.trajectories import FlatRef, _heading


def _falling(k: int, order: int) -> float:
    """k·(k-1)·...·(k-order+1)."""
    c = 1.0
    for j in range(order):
        c *= (k - j)
    return c


def _q_norm(deg: int, d: int) -> np.ndarray:
    """int_0^1 (d^d/dtau^d p)^2 dtau in the normalized monomial basis."""
    n = deg + 1
    Q = np.zeros((n, n))
    for k in range(d, n):
        for l in range(d, n):
            expo = k + l - 2 * d + 1
            Q[k, l] = _falling(k, d) * _falling(l, d) / expo
    return Q


def _deriv_norm(deg: int, frac: float, order: int) -> np.ndarray:
    """d^order/dtau^order of the normalized monomial basis at tau in [0,1]."""
    vals = np.zeros(deg + 1)
    for k in range(order, deg + 1):
        vals[k] = _falling(k, order) * frac ** (k - order)
    return vals


class PolyTrajectory:
    """Piecewise-polynomial flat-output trajectory through waypoints.

    Coefficients live in each segment's NORMALIZED time; evaluation applies
    the chain rule T^-order per derivative order. Yaw follows the velocity
    tangent (guarded at rest). `climb_time` (the benchmark's settling mask)
    is the first segment's duration."""

    def __init__(self, waypoints, times, d: int = 4, name: str = "minsnap",
                 pin_velocities: bool = True):
        self.wps = np.asarray(waypoints, float)
        self.times = np.asarray(times, float)
        self.d = int(d)
        self.name = name
        self.seg_T = np.diff(self.times)
        self.segments = self._solve(pin_velocities)

    # ------------------------------------------------------------------
    def _solve(self, pin_velocities: bool) -> list[np.ndarray]:
        """Per-segment closed form. With v and a pinned at BOTH ends of every
        segment (rest at the mission ends, corridor tangent velocity at the
        intermediate waypoints), each segment is an independent boundary
        value problem: 6 constraints on degree-(2d-1) coefficients. For
        d=3 (quintic) that determines the segment uniquely - the classic
        quintic Hermite spline. For d=4 (degree 7) two coefficients remain
        free and are chosen to minimize the segment's snap integral
        (particular solution + null-space 2x2 minimization) - exact, no
        global KKT conditioning to worry about (the monolithic solve was
        tried first and produced loiter-then-teleport references; see the
        module docstring)."""
        wps, d = self.wps, self.d
        n_seg = len(wps) - 1
        seg_T = self.seg_T
        deg = 2 * d - 1
        n = deg + 1

        # knot boundary values: rest at the ends, corridor tangent velocity
        # and zero acceleration at intermediate waypoints
        vel = np.zeros_like(wps)
        if pin_velocities:
            for i in range(1, n_seg):
                tangent = wps[i + 1] - wps[i - 1]
                norm = float(np.linalg.norm(tangent))
                if norm > 1e-9:
                    span = float(self.times[i + 1] - self.times[i - 1])
                    vel[i] = tangent / norm * min(norm / span, 0.8)
        acc = np.zeros_like(wps)

        segments = []
        for i in range(n_seg):
            T = float(seg_T[i])
            # rows in NORMALIZED time tau in [0,1]; the d/dtau row already
            # equals T·(d/dt), so the rhs carries the T^order factor and the
            # row stays unscaled (multiplying both by T^order leaves the
            # pinned value off by exactly 1/T - the bug this comment guards)
            A = np.array([
                _deriv_norm(deg, 0.0, 0),
                _deriv_norm(deg, 1.0, 0),
                _deriv_norm(deg, 0.0, 1),
                _deriv_norm(deg, 1.0, 1),
                _deriv_norm(deg, 0.0, 2),
                _deriv_norm(deg, 1.0, 2),
            ])
            B = np.array([
                wps[i], wps[i + 1],
                vel[i] * T, vel[i + 1] * T,
                acc[i] * T**2, acc[i + 1] * T**2,
            ])
            c_p, *_ = np.linalg.lstsq(A, B, rcond=None)   # particular solution
            # null space of A -> free directions; minimize the segment's
            # derivative-squared objective over them
            _, _, Vt = np.linalg.svd(A)
            null = Vt[len(B):].T                            # (n, n-6)
            if null.shape[1] > 0:
                Qn = _q_norm(deg, d) / T ** (2 * d - 1)
                H = null.T @ Qn @ null
                g = null.T @ Qn @ c_p
                try:
                    x = -np.linalg.solve(H, g)
                except np.linalg.LinAlgError:
                    x = -np.linalg.lstsq(H, g, rcond=None)[0]
                c_p = c_p + null @ x
            segments.append(c_p)
        return segments

    # ------------------------------------------------------------------
    @classmethod
    def through(cls, waypoints, speed: float = 0.5, d: int = 4,
                name: str = "minsnap") -> "PolyTrajectory":
        """Segment times from distance at a cruise speed (>= 1 s each)."""
        wps = np.asarray(waypoints, float)
        dist = np.linalg.norm(np.diff(wps, axis=0), axis=1)
        seg_t = np.maximum(dist / float(speed), 1.0)
        times = np.concatenate(([0.0], np.cumsum(seg_t)))
        return cls(wps, times, d=d, name=name)

    def _derivs(self, t: float, upto: int) -> np.ndarray:
        i = int(np.searchsorted(self.times, t, side="right") - 1)
        i = min(max(i, 0), len(self.segments) - 1)
        T = float(self.seg_T[i])
        frac = (t - float(self.times[i])) / T
        coef = self.segments[i]
        deg = coef.shape[0] - 1
        out = np.zeros((upto + 1, 3))
        for order in range(upto + 1):
            for k in range(order, deg + 1):
                out[order] += (coef[k] * _falling(k, order)
                               * frac ** (k - order) / T ** order)
        return out

    def evaluate(self, t: float) -> FlatRef:
        upto = min(self.d, 4)
        D = self._derivs(t, upto)
        psi = psi_dot = 0.0
        if np.linalg.norm(D[1, :2]) > 1e-6:
            psi, psi_dot = _heading(D[1, 0], D[1, 1], D[2, 0], D[2, 1])
        return FlatRef(
            p=D[0], v=D[1], a=D[2], psi=float(psi), psi_dot=float(psi_dot),
            jerk=D[3] if upto >= 3 else None,
            snap=D[4] if upto >= 4 else None)

    @property
    def climb_time(self) -> float:
        return float(self.seg_T[0])

    @property
    def duration(self) -> float:
        return float(self.times[-1])

    def snap_integral(self) -> float:
        """int ||snap||^2 dt over the whole path (the objective)."""
        total = 0.0
        for i, coef in enumerate(self.segments):
            T = float(self.seg_T[i])
            Qn = _q_norm(coef.shape[0] - 1, 4) / T ** 7
            total += float(np.trace(coef.T @ Qn @ coef))
        return total
