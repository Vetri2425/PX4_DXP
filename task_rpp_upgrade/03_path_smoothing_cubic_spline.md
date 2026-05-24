# 03 (DEPRECATED) — Cubic-spline path smoothing

> **DEPRECATED 2026-05-24.** Interpolating cubic splines through waypoints
> overshoot at sharp corners (Runge-like oscillation). For a marking rover
> where the path IS the painted line, rounding corners is wrong.
>
> **Use `03_path_geometry_and_stanley_tracking.md` instead.** That spec uses
> arc-length parameterisation of the **polyline** (no interpolation between
> waypoints), windowed Menger curvature for κ(s), and vertex classification
> (SMOOTH / SOFT / HARD) for the pivot sub-state. The path stays exactly as
> the planner gave it.

---

# (Original spec below, kept for historical context — DO NOT IMPLEMENT)