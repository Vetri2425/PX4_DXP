"""Is the EKF local-frame origin we cached still the frame PX4 is publishing?

Why this exists (field bug, 2026-07-27)
---------------------------------------
`resolve_surveyed_points` places a surveyed mission by translating it through
the EKF's DECLARED local-frame origin (`/mavros/global_position/gp_origin`).
That choice is right — it is the only datum that makes placement deterministic
(see mission_placement.py) — but it was trusted unconditionally:

  * The server caches the origin in process state. MAVROS restarting, or the
    FCU rebooting, does NOT restart the server, so the cache outlived the EKF
    session that produced it. Measured: three different origins in one
    afternoon on the same rig, up to 4.8 m apart.
  * `_request_ekf_origin_tick` early-returned forever once the origin arrived,
    so nothing ever went looking for the new one.
  * Two aborted runs at 14:47 were placed 2.15 m and 2.25 m off the surveyed
    line. The bags for those runs contain NO gp_origin at all (MAVROS had
    nothing latched) while the server still served one from the 14:39 session.
  * There is NO operator-visible symptom until the rover moves.

Freshness alone is not enough. At 15:10 the same rig had a PRESENT and RECENT
declared origin that was still 0.91 m away from the frame PX4 was actually
publishing. So the check here is a direct measurement, not a liveness proxy.

The measurement
---------------
PX4 publishes both halves of its own transform:

    GLOBAL_POSITION_INT  -> /mavros/global_position/global   (lat, lon)
    LOCAL_POSITION_NED   -> /mavros/local_position/pose      (pos_n, pos_e)

and the second is, by construction, the first projected through the EKF's
origin. So for a simultaneous pair:

    expected_local = latlon_to_ned(lat, lon, declared_lat, declared_lon)
    delta          = (pos_n, pos_e) - expected_local

`delta` is zero (to sensor noise) exactly when the declared origin IS the
frame's origin, and equals the origin displacement when it is not. Reporting
the same fault as an implied origin — the origin that WOULD satisfy the pair —
gives the operator a number directly comparable to the declared one:

    implied_origin = ned_to_latlon(-delta_n, -delta_e, declared_lat, declared_lon)

Projection model — deliberate, do not "fix" this
------------------------------------------------
`path_engine.ned.latlon_to_ned` is an exact port of PX4's own
`MapProjection::project()`: a SPHERICAL azimuthal-equidistant projection at
R = 6 371 000 m (geo.cpp CONSTANTS_RADIUS_OF_EARTH). That is the correct and
only correct model here, because this check INVERTS PX4's own transform — the
quantity being reconstructed was produced by that exact projection, so any
other model injects error that is *proportional to distance from the origin*.
`tools/analyze_mission.py:_metres_per_degree` uses the WGS84 ellipsoid instead,
which is right for its job (true ground distance for grading a painted line
against surveyed truth) and wrong for this one: the two models differ by 0.51 %
north / 0.13 % east at 13 degN, i.e. 51 cm of pure model error 100 m from the
origin — the same order as the fault being detected, and it would grow with
distance until it false-tripped the gate on a healthy rig. This is the same
B6' decision recorded in `path_engine/parsers/georef.py:metres_per_degree` and
in the `path_engine.ned` module docstring.

(At the 0.9 m separation measured in the field the two models agree to ~4 mm,
so the field numbers do not discriminate between them. Distance from the
origin does, which is why the reasoning has to come from the model, not from
one measurement.)

This module is pure: it takes a telemetry-state dict and returns a verdict.
The same function backs the placement gate, the telemetry stream and
GET /api/health/origin, so what the operator is shown is what placement
enforced — they cannot disagree.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from config import (
    GLOBAL_POSITION_STALE_MS,
    ORIGIN_CONSISTENCY_MAX_M,
    POSE_GLOBAL_MAX_SKEW_MS,
    POSE_STALE_MS,
)
from path_engine.ned import latlon_to_ned, ned_to_latlon

# ── Verdicts ─────────────────────────────────────────────────────────────────
# Only OK is trusted. Everything else is a refusal reason, never a downgrade.
OK = "OK"                      # declared origin agrees with the live frame
NO_ORIGIN = "NO_ORIGIN"        # gp_origin never received, or dropped as stale
ORIGIN_INVALID = "ORIGIN_INVALID"    # declared value is non-finite / out of bounds
UNVERIFIABLE = "UNVERIFIABLE"  # no usable simultaneous sample to check it against
INCONSISTENT = "INCONSISTENT"  # declared origin is NOT the frame PX4 is in


@dataclass(frozen=True)
class OriginHealth:
    """Verdict on the cached EKF local-frame origin. `trusted` is the gate."""

    status: str
    trusted: bool
    detail: str
    threshold_m: float = ORIGIN_CONSISTENCY_MAX_M
    declared: tuple[float, float] | None = None
    implied: tuple[float, float] | None = None
    delta_n_m: float | None = None
    delta_e_m: float | None = None
    delta_m: float | None = None
    # Why the cache was dropped, when it was (ros_node fills these into state).
    invalidated_reason: str | None = None
    sample: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "trusted": self.trusted,
            "detail": self.detail,
            "threshold_m": self.threshold_m,
            "declared_lat": self.declared[0] if self.declared else None,
            "declared_lon": self.declared[1] if self.declared else None,
            "implied_lat": self.implied[0] if self.implied else None,
            "implied_lon": self.implied[1] if self.implied else None,
            "delta_n_m": self.delta_n_m,
            "delta_e_m": self.delta_e_m,
            "delta_m": self.delta_m,
            "invalidated_reason": self.invalidated_reason,
            "sample": self.sample,
        }


def _finite(*values) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def _age_ok(state: dict, key: str, limit_ms: float) -> bool:
    raw = state.get(key)
    if raw is None:
        return False
    try:
        age = float(raw)
    except (TypeError, ValueError):
        return False
    return math.isfinite(age) and 0.0 <= age <= limit_ms


def implied_origin(
    lat: float, lon: float, pos_n: float, pos_e: float,
    declared_lat: float, declared_lon: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """One sample -> (implied origin lat/lon, (delta_n, delta_e) in metres).

    `delta` is the local-frame position error the declared origin implies:
    where PX4 says the rover is, minus where the declared origin says it should
    be. The implied origin is the declared origin moved by -delta, i.e. the
    origin that would make the sample self-consistent.
    """
    exp_n, exp_e = latlon_to_ned(lat, lon, declared_lat, declared_lon)
    delta_n = float(pos_n) - exp_n
    delta_e = float(pos_e) - exp_e
    imp_lat, imp_lon = ned_to_latlon(-delta_n, -delta_e, declared_lat, declared_lon)
    return (imp_lat, imp_lon), (delta_n, delta_e)


def evaluate_origin_health(
    state: dict,
    threshold_m: float = ORIGIN_CONSISTENCY_MAX_M,
) -> OriginHealth:
    """Grade the cached EKF origin against the live PX4 frame.

    `state` is a `RosBridgeNode.get_state()` snapshot. Pure — no ROS, no clock,
    no I/O — so the same snapshot always grades the same way.
    """
    invalidated = state.get("ekf_origin_invalid_reason")

    # ── 1. Do we even have a declared origin? ────────────────────────────────
    if not state.get("ekf_origin_received"):
        return OriginHealth(
            status=NO_ORIGIN,
            trusted=False,
            threshold_m=threshold_m,
            invalidated_reason=invalidated,
            detail=(
                "no EKF local-frame origin: /mavros/global_position/gp_origin "
                "has not been received"
                + (f" ({invalidated})" if invalidated else "")
            ),
        )

    d_lat = state.get("ekf_origin_lat")
    d_lon = state.get("ekf_origin_lon")
    if not _finite(d_lat, d_lon):
        return OriginHealth(
            status=ORIGIN_INVALID, trusted=False, threshold_m=threshold_m,
            invalidated_reason=invalidated,
            detail=f"declared EKF origin is non-finite ({d_lat}, {d_lon})",
        )
    d_lat, d_lon = float(d_lat), float(d_lon)
    if not (-90.0 <= d_lat <= 90.0 and -180.0 <= d_lon <= 180.0):
        return OriginHealth(
            status=ORIGIN_INVALID, trusted=False, threshold_m=threshold_m,
            declared=(d_lat, d_lon), invalidated_reason=invalidated,
            detail=(
                f"declared EKF origin ({d_lat:.8f}, {d_lon:.8f}) is outside "
                "valid latitude/longitude bounds"
            ),
        )

    # ── 2. Is there a usable simultaneous sample to check it against? ────────
    # Both halves of PX4's transform must be fresh AND close together in
    # receive time, or the "error" measured is just the rover having moved.
    reasons: list[str] = []
    if not state.get("pose_received"):
        reasons.append("no local pose")
    if not state.get("global_position_received"):
        reasons.append("no fused global position")
    if not _age_ok(state, "local_pose_age_ms", POSE_STALE_MS):
        reasons.append(f"local pose not fresh (<= {POSE_STALE_MS:.0f} ms)")
    if not _age_ok(state, "global_position_age_ms", GLOBAL_POSITION_STALE_MS):
        reasons.append(
            f"global position not fresh (<= {GLOBAL_POSITION_STALE_MS:.0f} ms)"
        )
    if not _age_ok(state, "pose_global_skew_ms", POSE_GLOBAL_MAX_SKEW_MS):
        reasons.append(
            f"pose/global samples not simultaneous (<= {POSE_GLOBAL_MAX_SKEW_MS:.0f} ms)"
        )
    lat, lon = state.get("lat"), state.get("lon")
    pos_n, pos_e = state.get("pos_n"), state.get("pos_e")
    if not _finite(lat, lon, pos_n, pos_e):
        reasons.append("pose/global sample contains non-finite values")
    elif not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
        reasons.append("fused global position is outside valid bounds")
    elif float(lat) == 0.0 and float(lon) == 0.0:
        reasons.append("fused global position is the null-island no-fix sentinel")

    if reasons:
        return OriginHealth(
            status=UNVERIFIABLE, trusted=False, threshold_m=threshold_m,
            declared=(d_lat, d_lon), invalidated_reason=invalidated,
            detail=(
                "declared EKF origin cannot be verified against the live PX4 "
                "frame: " + "; ".join(reasons)
            ),
        )

    # ── 3. The measurement ──────────────────────────────────────────────────
    (imp_lat, imp_lon), (delta_n, delta_e) = implied_origin(
        float(lat), float(lon), float(pos_n), float(pos_e), d_lat, d_lon
    )
    delta = math.hypot(delta_n, delta_e)
    sample = {
        "lat": float(lat), "lon": float(lon),
        "pos_n": float(pos_n), "pos_e": float(pos_e),
        "pose_global_skew_ms": state.get("pose_global_skew_ms"),
    }
    if not math.isfinite(delta):
        return OriginHealth(
            status=UNVERIFIABLE, trusted=False, threshold_m=threshold_m,
            declared=(d_lat, d_lon), invalidated_reason=invalidated, sample=sample,
            detail="origin consistency computation produced a non-finite result",
        )

    if delta > threshold_m:
        return OriginHealth(
            status=INCONSISTENT, trusted=False, threshold_m=threshold_m,
            declared=(d_lat, d_lon), implied=(imp_lat, imp_lon),
            delta_n_m=delta_n, delta_e_m=delta_e, delta_m=delta,
            invalidated_reason=invalidated, sample=sample,
            detail=(
                f"declared EKF origin ({d_lat:.8f}, {d_lon:.8f}) is NOT the frame "
                f"PX4 is publishing: the live pose/global pair implies "
                f"({imp_lat:.8f}, {imp_lon:.8f}), off by {delta:.3f} m "
                f"(dN {delta_n:+.3f}, dE {delta_e:+.3f}; limit {threshold_m:.2f} m). "
                "Most likely the FCU rebooted or the EKF reset and the cached "
                "origin is from the previous session."
            ),
        )

    return OriginHealth(
        status=OK, trusted=True, threshold_m=threshold_m,
        declared=(d_lat, d_lon), implied=(imp_lat, imp_lon),
        delta_n_m=delta_n, delta_e_m=delta_e, delta_m=delta,
        invalidated_reason=invalidated, sample=sample,
        detail=(
            f"declared EKF origin agrees with the live PX4 frame to "
            f"{delta:.3f} m (limit {threshold_m:.2f} m)"
        ),
    )
