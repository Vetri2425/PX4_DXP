"""ROS-free contract for the independent spray fail-closed watchdog.

The spray controller publishes a short-lived lease while an ON command is
allowed to remain energized.  A separate process consumes the lease and sends
OFF whenever it is absent, denied, malformed, or stale.  Keeping the contract
pure makes the safety decisions testable without ROS or vehicle hardware.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional


LEASE_SCHEMA_VERSION = 1
LEASE_TOPIC = "/spray/safety_lease"
SUPPORTED_BACKENDS = frozenset({"mavlink_actuator", "mavlink_servo_pwm"})


class LeaseValidationError(ValueError):
    """Raised when a lease cannot safely describe the physical OFF command."""


@dataclass(frozen=True)
class SpraySafetyLease:
    allow_on: bool
    command_seq: int
    backend: str
    actuator_set_index: int
    off_value: float
    servo_instance: int
    off_pwm_us: int


def _strict_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise LeaseValidationError(f"{name} must be an integer")
    return value


def _strict_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LeaseValidationError(f"{name} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise LeaseValidationError(f"{name} must be finite")
    return parsed


def lease_to_json(lease: SpraySafetyLease) -> str:
    """Serialize a validated controller lease with a versioned schema."""
    validate_lease(lease)
    return json.dumps(
        {
            "schema": LEASE_SCHEMA_VERSION,
            "allow_on": lease.allow_on,
            "command_seq": lease.command_seq,
            "backend": lease.backend,
            "actuator_set_index": lease.actuator_set_index,
            "off_value": lease.off_value,
            "servo_instance": lease.servo_instance,
            "off_pwm_us": lease.off_pwm_us,
        },
        separators=(",", ":"),
        sort_keys=True,
        allow_nan=False,
    )


def lease_from_json(payload: str) -> SpraySafetyLease:
    """Parse a lease, rejecting coercions and unsafe actuator values."""
    try:
        raw = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as exc:
        raise LeaseValidationError(f"invalid JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise LeaseValidationError("lease must be a JSON object")
    if _strict_int(raw.get("schema"), "schema") != LEASE_SCHEMA_VERSION:
        raise LeaseValidationError("unsupported lease schema")
    allow_on = raw.get("allow_on")
    if not isinstance(allow_on, bool):
        raise LeaseValidationError("allow_on must be boolean")
    lease = SpraySafetyLease(
        allow_on=allow_on,
        command_seq=_strict_int(raw.get("command_seq"), "command_seq"),
        backend=str(raw.get("backend", "")),
        actuator_set_index=_strict_int(
            raw.get("actuator_set_index"), "actuator_set_index"
        ),
        off_value=_strict_float(raw.get("off_value"), "off_value"),
        servo_instance=_strict_int(raw.get("servo_instance"), "servo_instance"),
        off_pwm_us=_strict_int(raw.get("off_pwm_us"), "off_pwm_us"),
    )
    validate_lease(lease)
    return lease


def validate_lease(lease: SpraySafetyLease) -> None:
    if not isinstance(lease.allow_on, bool):
        raise LeaseValidationError("allow_on must be boolean")
    _strict_int(lease.command_seq, "command_seq")
    _strict_int(lease.actuator_set_index, "actuator_set_index")
    _strict_int(lease.servo_instance, "servo_instance")
    _strict_int(lease.off_pwm_us, "off_pwm_us")
    _strict_float(lease.off_value, "off_value")
    if lease.command_seq < 0:
        raise LeaseValidationError("command_seq must be non-negative")
    if lease.backend not in SUPPORTED_BACKENDS:
        raise LeaseValidationError(f"unsupported backend {lease.backend!r}")
    if not 1 <= lease.actuator_set_index <= 6:
        raise LeaseValidationError("actuator_set_index must be in 1..6")
    if not math.isfinite(lease.off_value) or not -1.0 <= lease.off_value <= 1.0:
        raise LeaseValidationError("off_value must be finite and in [-1, 1]")
    if not 1 <= lease.servo_instance <= 16:
        raise LeaseValidationError("servo_instance must be in 1..16")
    if not 0 <= lease.off_pwm_us <= 2200:
        raise LeaseValidationError("off_pwm_us must be in 0..2200")


class SprayLeaseMonitor:
    """Track receive-time freshness and return why OFF is currently required."""

    def __init__(self, timeout_s: float = 0.35) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        self.timeout_s = float(timeout_s)
        self.last_lease: Optional[SpraySafetyLease] = None
        self.last_receive_s: Optional[float] = None
        self.invalid_reason: Optional[str] = None

    def observe(self, payload: str, now_s: float) -> SpraySafetyLease:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        lease = lease_from_json(payload)
        self.last_lease = lease
        self.last_receive_s = float(now_s)
        self.invalid_reason = None
        return lease

    def invalidate(self, reason: str) -> None:
        """Revoke ON immediately while retaining the last known OFF mapping."""
        self.invalid_reason = str(reason) or "invalid controller lease"

    def off_reason(self, now_s: float) -> Optional[str]:
        """Return None only while a fresh, valid lease explicitly allows ON."""
        if self.invalid_reason is not None:
            return self.invalid_reason
        if self.last_lease is None or self.last_receive_s is None:
            return "no controller lease"
        age_s = max(0.0, float(now_s) - self.last_receive_s)
        if age_s > self.timeout_s:
            return f"controller lease stale ({age_s:.3f}s > {self.timeout_s:.3f}s)"
        if not self.last_lease.allow_on:
            return "controller lease denies ON"
        return None
