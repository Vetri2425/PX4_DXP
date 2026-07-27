"""Telemetry / status contract for the Spray Controller V2 (plan §6).

PURE module — no rclpy/ROS imports, stdlib only. Import & unit-test on
plain Python 3 (no ROS runtime required).

Root cause this module exists to make structurally impossible (plan §1
defect #1, `main`'s crash loop): a raw ``float("inf")``/``float("nan")``
reaching ``json.dumps(allow_nan=False)`` raised, and that raise crashed
the whole spray-status publish path -> pipeline restart -> ~11s OFFBOARD
blackout -> ~1.5m corner drift on the physical rover.

Design (plan §6): ``SpraySessionStatus`` is a typed, frozen dataclass.
The ``Optional[float]`` fields (``distance_to_boundary_m``,
``xtrack_error_m``) carry ``None`` for "no value" (e.g. continuous-mode
mission with zero boundary transitions) -- they are NEVER inf/nan. The
sanitizing happens once, at construction time via ``make_status``, not
as a downstream recursive scrubber bolted on after the fact. Because a
correctly-constructed status can never hold inf/nan,
``json.dumps(..., allow_nan=False)`` in ``status_to_json_safe`` is an
unreachable backstop in normal operation -- belt-and-suspenders, not the
only line of defense.

Note on the ``spraying`` field: the invariant "``spraying=True`` only
when ``fsm_state == 'ON_CONFIRMED'``" (plan §4) is enforced by the FSM /
node that builds a ``SpraySessionStatus``, not by this module. This
module faithfully stores whatever it is given -- it has no way to know
the caller's FSM state independently, so it cannot validate the
invariant itself. See ``src/test_spray_status.py`` for a test that
documents this division of responsibility.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from typing import Any, Optional

STATUS_SCHEMA_VERSION = 1

# §4 SprayState names -- kept here as a documentation reference only;
# fsm_state is a plain str field (not an enum) so this module stays
# decoupled from wherever the FSM class itself lives.
SPRAY_FSM_STATES = (
    "OFF_UNCONFIRMED",
    "OFF_CONFIRMED",
    "ON_PENDING",
    "ON_CONFIRMED",
    "OFF_PENDING",
    "RECOVERY",
    "DISABLED",
)


@dataclass(frozen=True)
class SpraySessionStatus:
    schema_version: int
    mode: str
    fsm_state: str  # one of the SPRAY_FSM_STATES names
    spraying: bool  # true only if fsm_state == "ON_CONFIRMED"
    desired: bool
    manual_active: bool
    safety_ok: bool
    safety_reason: str
    distance_to_boundary_m: Optional[float]  # None, NEVER inf/nan
    gps_fix_ok: bool
    gps_fix_name: str
    xtrack_error_m: Optional[float]  # None, NEVER inf/nan
    mode_state: dict  # small mode-specific fields


def sanitize_optional_float(x: Any) -> Optional[float]:
    """Return a finite ``float``, or ``None`` for missing/non-finite input.

    This is the ONE place a non-finite value (inf/-inf/nan) becomes
    ``None``. ``None`` in -> ``None`` out. Anything that fails
    ``float()`` conversion or is non-finite also becomes ``None`` rather
    than raising, since this function sits directly in the path of
    "make bad telemetry values structurally harmless".
    """
    if x is None:
        return None
    try:
        f = float(x)
    except (TypeError, ValueError):
        return None
    if math.isinf(f) or math.isnan(f):
        return None
    return f


def _sanitize_mode_state_key(key: Any) -> Any:
    """Coerce a dict key to something json.dumps accepts as an object key.

    json only accepts str/int/float/bool/None keys, and raises TypeError on
    anything else (tuple/bytes/object) and ValueError on a nan/inf float key
    — either would crash the status publish (defect-#1 class). So: pass
    through str/int/bool/None and FINITE floats unchanged (json stringifies
    numeric keys itself); coerce a non-finite float, or any other type, to
    its string form.
    """
    if key is None or isinstance(key, (str, int)):  # bool is an int subclass
        return key
    if isinstance(key, float) and not (math.isnan(key) or math.isinf(key)):
        return key
    return str(key)


def _sanitize_mode_state(value: Any) -> Any:
    """Recursively sanitize floats inside a mode_state dict/list/scalar.

    Non-finite floats become ``None``; everything else is passed through
    unchanged. Applied only inside ``make_status`` (not
    ``status_to_dict``/``status_to_json_safe``) so the dataclass itself
    is the single point where sanitization happens.
    """
    if isinstance(value, float):
        return sanitize_optional_float(value)
    if isinstance(value, dict):
        # Sanitize KEYS too: a non-finite float key survives value
        # sanitization and would still make json.dumps(allow_nan=False)
        # raise — the exact defect-#1 crash class this module exists to
        # make impossible. JSON object keys must be strings anyway, so a
        # non-finite float key is coerced to its repr string.
        return {_sanitize_mode_state_key(k): _sanitize_mode_state(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        sanitized = [_sanitize_mode_state(v) for v in value]
        return sanitized if isinstance(value, list) else tuple(sanitized)
    return value


def make_status(**kwargs: Any) -> SpraySessionStatus:
    """Convenience constructor for ``SpraySessionStatus``.

    Runs ``distance_to_boundary_m`` and ``xtrack_error_m`` through
    ``sanitize_optional_float`` BEFORE building the frozen dataclass, and
    recursively sanitizes any float values inside ``mode_state`` --  so
    even a caller that passes inf/nan ends up with a status object that
    structurally cannot contain them.

    Defaults: ``schema_version=STATUS_SCHEMA_VERSION``, ``mode_state={}``
    (a fresh dict copy per call, never a shared mutable default).
    """
    kwargs = dict(kwargs)
    kwargs.setdefault("schema_version", STATUS_SCHEMA_VERSION)

    mode_state = kwargs.get("mode_state")
    if mode_state is None:
        mode_state = {}
    kwargs["mode_state"] = _sanitize_mode_state(dict(mode_state))

    kwargs["distance_to_boundary_m"] = sanitize_optional_float(
        kwargs.get("distance_to_boundary_m")
    )
    kwargs["xtrack_error_m"] = sanitize_optional_float(kwargs.get("xtrack_error_m"))

    return SpraySessionStatus(**kwargs)


def status_to_dict(status: SpraySessionStatus) -> dict:
    """Plain dict of all fields."""
    return asdict(status)


def status_to_json_safe(status: SpraySessionStatus) -> str:
    """Serialize ``status`` to a JSON string, guaranteed not to raise.

    ``allow_nan=False`` keeps inf/nan out (they are already sanitized to
    None by ``make_status``). ``default=str`` is the belt-and-suspenders
    that closes the OTHER crash class: a ``mode_state`` value of a type
    json can't serialize natively (a ``set``, ``bytes``, ``Decimal``, a
    custom object) would otherwise raise ``TypeError`` at the publish
    boundary and take down the node — the exact "status-publish crash ->
    pipeline restart -> OFFBOARD blackout" failure this module exists to
    make impossible. With ``default=str`` any such value is stringified
    instead of crashing. mode_state is small and debug-only, so a stringy
    fallback for an exotic value is strictly better than a crash.
    """
    return json.dumps(status_to_dict(status), allow_nan=False, default=str)
