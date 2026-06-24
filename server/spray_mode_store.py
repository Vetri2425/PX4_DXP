"""Per-path spray-mode configuration sidecar.

Stores the spray mode and all mode-specific params as a hidden JSON file
alongside the DXF/path file. The sidecar is merged into the staged mission
artifact at plan-and-stage time when spray_mode is omitted from
PathPlanRequest (None → use sidecar).

Sidecar path: .{basename}.spray_mode.json  (in the MISSION_DIR)
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from spray_config import validate_spray_configuration  # noqa: E402
from spray_mission_config import staged_spray_defaults  # noqa: E402

_SUFFIX = ".spray_mode.json"


def _sidecar_path(paths_dir: str, name: str) -> str:
    basename = os.path.basename(name)
    return os.path.join(paths_dir, f".{basename}{_SUFFIX}")


def load_spray_mode(paths_dir: str, name: str) -> dict[str, Any]:
    """Return sidecar merged over factory defaults.

    Falls back to pure defaults on FileNotFoundError or JSON decode error
    so callers never see a partial or corrupt config.
    """
    defaults = staged_spray_defaults()
    if not name:
        return defaults
    path = _sidecar_path(paths_dir, name)
    try:
        with open(path, encoding="utf-8") as f:
            saved = json.load(f)
        if not isinstance(saved, dict):
            return defaults
        return {**defaults, **saved}
    except FileNotFoundError:
        return defaults
    except (json.JSONDecodeError, OSError):
        return defaults


def save_spray_mode(paths_dir: str, name: str, data: dict[str, Any]) -> None:
    """Validate then atomically write config dict to sidecar.

    Raises ValueError if validation fails (caller should convert to HTTP 422).
    """
    validate_spray_configuration(data)
    path = _sidecar_path(paths_dir, name)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)


def delete_spray_mode(paths_dir: str, name: str) -> bool:
    """Remove sidecar. Returns True if the file existed and was removed."""
    try:
        os.remove(_sidecar_path(paths_dir, name))
        return True
    except FileNotFoundError:
        return False


def sidecar_exists(paths_dir: str, name: str) -> bool:
    """True if a sidecar has been written for this path."""
    return os.path.isfile(_sidecar_path(paths_dir, name))
