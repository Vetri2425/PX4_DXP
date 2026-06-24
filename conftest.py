"""Repository-wide pytest import path setup.

Normal local pytest runs should not require ad hoc PYTHONPATH exports.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
for rel in ("src", "server"):
    path = str(ROOT / rel)
    if path not in sys.path:
        sys.path.insert(0, path)
