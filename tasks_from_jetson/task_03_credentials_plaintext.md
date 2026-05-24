# Task 03 — NTRIP Credentials Hardcoded in Plain Text

**Priority:** HIGH
**File:** `~/ntrip_rtcm_node.py`
**Lines:** 6–11

---

## Problem

NTRIP caster credentials are hardcoded as module-level constants:

```python
# ntrip_rtcm_node.py:6-11
NTRIP_HOST   = "caster.emlid.com"
NTRIP_PORT   = 2101
NTRIP_MOUNTPT= "MP23960a"
NTRIP_USER   = "u98264"
NTRIP_PASS   = "998ckc"
RECONNECT_SEC= 5
```

This file lives in `~/` on the Jetson. The PX4_DXP repo already has a `.gitignore`, but `ntrip_rtcm_node.py` is **outside** the repo (`~/`, not `~/PX4_DXP/`). However:

1. The README references this file and its path — it's clearly intended to be part of the project.
2. If this file is ever moved into the repo (the natural next step as Phase 2 formalises the `src/` layout), credentials go to GitHub.
3. The Emlid account `u98264` controls RTK correction access for the rover. A leaked credential means someone else can consume your correction stream quota or lock you out.

## Why It Matters Now

- The repo already tracks a GitHub remote (`Vetri2425/PX4-Autopilot` referenced in README). A future `git add` sweep can accidentally include this file.
- `.gitignore` does not currently exclude `~/ntrip_rtcm_node.py` because the file is outside the repo. Once it moves into `src/`, it has no protection.

## Required Fix (do not apply — analysis only)

Read from environment variables with clear error messages on missing config:

```python
import os

NTRIP_HOST    = os.environ.get("NTRIP_HOST",   "caster.emlid.com")
NTRIP_PORT    = int(os.environ.get("NTRIP_PORT", "2101"))
NTRIP_MOUNTPT = os.environ["NTRIP_MOUNTPT"]   # fail fast if not set
NTRIP_USER    = os.environ["NTRIP_USER"]
NTRIP_PASS    = os.environ["NTRIP_PASS"]
```

Set credentials in the systemd service file under `EnvironmentFile=`:

```ini
# /etc/systemd/system/px4-dxp.service
EnvironmentFile=/etc/px4-dxp/ntrip.env
```

```bash
# /etc/px4-dxp/ntrip.env  (mode 0600, owned root:root)
NTRIP_MOUNTPT=MP23960a
NTRIP_USER=u98264
NTRIP_PASS=998ckc
```

The `.env` file lives outside the repo, is never committed, and is readable only by root (systemd runs as `flash` user but `EnvironmentFile` is read before privilege drop).

Also add to `.gitignore`:
```
*.env
ntrip.env
```

---

**Depends on:** None
**Blocks:** Moving `ntrip_rtcm_node.py` into `src/` (Phase 2 layout)
