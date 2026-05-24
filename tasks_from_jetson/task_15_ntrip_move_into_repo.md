# Task 15 — ntrip_rtcm_node.py Lives Outside the Repo

**Priority:** LOW (housekeeping, enables all other fixes)
**File:** `~/ntrip_rtcm_node.py`

---

## Problem

`ntrip_rtcm_node.py` lives at `~/ntrip_rtcm_node.py`, outside the `PX4_DXP` git repo. It is referenced by path from `px4_start_service.sh` (line 155) and the systemd service's `ReadWritePaths`. It is not version-controlled.

This means:
- Fixes made to `ntrip_rtcm_node.py` are not in git — no history, no rollback
- Tasks 02, 03, 09, 10, 11 all modify this file — they can't be tracked in the repo as-is
- If the Jetson is re-flashed or `/home/flash` is wiped, the file is lost
- The laptop cannot review or pull changes to this file

## Why It Was Left Outside

The `PX4_DXP/` repo was set up after `ntrip_rtcm_node.py` already existed in `~/`. It's cleaner to move it into the repo when restructuring for Phase 2.

## Required Fix (do not apply — analysis only)

As part of Phase 2 `src/` layout creation:

1. Move to `~/PX4_DXP/src/ntrip_rtcm_node.py`
2. Update `px4_start_service.sh` line 155:
   ```bash
   python3 /home/flash/PX4_DXP/src/ntrip_rtcm_node.py
   ```
3. Add `PX4_DXP/src/` to `.gitignore` exclusions if needed (it should be tracked, not ignored)
4. Commit the file into git with history note that it was previously untracked

This should be done in the same commit that implements Task 03 (credentials to env vars), so the credentials are never committed to git even momentarily.

---

**Depends on:** Task 03 (credentials must be removed before the file can be committed)
**Blocks:** Tasks 02, 09, 10, 11 (all modify `ntrip_rtcm_node.py` — easier to do after it's in the repo)
