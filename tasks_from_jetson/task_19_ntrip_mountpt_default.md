# Task 19 — NTRIP_MOUNTPT Has Account-Specific Hardcoded Default

**Priority:** LOW
**File:** `ntrip_rtcm_node.py`
**Line:** 26

---

## Problem

```python
# ntrip_rtcm_node.py:26
NTRIP_MOUNTPT = os.environ.get("NTRIP_MOUNTPT", "MP23960a")
```

`NTRIP_USER` and `NTRIP_PASS` are required (no default, raise `RuntimeError` if missing). But `NTRIP_MOUNTPT` has a hardcoded default of `"MP23960a"` — a specific mountpoint for the Emlid caster account `u98264`.

This is inconsistent with the credentials pattern and creates two problems:

1. **Silent wrong behavior:** If `NTRIP_MOUNTPT` is not set in the env file, the node silently uses `MP23960a`. If the caster is changed (different provider, different account), the node connects to the wrong mountpoint and RTCM flow fails — but the error message says nothing about a mountpoint mismatch.

2. **Account-specific default in repo:** `MP23960a` is coupled to a specific Emlid account. If someone else clones this repo and doesn't set `NTRIP_MOUNTPT`, they silently target someone else's mountpoint.

## Required Fix (do not apply — analysis only)

Either make it required (like USER/PASS):

```python
_NTRIP_MOUNTPT = os.environ.get("NTRIP_MOUNTPT")
if not _NTRIP_MOUNTPT:
    raise RuntimeError("NTRIP_MOUNTPT environment variable is required (no default)")
```

Or use a clearly generic default and document it:

```python
NTRIP_MOUNTPT = os.environ.get("NTRIP_MOUNTPT", "")
if not NTRIP_MOUNTPT:
    raise RuntimeError("NTRIP_MOUNTPT environment variable is required")
```

Add `NTRIP_MOUNTPT` to the env file template in `deploy.sh` prompt:

```bash
read -rp "  NTRIP_MOUNTPT (e.g. MP23960a): " ntrip_mountpt
printf "NTRIP_USER=%s\nNTRIP_PASS=%s\nNTRIP_MOUNTPT=%s\n" \
    "$ntrip_user" "$ntrip_pass" "$ntrip_mountpt" > "$NTRIP_ENV"
```

---

**Depends on:** Task 18 (deploy.sh credential file creation cleanup)
**Blocks:** Nothing
