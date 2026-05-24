# Task 18 — deploy.sh Creates NTRIP Env File as Root — flash Cannot Update

**Priority:** LOW
**File:** `deploy.sh`
**Lines:** 77–89

---

## Problem

```bash
# deploy.sh:77-89
sudo mkdir -p "$(dirname "$NTRIP_ENV")"
...
echo "NTRIP_USER=${ntrip_user}" | sudo tee "$NTRIP_ENV" > /dev/null
echo "NTRIP_PASS=${ntrip_pass}" | sudo tee -a "$NTRIP_ENV" > /dev/null
sudo chmod 600 "$NTRIP_ENV"
```

`sudo tee` creates the file owned by `root:root`. `chmod 600` makes it root-readable only.

This works for the service because systemd reads `EnvironmentFile=` as root before dropping to `flash`. But it creates a usability problem:

- If the NTRIP password changes (e.g., Emlid account reset), `flash` cannot update the file:
  ```bash
  echo "NTRIP_PASS=newpass" > ~/.config/ntrip/env  # Permission denied
  ```
- The user must `sudo nano ~/.config/ntrip/env` or re-run `deploy.sh`
- `ls -la ~/.config/ntrip/` shows root-owned files inside the user's `.config/`, which is confusing

## Required Fix (do not apply — analysis only)

Create the env file as `flash:flash`, mode 600. It's still unreadable to other users, and readable by root (for systemd), and by flash (for manual updates):

```bash
mkdir -p "$(dirname "$NTRIP_ENV")"  # no sudo — flash owns ~/.config

printf "NTRIP_USER=%s\nNTRIP_PASS=%s\n" "$ntrip_user" "$ntrip_pass" > "$NTRIP_ENV"
chmod 600 "$NTRIP_ENV"
log "ntrip: env file created at ${NTRIP_ENV} (mode 600, owner flash)"
```

No `sudo` needed. The file ends up `flash:flash`, mode 600. systemd reads it fine because systemd's EnvironmentFile parsing runs as root, which can read any file regardless of permissions.

---

**Depends on:** None
**Blocks:** Nothing
