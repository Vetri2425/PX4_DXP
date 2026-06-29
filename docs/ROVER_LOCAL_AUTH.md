# Rover Local Password Authentication

The rover backend uses a local operator password only for login and password
changes. The password is stored as a PBKDF2-HMAC-SHA256 hash in
`config/rover_password.json` with mode `0600`. The server never auto-generates
a default password.

Existing `config/rover_token` or `~/.rover_token` deployments are intentionally
not migrated into an operator password. Run the setup steps below once, then use
the printed machine token only for bag auto-record.

## Initial Jetson Setup

Run on the Jetson as `flash`:

```bash
cd ~/PX4_DXP
python3 server/rover_auth_cli.py setup
python3 server/rover_auth_cli.py create-machine-token --name bag-autorecord
```

Copy the printed machine token into:

```bash
install -m 600 /dev/null ~/PX4_DXP/config/bag_autorecord.token
nano ~/PX4_DXP/config/bag_autorecord.token
```

Then deploy/reload service definitions and restart only the affected services:

```bash
./deploy.sh
sudo systemctl restart rover-server
sudo systemctl restart bag-autorecord
```

## Credentials

- Operator login: `POST /api/auth/login` with the human password.
- Operator REST calls: `X-Rover-Token: <operator-session-token>`.
- Socket.IO: connect with `auth: { token: <operator-session-token> }`.
- Logout: `POST /api/auth/logout`.
- Password change: `POST /api/auth/change-password` with current password and
  new password.

Operator session tokens are random, in-memory, expiring, revocable, and stored
server-side only as SHA-256 token identifiers.

## Password Reset

Run locally on the Jetson:

```bash
cd ~/PX4_DXP
python3 server/rover_auth_cli.py reset-password
sudo systemctl restart rover-server
```

Password changes through the API are blocked while a mission is active or
joystick control is active. A successful password change rotates the requester
session token, revokes all other operator sessions, and disconnects their
Socket.IO clients.

## Bag Auto-Record Machine Token

`bag-autorecord` uses a separate read-only machine token from
`config/bag_autorecord.token`. It is restricted to:

- `GET /api/mission/status`
- `GET /api/mission/loaded-path`
- `GET /api/activity`

It cannot access vehicle control, mission control, spray, RTK, joystick,
parameter writes, telemetry snapshots, or password APIs.
