# RTK and NTRIP operations

## Profile setup

The authenticated tablet RTK settings page manages named caster profiles on
the Jetson. The backend stores them in the gitignored
`config/ntrip_profiles.json` registry with mode `0600`. Passwords are
write-only through the API: the tablet may add or replace one, but the backend
never returns it and the tablet does not persist it.

Creating or editing a profile does not interrupt the current correction
stream. **Set default** selects the profile used the next time `rover-server`
starts. Runtime profile activation is a separate operation and is not part of
the profile-management phase.

All profile routes require `X-Rover-Token`. That token authenticates the
operator; it does not encrypt plain HTTP. Use the isolated rover network for a
controlled deployment and HTTPS or a secure tunnel on untrusted networks.

## Legacy one-time migration

```bash
cd ~/PX4_DXP
cp config/ntrip.env.example config/ntrip.env
chmod 600 config/ntrip.env
nano config/ntrip.env
```

When no profile registry exists, `rover-server` imports `config/ntrip.env` once
as the `Migrated Default` profile. The legacy file is retained for rollback.
If it is malformed, the backend creates an empty, tablet-repairable registry
and reports a sanitized migration warning.

The selected default profile autostarts whenever `rover-server` starts. An
unexpected NTRIP child exit is restarted with bounded exponential backoff
without restarting MAVROS, RPP, or the QGC bridge.
`ROVER_NTRIP_AUTOSTART=0` is the explicit bench/LoRa override.

## Before every field run

```bash
curl -H "X-Rover-Token: $ROVER_TOKEN" http://localhost:5001/api/rtk/status
curl -H "X-Rover-Token: $ROVER_TOKEN" http://localhost:5001/api/rtk/profiles
ros2 topic echo /mavros/gpsstatus/gps1/raw --once
```

The API must report:

- `desired_mode: "ntrip"`
- `running: true`
- `healthy: true`
- `last_frame_age_s` below 10 seconds and continuing to update

Outdoors, GPSRAW must report `fix_type: 6` and a non-zero `h_acc` no greater
than 100 mm. Both drive and AUTO spray use a 0.5-second GPSRAW freshness
gate. Both stop immediately on degradation and require one continuous second
of good RTK before resuming.

## Fault checks

```bash
journalctl -u rover-server.service -n 100 --no-pager | grep -i ntrip
curl -H "X-Rover-Token: $ROVER_TOKEN" http://localhost:5001/api/rtk/status
ros2 topic hz /mavros/gpsstatus/gps1/raw
```

- `desired_mode=ntrip`, `mode=idle`, `source_state=restarting`: child exited;
  the supervisor is retrying. Inspect `last_error`.
- `running=true`, `healthy=false`: process exists but no current correction
  stream. Check caster/network/GGA and frame age.
- `h_acc: 0`: receiver accuracy is unknown, so production driving and AUTO
  spray remain blocked. Fix the receiver/plugin reporting before field use.
- For an intentional bench only, the explicit escape hatches are
  `rtk_require_accuracy:=false` and `spray_require_accuracy:=false`.
