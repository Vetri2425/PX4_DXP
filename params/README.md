# PX4 Parameter Files

Upload these to QGroundControl to configure the FCU.

## Param_with_Roboclaw.params

Full parameter set for CubeOrangePlus running PX4 v1.16.2 with RoboClaw motor driver.

**CRITICAL:** `RBCLW_QPPS_MAX` is currently 0. You MUST set this from Motion Studio autotune before testing:

1. Connect RoboClaw to Motion Studio via USB
2. Run Motor 1 → Automated Tuning → Tune
3. Run Motor 2 → Automated Tuning → Tune
4. Read QPPS_MAX from Motion Studio → set in QGC

### Key safety params already configured:

| Param | Value | Why |
|---|---|---|
| SER_TEL2_BAUD | 115200 | RoboClaw serial on TELEM2 |
| COM_OF_LOSS_T | 0.3 | 12cm coast max at 0.4 m/s |
| COM_OBL_RC_ACT | 5 | Hold mode = safe auto-stop |
| COM_RCL_EXCEPT | 4 | Allow OFFBOARD without RC |
| RO_SPEED_LIM | 0.5 | 25% headroom |
| RO_MAX_THR_SPEED | 0.5 | Matches RO_SPEED_LIM |
| RO_ACCEL_LIM | 0.4 | 1s to cruise |
| RO_DECEL_LIM | 0.8 | 0.5s stop |