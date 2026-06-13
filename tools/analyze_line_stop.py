#!/usr/bin/env python3
"""
Analyze line_stop rosbag: xtrack error, velocity, heading, waypoint events.
Reads raw SQLite3 DB + manually deserializes CDR for Float32MultiArray and basic types.
"""
import sqlite3
import struct
import math
import json
from pathlib import Path
from collections import OrderedDict

BAG = Path("/Users/dyx_a1/Vetri/PX4_DXP/bags/temp/line_stop_20260611_194002/line_stop_20260611_194002_0.db3")

# ── CDR Helpers (ROS2 CDR little-endian) ──────────────────────────────

def cdr_read_uint32(data, offset):
    return struct.unpack_from('<I', data, offset)[0], offset + 4

def cdr_read_float32(data, offset):
    return struct.unpack_from('<f', data, offset)[0], offset + 4

def cdr_read_float64(data, offset):
    return struct.unpack_from('<d', data, offset)[0], offset + 8

def cdr_read_int16(data, offset):
    return struct.unpack_from('<h', data, offset)[0], offset + 2

def cdr_read_uint16(data, offset):
    return struct.unpack_from('<H', data, offset)[0], offset + 2

def cdr_read_int8(data, offset):
    return struct.unpack_from('<b', data, offset)[0], offset + 1

def cdr_read_uint8(data, offset):
    return struct.unpack_from('<B', data, offset)[0], offset + 1

def cdr_read_bool(data, offset):
    v, offset = cdr_read_uint8(data, offset)
    return bool(v), offset

def cdr_read_string(data, offset):
    """Read CDR string: uint32 length + UTF-8 chars (no null term)."""
    length, offset = cdr_read_uint32(data, offset)
    s = data[offset:offset+length].decode('utf-8', errors='replace')
    # Align to 4 bytes
    aligned = (length + 3) & ~3
    offset += aligned
    return s, offset

# ── Float32MultiArray Deserializer ────────────────────────────────────

def deserialize_float32_multiarray(data):
    """Parse CDR for std_msgs/Float32MultiArray."""
    offset = 4  # Skip CDR encapsulation header (uint16 encoding + uint16 options)
    # MultiArrayLayout layout
    #   dim[] (sequence): uint32 size + for each: string label + uint32 size + uint32 stride
    dim_count, offset = cdr_read_uint32(data, offset)
    dims = []
    for _ in range(dim_count):
        label, offset = cdr_read_string(data, offset)
        sz, offset = cdr_read_uint32(data, offset)
        stride, offset = cdr_read_uint32(data, offset)
        dims.append({'label': label, 'size': sz, 'stride': stride})
    data_offset, offset = cdr_read_uint32(data, offset)
    # float32[] data
    count, offset = cdr_read_uint32(data, offset)
    values = []
    for _ in range(count):
        v, offset = cdr_read_float32(data, offset)
        values.append(v)
    return values

# ── Vector3Stamped Deserializer ───────────────────────────────────────

def deserialize_vector3_stamped(data):
    """Parse geometry_msgs/Vector3Stamped: std_msgs/Header + Vector3"""
    offset = 0  # No CDR encapsulation header for this topic
    # Header: uint32 seq, Time stamp (uint32 secs + uint32 nsecs), string frame_id
    seq, offset = cdr_read_uint32(data, offset)
    sec, offset = cdr_read_uint32(data, offset)
    nsec, offset = cdr_read_uint32(data, offset)
    frame_id, offset = cdr_read_string(data, offset)
    # Vector3: float64 x, y, z
    x, offset = cdr_read_float64(data, offset)
    y, offset = cdr_read_float64(data, offset)
    z, offset = cdr_read_float64(data, offset)
    return {'seq': seq, 'sec': sec, 'nsec': nsec, 'frame_id': frame_id,
            'x': x, 'y': y, 'z': z}

# ── PoseStamped Deserializer ──────────────────────────────────────────

def deserialize_pose_stamped(data):
    """Parse geometry_msgs/PoseStamped: Header + Pose (Point + Quaternion)"""
    offset = 0
    seq, offset = cdr_read_uint32(data, offset)
    sec, offset = cdr_read_uint32(data, offset)
    nsec, offset = cdr_read_uint32(data, offset)
    frame_id, offset = cdr_read_string(data, offset)
    # Point: float64 x, y, z
    px, offset = cdr_read_float64(data, offset)
    py, offset = cdr_read_float64(data, offset)
    pz, offset = cdr_read_float64(data, offset)
    # Quaternion: float64 x, y, z, w
    ox, offset = cdr_read_float64(data, offset)
    oy, offset = cdr_read_float64(data, offset)
    oz, offset = cdr_read_float64(data, offset)
    ow, offset = cdr_read_float64(data, offset)
    # Convert quaternion to yaw (ENU frame from MAVROS)
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    yaw_enu = math.atan2(siny_cosp, cosy_cosp)
    return {'sec': sec, 'nsec': nsec,
            'pos_x': px, 'pos_y': py, 'pos_z': pz,
            'qx': ox, 'qy': oy, 'qz': oz, 'qw': ow,
            'yaw_enu': yaw_enu}

# ── PositionTarget Deserializer (mavros_msgs/PositionTarget) ──────────

def deserialize_position_target(data):
    """Parse mavros_msgs/PositionTarget (simplified - key fields)."""
    offset = 4  # Skip CDR encapsulation header
    # Header
    seq, offset = cdr_read_uint32(data, offset)
    sec, offset = cdr_read_uint32(data, offset)
    nsec, offset = cdr_read_uint32(data, offset)
    frame_id, offset = cdr_read_string(data, offset)
    # uint8 coordinate_frame
    coord_frame, offset = cdr_read_uint8(data, offset)
    # uint16 type_mask
    type_mask, offset = cdr_read_uint16(data, offset)
    # float64[3] position
    px, offset = cdr_read_float64(data, offset)
    py, offset = cdr_read_float64(data, offset)
    pz, offset = cdr_read_float64(data, offset)
    # float64[3] velocity
    vx, offset = cdr_read_float64(data, offset)
    vy, offset = cdr_read_float64(data, offset)
    vz, offset = cdr_read_float64(data, offset)
    # float64[3] acceleration_or_force
    afx, offset = cdr_read_float64(data, offset)
    afy, offset = cdr_read_float64(data, offset)
    afz, offset = cdr_read_float64(data, offset)
    # float64 yaw, float64 yaw_rate
    yaw, offset = cdr_read_float64(data, offset)
    yaw_rate, offset = cdr_read_float64(data, offset)
    return {'sec': sec, 'nsec': nsec, 'coord_frame': coord_frame,
            'type_mask': type_mask, 'pos_x': px, 'pos_y': py, 'pos_z': pz,
            'vel_x': vx, 'vel_y': vy, 'vel_z': vz,
            'yaw': yaw, 'yaw_rate': yaw_rate}

# ── State Deserializer (mavros_msgs/State) ────────────────────────────

def deserialize_mavros_state(data):
    """Parse mavros_msgs/State (simplified)."""
    offset = 4  # Skip CDR encapsulation header
    # Header (we skip sometimes)
    seq, offset = cdr_read_uint32(data, offset)
    sec, offset = cdr_read_uint32(data, offset)
    nsec, offset = cdr_read_uint32(data, offset)
    frame_id, offset = cdr_read_string(data, offset)
    # string connected, armed, guided, mode, system_status (simplified)
    connected, offset = cdr_read_string(data, offset)
    armed, offset = cdr_read_string(data, offset)
    guided, offset = cdr_read_string(data, offset)
    mode, offset = cdr_read_string(data, offset)
    sys_status, offset = cdr_read_string(data, offset)
    return {'sec': sec, 'nsec': nsec,
            'connected': connected,
            'armed': armed,
            'guided': guided,
            'mode': mode,
            'system_status': sys_status}

# ── Path Deserializer (nav_msgs/Path - for waypoints) ─────────────────

def deserialize_path(data):
    """Parse nav_msgs/Path: Header + PoseStamped[] poses."""
    offset = 4  # Skip CDR encapsulation header
    seq, offset = cdr_read_uint32(data, offset)
    sec, offset = cdr_read_uint32(data, offset)
    nsec, offset = cdr_read_uint32(data, offset)
    frame_id, offset = cdr_read_string(data, offset)
    # PoseStamped[] poses
    pose_count, offset = cdr_read_uint32(data, offset)
    poses = []
    for _ in range(pose_count):
        # Inner header
        pseq, offset = cdr_read_uint32(data, offset)
        psec, offset = cdr_read_uint32(data, offset)
        pnsec, offset = cdr_read_uint32(data, offset)
        pframe_id, offset = cdr_read_string(data, offset)
        # Point
        px, offset = cdr_read_float64(data, offset)
        py, offset = cdr_read_float64(data, offset)
        pz, offset = cdr_read_float64(data, offset)
        # Quaternion
        ox, offset = cdr_read_float64(data, offset)
        oy, offset = cdr_read_float64(data, offset)
        oz, offset = cdr_read_float64(data, offset)
        ow, offset = cdr_read_float64(data, offset)
        siny_cosp = 2.0 * (ow * oz + ox * oy)
        cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        poses.append({'x': px, 'y': py, 'z': pz, 'yaw': yaw})
    return {'sec': sec, 'nsec': nsec, 'poses': poses}


# ── MAIN ──────────────────────────────────────────────────────────────

def main():
    conn = sqlite3.connect(str(BAG))
    cur = conn.cursor()

    # 1. Get topic mapping
    cur.execute("SELECT id, name, type FROM topics")
    topics = {row[0]: {'name': row[1], 'type': row[2]} for row in cur.fetchall()}
    print("=== TOPICS ===")
    for tid, info in topics.items():
        print(f"  {tid}: {info['name']} ({info['type']})")

    # 2. Fetch all messages with their timestamps
    # messages table: topic_id, timestamp, data
    cur.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")

    # Classify messages
    rpp_debug_msgs = []
    segment_debug_msgs = []
    velocity_msgs = []
    pose_msgs = []
    setpoint_msgs = []
    state_msgs = []
    path_msgs = []

    # Build reverse topic map
    name_to_id = {info['name']: tid for tid, info in topics.items()}

    for topic_id, timestamp, data in cur.fetchall():
        tname = topics[topic_id]['name']
        msg = {
            'timestamp': timestamp,
            'topic_id': topic_id,
            'topic_name': tname
        }

        if tname == '/rpp/debug':
            try:
                vals = deserialize_float32_multiarray(data)
                if len(vals) >= 2:
                    rpp_debug_msgs.append({
                        'timestamp': timestamp,
                        'xtrack': vals[0],        # index 0 = cross-track error
                        'alongtrack': vals[1],    # index 1 = along-track
                        'raw': vals[:10]          # first 10 values as preview
                    })
            except Exception as e:
                pass

        elif tname == '/rpp/segment_debug':
            try:
                vals = deserialize_float32_multiarray(data)
                segment_debug_msgs.append({
                    'timestamp': timestamp,
                    'values': vals[:8],
                    'raw': vals[:10]
                })
            except Exception as e:
                pass

        elif tname == '/rpp/velocity_ned':
            try:
                v = deserialize_vector3_stamped(data)
                speed = math.sqrt(v['x']**2 + v['y']**2 + v['z']**2)
                velocity_msgs.append({
                    'timestamp': timestamp,
                    'v_north': v['x'],
                    'v_east': v['y'],
                    'v_down': v['z'],
                    'speed': speed
                })
            except Exception as e:
                pass

        elif tname == '/mavros/local_position/pose':
            try:
                p = deserialize_pose_stamped(data)
                pose_msgs.append({
                    'timestamp': timestamp,
                    'pos_x': p['pos_x'],
                    'pos_y': p['pos_y'],
                    'pos_z': p['pos_z'],
                    'yaw_enu': p['yaw_enu']
                })
            except Exception as e:
                pass

        elif tname == '/mavros/setpoint_raw/local':
            try:
                sp = deserialize_position_target(data)
                setpoint_msgs.append({
                    'timestamp': timestamp,
                    'pos_x': sp['pos_x'],
                    'pos_y': sp['pos_y'],
                    'vel_x': sp['vel_x'],
                    'vel_y': sp['vel_y'],
                    'yaw': sp['yaw'],
                    'yaw_rate': sp['yaw_rate'],
                    'type_mask': sp['type_mask']
                })
            except Exception as e:
                pass

        elif tname == '/mavros/state':
            try:
                s = deserialize_mavros_state(data)
                state_msgs.append(s)
            except Exception as e:
                pass

        elif tname == '/path':
            try:
                path_data = deserialize_path(data)
                path_msgs.append({
                    'timestamp': timestamp,
                    'pose_count': len(path_data['poses']),
                    'first_pose': path_data['poses'][0] if path_data['poses'] else None,
                    'last_pose': path_data['poses'][-1] if path_data['poses'] else None
                })
            except Exception as e:
                pass

    conn.close()

    # ── Compute analysis ──────────────────────────────────────────────

    print("\n\n============================================================")
    print("  LINE_STOP ANALYSIS REPORT")
    print("============================================================")
    print(f"  Bag duration: ~32.0 seconds")
    print(f"  Topic message counts:")
    print(f"    /rpp/debug:         {len(rpp_debug_msgs)}")
    print(f"    /rpp/segment_debug: {len(segment_debug_msgs)}")
    print(f"    /rpp/velocity_ned:  {len(velocity_msgs)}")
    print(f"    /mavros/local_position/pose: {len(pose_msgs)}")
    print(f"    /mavros/setpoint_raw/local:  {len(setpoint_msgs)}")
    print(f"    /mavros/state:       {len(state_msgs)}")
    print(f"    /path:               {len(path_msgs)}")
    print(f"    (paths published: {len(path_msgs)}x, each contains waypoints)")

    # ── XTRACK ERROR ANALYSIS ──
    print("\n\n─── 1. CROSS-TRACK ERROR (XTRACK) ───")
    if rpp_debug_msgs:
        xtracks = [m['xtrack'] for m in rpp_debug_msgs]
        print(f"    Samples: {len(xtracks)}")
        print(f"    Mean xtrack:   {sum(xtracks)/len(xtracks):.4f} m")
        print(f"    Max xtrack:    {max(xtracks):.4f} m")
        print(f"    Min xtrack:    {min(xtracks):.4f} m")
        print(f"    Std xtrack:    {math.sqrt(sum((x - sum(xtracks)/len(xtracks))**2 for x in xtracks)/len(xtracks)):.4f} m")
        # 95th percentile
        sorted_x = sorted(xtracks)
        p95 = sorted_x[int(len(sorted_x)*0.95)]
        print(f"    95th percentile: {p95:.4f} m")
        # RMS
        rms = math.sqrt(sum(x*x for x in xtracks)/len(xtracks))
        print(f"    RMS xtrack:    {rms:.4f} m")

        # Along-track
        alongs = [m['alongtrack'] for m in rpp_debug_msgs]
        print(f"\n    Along-track distance:")
        print(f"      Start:  {alongs[0]:.2f} m")
        print(f"      End:    {alongs[-1]:.2f} m")
        print(f"      Travel: {alongs[-1] - alongs[0]:.2f} m")

        # Xtrack over time segments (start, middle, end)
        n = len(xtracks)
        seg_size = n // 3
        segments = {
            'First 33%': xtracks[:seg_size],
            'Middle 33%': xtracks[seg_size:2*seg_size],
            'Last 33%': xtracks[2*seg_size:],
        }
        print(f"\n    Xtrack by journey phase:")
        for seg_name, seg_data in segments.items():
            if seg_data:
                print(f"      {seg_name}: mean={sum(seg_data)/len(seg_data):.4f}m  max={max(seg_data):.4f}m  min={min(seg_data):.4f}m")

    # ── VELOCITY ANALYSIS ──
    print("\n\n─── 2. SPEED / VELOCITY ANALYSIS (NED) ───")
    if velocity_msgs:
        speeds = [m['speed'] for m in velocity_msgs]
        vx = [m['v_north'] for m in velocity_msgs]
        vy = [m['v_east'] for m in velocity_msgs]

        print(f"    Samples: {len(speeds)}")
        print(f"    Speed (magnitude):")
        print(f"      Mean:   {sum(speeds)/len(speeds):.2f} m/s")
        print(f"      Max:    {max(speeds):.2f} m/s")
        print(f"      Min:    {min(speeds):.2f} m/s")

        # Find NT-start (when motion begins) and NT-stop (when motion ends)
        # NT-start = first time speed exceeds 0.05 m/s
        # NT-stop = last time speed falls below 0.05 m/s
        NT_THRESH = 0.05
        start_idx = None
        stop_idx = None
        for i, s in enumerate(speeds):
            if s > NT_THRESH and start_idx is None:
                start_idx = i
            if s > NT_THRESH:
                stop_idx = i  # keeps updating to last moving sample

        if start_idx is not None and stop_idx is not None:
            t_start = velocity_msgs[start_idx]['timestamp']
            t_stop = velocity_msgs[stop_idx]['timestamp']
            dt_ns = t_stop - t_start
            dt_s = dt_ns / 1e9
            print(f"\n    NT START (first motion > {NT_THRESH} m/s):")
            print(f"      timestamp:  {t_start}")
            print(f"      speed:      {speeds[start_idx]:.3f} m/s")
            print(f"      V_north:    {vx[start_idx]:.3f} m/s")
            print(f"      V_east:     {vy[start_idx]:.3f} m/s")

            print(f"\n    NT STOP (last motion > {NT_THRESH} m/s):")
            print(f"      timestamp:  {t_stop}")
            print(f"      speed:      {speeds[stop_idx]:.3f} m/s")
            print(f"      V_north:    {vx[stop_idx]:.3f} m/s")
            print(f"      V_east:     {vy[stop_idx]:.3f} m/s")

            print(f"\n    NT duration: {dt_s:.2f} s")

            # Speed profile during NT
            nt_speeds = speeds[start_idx:stop_idx+1]
            if nt_speeds:
                print(f"\n    Speed during NT motion ({len(nt_speeds)} samples):")
                print(f"      Mean:      {sum(nt_speeds)/len(nt_speeds):.2f} m/s")
                print(f"      Max:       {max(nt_speeds):.2f} m/s")
                print(f"      Min:       {min(nt_speeds):.2f} m/s")
                # Variation
                range_s = max(nt_speeds) - min(nt_speeds)
                print(f"      Range:     {range_s:.2f} m/s")
                if sum(nt_speeds)/len(nt_speeds) > 0:
                    cv = math.sqrt(sum((s - sum(nt_speeds)/len(nt_speeds))**2 for s in nt_speeds)/len(nt_speeds)) / (sum(nt_speeds)/len(nt_speeds))
                    print(f"      CV:        {cv:.3f} ({cv*100:.1f}%)")

            # Speed at key phases
            if len(nt_speeds) >= 10:
                early = nt_speeds[:len(nt_speeds)//4]
                mid = nt_speeds[len(nt_speeds)//4:3*len(nt_speeds)//4]
                late = nt_speeds[3*len(nt_speeds)//4:]
                print(f"      Early 25%:  mean={sum(early)/len(early):.2f}m/s")
                print(f"      Mid 50%:    mean={sum(mid)/len(mid):.2f}m/s")
                print(f"      Late 25%:   mean={sum(late)/len(late):.2f}m/s")

    # ── HEADING ANALYSIS ──
    print("\n\n─── 3. HEADING / YAW ANALYSIS (ENU frame from pose) ───")
    if pose_msgs:
        yaws = [m['yaw_enu'] for m in pose_msgs]
        yaw_deg = [math.degrees(y) for y in yaws]
        # Unwrap
        unwrapped = []
        prev = yaws[0]
        unwrapped.append(prev)
        for y in yaws[1:]:
            diff = y - prev
            if diff > math.pi:
                y -= 2*math.pi
            elif diff < -math.pi:
                y += 2*math.pi
            unwrapped.append(y)
            prev = y
        unwrapped_deg = [math.degrees(u) for u in unwrapped]
        print(f"    Samples: {len(yaws)}")
        print(f"    Heading (ENU yaw):")
        print(f"      Start:       {math.degrees(yaws[0]):.1f}°")
        print(f"      End:         {math.degrees(yaws[-1]):.1f}°")
        print(f"      Mean:        {sum(yaws)/len(yaws):.4f} rad ({math.degrees(sum(yaws)/len(yaws)):.1f}°)")
        print(f"      Max variation from mean: {max(abs(y - sum(yaws)/len(yaws)) for y in yaws):.4f} rad ({math.degrees(max(abs(y - sum(yaws)/len(yaws)) for y in yaws)):.1f}°)")
        # Variation during NT period
        if start_idx is not None and stop_idx is not None and start_idx < len(yaws) and stop_idx < len(yaws):
            nt_yaws = yaws[start_idx:stop_idx+1]
            # Also try to align by time: find yaw indices within nt time window
            nt_time_start = velocity_msgs[start_idx]['timestamp']
            nt_time_end = velocity_msgs[stop_idx]['timestamp']
            nt_pose_yaws = []
            for p in pose_msgs:
                if nt_time_start <= p['timestamp'] <= nt_time_end:
                    nt_pose_yaws.append(p['yaw_enu'])
            if nt_pose_yaws:
                ny_deg = [math.degrees(y) for y in nt_pose_yaws]
                print(f"\n    Heading during NT motion ({len(nt_pose_yaws)} samples):")
                print(f"      Start:       {ny_deg[0]:.1f}°")
                print(f"      End:         {ny_deg[-1]:.1f}°")
                print(f"      Mean:        {math.degrees(sum(nt_pose_yaws)/len(nt_pose_yaws)):.1f}°")
                print(f"      Max delta:   {max(ny_deg) - min(ny_deg):.1f}°")
                print(f"      Std:         {math.sqrt(sum((y - sum(nt_pose_yaws)/len(nt_pose_yaws))**2 for y in nt_pose_yaws)/len(nt_pose_yaws)):.4f} rad ({math.degrees(math.sqrt(sum((y - sum(nt_pose_yaws)/len(nt_pose_yaws))**2 for y in nt_pose_yaws)/len(nt_pose_yaws))):.2f}°)")

    # ── WAYPOINT / PATH EVENTS ──
    print("\n\n─── 4. PATH / WAYPOINT EVENTS ───")
    print(f"    Number of /path publications: {len(path_msgs)}")
    for i, pm in enumerate(path_msgs):
        print(f"    Path #{i}:")
        print(f"      Timestamp: {pm['timestamp']}")
        print(f"      Poses: {pm['pose_count']}")
        if pm['first_pose']:
            fp = pm['first_pose']
            print(f"      First waypoint: ({fp['x']:.2f}, {fp['y']:.2f}, {fp['z']:.2f})  yaw={math.degrees(fp['yaw']):.1f}°")
        if pm['last_pose']:
            lp = pm['last_pose']
            print(f"      Last waypoint:  ({lp['x']:.2f}, {lp['y']:.2f}, {lp['z']:.2f})  yaw={math.degrees(lp['yaw']):.1f}°")

    # ── MAVROS STATE EVENTS ──
    print("\n\n─── 5. MAVROS STATE EVENTS ───")
    if state_msgs:
        for s in state_msgs:
            print(f"    t={s['sec']}.{s['nsec']:09d}  connected={s['connected']}  armed={s['armed']}  guided={s['guided']}  mode={s['mode']}  status={s['system_status']}")

    # ── COMBINED NT START/STOP WITH XTRACK ──
    print("\n\n─── 6. NT START → NT STOP — FULL BREAKDOWN ───")
    if start_idx is not None and stop_idx is not None:
        nt_time_start = velocity_msgs[start_idx]['timestamp']
        nt_time_end = velocity_msgs[stop_idx]['timestamp']
        dt_ns = nt_time_end - nt_time_start
        dt_s = dt_ns / 1e9
        print(f"    NT window: {nt_time_start} → {nt_time_end}  ({dt_s:.2f}s)")
        print(f"")
        # Xtrack during NT
        nt_xtracks = [m['xtrack'] for m in rpp_debug_msgs if nt_time_start <= m['timestamp'] <= nt_time_end]
        if nt_xtracks:
            print(f"    Xtrack during NT:")
            print(f"      Samples: {len(nt_xtracks)}")
            print(f"      Mean:    {sum(nt_xtracks)/len(nt_xtracks):.4f} m")
            print(f"      Max:     {max(nt_xtracks):.4f} m")
            print(f"      Min:     {min(nt_xtracks):.4f} m")
            sorted_nx = sorted(nt_xtracks)
            print(f"      95%:     {sorted_nx[int(len(sorted_nx)*0.95)]:.4f} m")
            print(f"      99%:     {sorted_nx[int(len(sorted_nx)*0.99)]:.4f} m")
            print(f"      RMS:     {math.sqrt(sum(x*x for x in nt_xtracks)/len(nt_xtracks)):.4f} m")
        # Speed during NT
        nt_speeds = [m['speed'] for m in velocity_msgs if nt_time_start <= m['timestamp'] <= nt_time_end]
        if nt_speeds:
            print(f"\n    Speed during NT:")
            print(f"      Mean:   {sum(nt_speeds)/len(nt_speeds):.2f} m/s")
            print(f"      Max:    {max(nt_speeds):.2f} m/s")
            print(f"      Min:    {min(nt_speeds):.2f} m/s")
            print(f"      Range:  {max(nt_speeds) - min(nt_speeds):.2f} m/s")
            # 0.1s rolling window variation
            if len(nt_speeds) > 20:
                window = 20  # ~0.2s at 100Hz
                max_dv = max(abs(nt_speeds[i] - nt_speeds[i-1]) for i in range(1, len(nt_speeds)))
                print(f"      Max sample-to-sample delta: {max_dv:.2f} m/s")
        # Heading during NT (from pose)
        nt_yaws = [m['yaw_enu'] for m in pose_msgs if nt_time_start <= m['timestamp'] <= nt_time_end]
        if nt_yaws:
            ny_deg = [math.degrees(y) for y in nt_yaws]
            print(f"\n    Heading during NT:")
            print(f"      Samples: {len(nt_yaws)}")
            print(f"      Start:   {ny_deg[0]:.1f}°")
            print(f"      End:     {ny_deg[-1]:.1f}°")
            print(f"      Delta:   {ny_deg[-1] - ny_deg[0]:.1f}°")
            print(f"      Mean:    {sum(ny_deg)/len(ny_deg):.1f}°")
            print(f"      Max peak deviation from mean: {max(abs(d - sum(ny_deg)/len(ny_deg)) for d in ny_deg):.1f}°")
            if len(ny_deg) > 1:
                max_dyaw = max(abs(ny_deg[i] - ny_deg[i-1]) for i in range(1, len(ny_deg)))
                print(f"      Max sample-to-sample delta: {max_dyaw:.2f}°")

    # ── SEGMENT DEBUG FIELDS ──
    print("\n\n─── 7. SEGMENT DEBUG (first & last samples) ───")
    if segment_debug_msgs:
        print(f"    Total samples: {len(segment_debug_msgs)}")
        fm = segment_debug_msgs[0]
        lm = segment_debug_msgs[-1]
        print(f"    First segment debug values: {[f'{v:.4f}' for v in fm['values']]}")
        print(f"    Last  segment debug values: {[f'{v:.4f}' for v in lm['values']]}")
        # Check for NT transitions in segment debug
        print(f"\n    Segment debug fields (typical):")
        print(f"      [0]=segment_idx, [1]=target_speed, [2]=segment_progress,")
        print(f"      [3]=nt_start_idx, [4]=nt_end_idx, [5]=nt_progress,")
        print(f"      [6]=waypoint_switch, [7]=e_stop")
        # Print all unique segment indices
        seg_indices = set()
        for m in segment_debug_msgs:
            if len(m['values']) > 0:
                seg_indices.add(int(m['values'][0]))
        print(f"    Unique segment indices seen: {sorted(seg_indices)}")

        # Flatten segment debug series
        speeds_target = []
        progresses = []
        for m in segment_debug_msgs:
            vals = m['values']
            if len(vals) >= 3:
                speeds_target.append(vals[1])
                progresses.append(vals[2])
        if speeds_target:
            print(f"    Target speed range: [{min(speeds_target):.2f}, {max(speeds_target):.2f}] m/s")
        if progresses:
            print(f"    Segment progress range: [{min(progresses):.2f}, {max(progresses):.2f}]")

    print("\n============================================================")
    print("  END OF REPORT")
    print("============================================================")


if __name__ == '__main__':
    main()