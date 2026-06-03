#!/usr/bin/env python3
"""
Direct extraction of ROS2 .db3 bag to CSV files using sqlite3 + manual CDR parsing.
This works for ALL topics including mavros_msgs that aren't in rosbags' built-in store.
"""

import sqlite3
import struct
import csv
import os
from collections import OrderedDict

DB_PATH = r"D:\Vetri\3WD_GCS\bags\arc_fix_20_20260602_151220\arc_fix_20_20260602_151220_0.db3"
OUTPUT_DIR = r"D:\Vetri\3WD_GCS\bags\arc_fix_20_20260602_151220\csv_output"

# Topic ID mapping from the bag
TOPIC_NAMES = {
    1: "diagnostics",
    2: "mavros_local_position_pose",
    3: "mavros_state",
    4: "mavros_time_reference",
    5: "path",
    6: "rpp_debug",
    7: "mavros_setpoint_raw_local",
    8: "rpp_velocity_ned",
    9: "rpp_yaw_rate_body",
}

# ============================================================
# CDR primitive readers (ROS2 Little Endian)
# ============================================================

def read_uint8(data, offset):
    return struct.unpack_from('<B', data, offset)[0], offset + 1

def read_uint16(data, offset):
    return struct.unpack_from('<H', data, offset)[0], offset + 2

def read_uint32(data, offset):
    return struct.unpack_from('<I', data, offset)[0], offset + 4

def read_uint64(data, offset):
    return struct.unpack_from('<Q', data, offset)[0], offset + 8

def read_int8(data, offset):
    return struct.unpack_from('<b', data, offset)[0], offset + 1

def read_int16(data, offset):
    return struct.unpack_from('<h', data, offset)[0], offset + 2

def read_int32(data, offset):
    return struct.unpack_from('<i', data, offset)[0], offset + 4

def read_int64(data, offset):
    return struct.unpack_from('<q', data, offset)[0], offset + 8

def read_float32(data, offset):
    return struct.unpack_from('<f', data, offset)[0], offset + 4

def read_float64(data, offset):
    return struct.unpack_from('<d', data, offset)[0], offset + 8

def read_bool(data, offset):
    v, offset = read_uint8(data, offset)
    return bool(v), offset

def align4(offset):
    return (offset + 3) & ~3

def read_string(data, offset):
    """ROS2 CDR string: 4-byte length + UTF-8 data, 4-byte aligned."""
    length, offset = read_uint32(data, offset)
    s = data[offset:offset + length].decode('utf-8', errors='replace')
    offset += length
    offset = align4(offset)
    return s, offset

def skip_cdr_header(data, offset):
    """Skip the CDR encapsulation header (4 bytes for CDR_LE)."""
    # CDR LE header: 2 bytes (encoding), 2 bytes (options)
    return offset + 4

# ============================================================
# Topic-specific parsers (raw CDR binary -> flat OrderedDict)
# ============================================================

def parse_Float32(data):
    """std_msgs/msg/Float32"""
    off = skip_cdr_header(data, 0)
    val, off = read_float32(data, off)
    return OrderedDict([("data", val)])

def parse_Vector3Stamped(data):
    """geometry_msgs/msg/Vector3Stamped"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    # header.stamp
    sec, off = read_int32(data, off)
    nanosec, off = read_uint32(data, off)
    result["header_stamp_sec"] = sec
    result["header_stamp_nanosec"] = nanosec
    # header.frame_id
    frame_id, off = read_string(data, off)
    result["frame_id"] = frame_id
    # vector
    vx, off = read_float64(data, off)
    vy, off = read_float64(data, off)
    vz, off = read_float64(data, off)
    result["vector_x"] = vx
    result["vector_y"] = vy
    result["vector_z"] = vz
    return result

def parse_PoseStamped(data):
    """geometry_msgs/msg/PoseStamped"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    sec, off = read_int32(data, off)
    nanosec, off = read_uint32(data, off)
    result["header_stamp_sec"] = sec
    result["header_stamp_nanosec"] = nanosec
    result["frame_id"], off = read_string(data, off)
    result["position_x"], off = read_float64(data, off)
    result["position_y"], off = read_float64(data, off)
    result["position_z"], off = read_float64(data, off)
    result["orientation_x"], off = read_float64(data, off)
    result["orientation_y"], off = read_float64(data, off)
    result["orientation_z"], off = read_float64(data, off)
    result["orientation_w"], off = read_float64(data, off)
    return result

def parse_State(data):
    """mavros_msgs/msg/State"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    result["connected"], off = read_bool(data, off)
    result["armed"], off = read_bool(data, off)
    result["guided"], off = read_bool(data, off)
    result["manual_input"], off = read_bool(data, off)
    result["mode"], off = read_string(data, off)
    result["system_status"], off = read_uint8(data, off)
    return result

def parse_TimeReference(data):
    """sensor_msgs/msg/TimeReference"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    sec, off = read_int32(data, off)
    nanosec, off = read_uint32(data, off)
    result["header_stamp_sec"] = sec
    result["header_stamp_nanosec"] = nanosec
    result["frame_id"], off = read_string(data, off)
    ref_sec, off = read_int32(data, off)
    ref_nanosec, off = read_uint32(data, off)
    result["time_ref_sec"] = ref_sec
    result["time_ref_nanosec"] = ref_nanosec
    result["source"], off = read_string(data, off)
    return result

def parse_Path(data):
    """nav_msgs/msg/Path"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    sec, off = read_int32(data, off)
    nanosec, off = read_uint32(data, off)
    result["header_stamp_sec"] = sec
    result["header_stamp_nanosec"] = nanosec
    result["frame_id"], off = read_string(data, off)
    poses_len, off = read_uint32(data, off)
    result["poses_count"] = poses_len
    for i in range(min(poses_len, 3)):
        psec, off = read_int32(data, off)
        pnsec, off = read_uint32(data, off)
        result[f"pose_{i}_header_stamp_sec"] = psec
        result[f"pose_{i}_header_stamp_nanosec"] = pnsec
        pfid, off = read_string(data, off)
        result[f"pose_{i}_frame_id"] = pfid
        result[f"pose_{i}_position_x"], off = read_float64(data, off)
        result[f"pose_{i}_position_y"], off = read_float64(data, off)
        result[f"pose_{i}_position_z"], off = read_float64(data, off)
        result[f"pose_{i}_orientation_x"], off = read_float64(data, off)
        result[f"pose_{i}_orientation_y"], off = read_float64(data, off)
        result[f"pose_{i}_orientation_z"], off = read_float64(data, off)
        result[f"pose_{i}_orientation_w"], off = read_float64(data, off)
    return result

def parse_Float32MultiArray(data):
    """std_msgs/msg/Float32MultiArray"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    # layout: dim[] + data_offset
    dim_len, off = read_uint32(data, off)
    result["layout_dim_count"] = dim_len
    for i in range(dim_len):
        label, off = read_string(data, off)
        result[f"layout_dim_{i}_label"] = label
        size, off = read_uint32(data, off)
        result[f"layout_dim_{i}_size"] = size
        stride, off = read_uint32(data, off)
        result[f"layout_dim_{i}_stride"] = stride
    data_offset, off = read_uint32(data, off)
    result["layout_data_offset"] = data_offset
    # data[]
    data_len, off = read_uint32(data, off)
    result["data_count"] = data_len
    for i in range(data_len):
        result[f"data_{i}"], off = read_float32(data, off)
    return result

def parse_PositionTarget(data):
    """mavros_msgs/msg/PositionTarget"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    sec, off = read_int32(data, off)
    nanosec, off = read_uint32(data, off)
    result["header_stamp_sec"] = sec
    result["header_stamp_nanosec"] = nanosec
    result["frame_id"], off = read_string(data, off)
    result["coordinate_frame"], off = read_uint16(data, off)
    result["type_mask"], off = read_uint32(data, off)
    result["position_x"], off = read_float64(data, off)
    result["position_y"], off = read_float64(data, off)
    result["position_z"], off = read_float64(data, off)
    result["velocity_x"], off = read_float64(data, off)
    result["velocity_y"], off = read_float64(data, off)
    result["velocity_z"], off = read_float64(data, off)
    result["acceleration_or_force_x"], off = read_float64(data, off)
    result["acceleration_or_force_y"], off = read_float64(data, off)
    result["acceleration_or_force_z"], off = read_float64(data, off)
    result["yaw"], off = read_float64(data, off)
    result["yaw_rate"], off = read_float64(data, off)
    return result

def parse_DiagnosticArray(data):
    """diagnostic_msgs/msg/DiagnosticArray"""
    off = skip_cdr_header(data, 0)
    result = OrderedDict()
    sec, off = read_int32(data, off)
    nanosec, off = read_uint32(data, off)
    result["header_stamp_sec"] = sec
    result["header_stamp_nanosec"] = nanosec
    result["frame_id"], off = read_string(data, off)
    status_len, off = read_uint32(data, off)
    result["status_count"] = status_len
    for i in range(status_len):
        level, off = read_uint8(data, off)
        off = align4(off)  # align for string field after uint8
        result[f"status_{i}_level"] = level
        name, off = read_string(data, off)
        result[f"status_{i}_name"] = name
        message, off = read_string(data, off)
        result[f"status_{i}_message"] = message
        hw_id, off = read_string(data, off)
        result[f"status_{i}_hardware_id"] = hw_id
        # values[] inside status
        values_len, off = read_uint32(data, off)
        result[f"status_{i}_values_count"] = values_len
        for j in range(min(values_len, 10)):
            key, off = read_string(data, off)
            result[f"status_{i}_values_{j}_key"] = key
            val, off = read_string(data, off)
            result[f"status_{i}_values_{j}_value"] = val
    return result


PARSERS = {
    1: parse_DiagnosticArray,
    2: parse_PoseStamped,
    3: parse_State,
    4: parse_TimeReference,
    5: parse_Path,
    6: parse_Float32MultiArray,
    7: parse_PositionTarget,
    8: parse_Vector3Stamped,
    9: parse_Float32,
}


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    cur = conn.cursor()

    for topic_id, parser in PARSERS.items():
        topic_name = TOPIC_NAMES[topic_id]
        csv_path = os.path.join(OUTPUT_DIR, f"{topic_name}.csv")

        cur.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC",
            (topic_id,)
        )
        rows = cur.fetchall()
        if not rows:
            print(f"  {topic_name} (id={topic_id}): No messages")
            continue

        # Parse first message to get fieldnames
        first_parsed = parser(rows[0]["data"])
        fieldnames = ["timestamp"] + list(first_parsed.keys())

        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            success = 0
            fail = 0
            for row in rows:
                try:
                    parsed = parser(row["data"])
                    parsed["timestamp"] = row["timestamp"]
                    writer.writerow(parsed)
                    success += 1
                except Exception as e:
                    fail += 1
                    if fail <= 2:
                        print(f"  {topic_name}: error at ts={row['timestamp']}: {e}")

        print(f"  {topic_name}: {success} rows, {fail} errors, {len(fieldnames)} columns -> {csv_path}")

    conn.close()

    print(f"\nDone! Output directory: {OUTPUT_DIR}")
    for f in sorted(os.listdir(OUTPUT_DIR)):
        fpath = os.path.join(OUTPUT_DIR, f)
        print(f"  {f}: {os.path.getsize(fpath):,} bytes")


if __name__ == "__main__":
    main()