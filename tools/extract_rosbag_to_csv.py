#!/usr/bin/env python3
"""Extract ROS2 .db3 bag to CSV using the rosbags library (proper deserialization)."""

from rosbags.rosbag2 import Reader
from rosbags.typesys import get_typestore, Stores
from pathlib import Path
import csv
import os

BAG_PATH = r"D:\Vetri\3WD_GCS\bags\arc_fix_20_20260602_151220"
OUTPUT_DIR = r"D:\Vetri\3WD_GCS\bags\arc_fix_20_20260602_151220\csv_output"


def flatten_msg(msg, prefix=""):
    """Recursively flatten a ROS message into a flat dict."""
    result = {}

    if isinstance(msg, bytes):
        result[prefix.rstrip("_")] = msg.hex()
        return result

    if isinstance(msg, (list, tuple)):
        for i, item in enumerate(msg):
            sub = flatten_msg(item, f"{prefix}{i}_")
            for k, v in sub.items():
                result[k] = v
        return result

    if isinstance(msg, (bool, int, float, str)):
        result[prefix.rstrip("_")] = msg
        return result

    if msg is None:
        result[prefix.rstrip("_")] = None
        return result

    # Try as a ROS message object with __slots__
    for field in dir(msg):
        if field.startswith("_"):
            continue
        val = getattr(msg, field)
        sub_prefix = f"{prefix}{field}_"
        if isinstance(val, (list, tuple)):
            sub = flatten_msg(val, sub_prefix)
            for k, v in sub.items():
                result[k] = v
        elif hasattr(val, "__slots__") or hasattr(val, "__dict__"):
            sub = flatten_msg(val, sub_prefix)
            for k, v in sub.items():
                result[k] = v
        elif isinstance(val, bytes):
            result[sub_prefix.rstrip("_")] = val.hex()
        else:
            result[sub_prefix.rstrip("_")] = val

    return result


def main():
    path = Path(BAG_PATH)
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Load the ROS2 Humble typestore
    print("Loading typestore...")
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Open reader
    with Reader(path) as reader:
        print(f"Found {reader.message_count} total messages across {len(reader.connections)} topics:")
        for conn in reader.connections:
            print(f"  [{conn.id}] {conn.topic} ({conn.msgtype})")

        # Build topic -> connection map
        topic_conns = {conn.topic: conn for conn in reader.connections}

        # Collect messages per topic
        topic_msgs = {conn.topic: [] for conn in reader.connections}
        for conn, timestamp, rawdata in reader.messages():
            topic_msgs[conn.topic].append((timestamp, rawdata))

        # Process each topic
        for topic_name, msgs in topic_msgs.items():
            if not msgs:
                print(f"\n  {topic_name}: No messages, skipping")
                continue

            conn = topic_conns[topic_name]
            safe_name = topic_name.replace("/", "_").strip("_")
            csv_path = os.path.join(OUTPUT_DIR, f"{safe_name}.csv")

            print(f"\n  Processing {topic_name} ({len(msgs)} msgs) -> {safe_name}.csv")

            # Get message definition
            msgdef = typestore.get_msgdef(conn.msgtype)

            # Deserialize first message to get field names
            first_msg = typestore.deserialize_cdr(msgs[0][1], conn.msgtype)
            first_flat = flatten_msg(first_msg)
            fieldnames = ["timestamp"] + sorted(first_flat.keys())

            with open(csv_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()

                success = 0
                fail = 0
                for timestamp, rawdata in msgs:
                    try:
                        msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                        row = flatten_msg(msg)
                        row["timestamp"] = timestamp
                        writer.writerow(row)
                        success += 1
                    except Exception as e:
                        fail += 1
                        if fail <= 3:
                            print(f"    Error at ts={timestamp}: {e}")

            print(f"    Wrote {success} rows, {fail} errors, {len(fieldnames)} columns")

    print(f"\nDone! CSV files written to: {OUTPUT_DIR}")
    for fname in sorted(os.listdir(OUTPUT_DIR)):
        fpath = os.path.join(OUTPUT_DIR, fname)
        size = os.path.getsize(fpath)
        print(f"  {fname}: {size:,} bytes")


if __name__ == "__main__":
    main()