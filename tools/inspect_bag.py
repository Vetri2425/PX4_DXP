from rosbags.rosbag2 import Reader
from pathlib import Path

path = Path(r'D:\Vetri\3WD_GCS\bags\arc_fix_20_20260602_151220')
with Reader(path) as reader:
    for conn in reader.connections:
        print(f'=== {conn.topic} ===')
        print(f'  msgtype: {conn.msgtype}')
        # Try to get a message for this topic
        for c, ts, raw in reader.messages():
            if c.topic == conn.topic:
                print(f'  raw len: {len(raw)}, hex start: {raw[:40].hex()}')
                break