import rosbags
print(f"Location: {rosbags.__file__}")

import rosbags.rosbag2
print(f"rosbag2 members: {[x for x in dir(rosbags.rosbag2) if not x.startswith('_')]}")

from rosbags.rosbag2 import Reader
print("Reader imported OK")

import rosbags.serde
print(f"serde members: {[x for x in dir(rosbags.serde) if not x.startswith('_')]}")

# Try deserialize
from rosbags.serde import deserialize_cdr
print("deserialize_cdr imported OK")