# PX4_DXP — local helpers. Prefer calling tools/ scripts directly.

.PHONY: jetson-tests

# Full src/ suite WITH rclpy. Jetson / ROS2 host only — do not run on Mac.
# Writes test-results/src-rclpy.json (override with RESULT_JSON=...).
jetson-tests:
	./tools/run_jetson_tests.sh
