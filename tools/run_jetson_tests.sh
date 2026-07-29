#!/usr/bin/env bash
# Run the full src/ pytest suite WITH rclpy (Jetson / ROS2 host only).
#
# Mac cannot run this — rclpy is unavailable there. The reviewer wires up how
# this is invoked on the robot (ssh, systemd oneshot, CI job, etc.).
#
# Usage:
#   ./tools/run_jetson_tests.sh
#   ./tools/run_jetson_tests.sh /tmp/src-rclpy.json
#   RESULT_JSON=/path/out.json ./tools/run_jetson_tests.sh
#
# Exit: pytest's exit code (non-zero on failure). Always writes RESULT_JSON.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

RESULT_JSON="${1:-${RESULT_JSON:-$ROOT/test-results/src-rclpy.json}}"
mkdir -p "$(dirname "$RESULT_JSON")"

PYTHON="${PYTHON:-python3}"

# Source ROS2 BEFORE probing for rclpy. A non-interactive ssh (`ssh host
# './tools/run_jetson_tests.sh'`) gets no login shell, so ROS is not on the
# path and the probe below would fail on the Jetson itself — which is exactly
# how this ran on 2026-07-29. Probe-then-source is the wrong order.
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if ! "$PYTHON" -c "import rclpy" 2>/dev/null; then
  cat >&2 <<EOF
ERROR: rclpy is not importable with: $PYTHON
This runner is for the Jetson (or any host with ROS2 Humble + rclpy).
On Mac use the ignored-15 suite documented in docs/CURSOR_TASK_2026-07-29.md §2.
EOF
  # Still write a result file so callers can parse failure mode.
  "$PYTHON" - "$RESULT_JSON" <<'PY'
import json, sys, time
from pathlib import Path
out = Path(sys.argv[1])
out.write_text(json.dumps({
    "ok": False,
    "exit_code": 127,
    "error": "rclpy not importable",
    "suite": "src/",
    "started_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "finished_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "passed": 0,
    "failed": 0,
    "errors": 0,
    "skipped": 0,
}, indent=2) + "\n")
sys.exit(127)
PY
fi

export PYTHONPATH="${ROOT}/src:${ROOT}:${PYTHONPATH:-}"

STARTED_AT="$("$PYTHON" -c 'import time; print(time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))')"
JUNIT_XML="${RESULT_JSON%.json}.junit.xml"

set +e
"$PYTHON" -m pytest src/ -q --tb=short --junitxml="$JUNIT_XML"
EXIT_CODE=$?
set -e

FINISHED_AT="$("$PYTHON" -c 'import time; print(time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))')"

"$PYTHON" - "$RESULT_JSON" "$JUNIT_XML" "$EXIT_CODE" "$STARTED_AT" "$FINISHED_AT" <<'PY'
"""Summarise junitxml into a small JSON result file."""
from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

out = Path(sys.argv[1])
junit = Path(sys.argv[2])
exit_code = int(sys.argv[3])
started_at = sys.argv[4]
finished_at = sys.argv[5]

passed = failed = errors = skipped = 0
tests = 0
if junit.is_file():
    root = ET.parse(junit).getroot()
    # pytest may emit <testsuites><testsuite …> or a bare <testsuite …>
    suites = root.findall("testsuite") if root.tag == "testsuites" else [root]
    for suite in suites:
        tests += int(suite.attrib.get("tests", 0) or 0)
        failed += int(suite.attrib.get("failures", 0) or 0)
        errors += int(suite.attrib.get("errors", 0) or 0)
        skipped += int(suite.attrib.get("skipped", 0) or 0)
    passed = max(0, tests - failed - errors - skipped)

payload = {
    "ok": exit_code == 0,
    "exit_code": exit_code,
    "suite": "src/",
    "started_at": started_at,
    "finished_at": finished_at,
    "tests": tests,
    "passed": passed,
    "failed": failed,
    "errors": errors,
    "skipped": skipped,
    "junitxml": str(junit),
}
out.write_text(json.dumps(payload, indent=2) + "\n")
print(f"Wrote {out}", file=sys.stderr)
sys.exit(exit_code)
PY
