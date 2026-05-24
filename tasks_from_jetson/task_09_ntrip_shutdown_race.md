# Task 09 — NTRIP Node Publisher Use-After-Free on Shutdown

**Priority:** MEDIUM
**File:** `~/ntrip_rtcm_node.py`
**Lines:** 86–94

---

## Problem

The `_run()` network thread and `main()` can race during shutdown:

```python
# ntrip_rtcm_node.py:62-80 (_run thread)
def _run(self):
    while rclpy.ok():          # (A) checks rclpy state
        ...
        self.pub.publish(msg)  # (C) uses publisher

# ntrip_rtcm_node.py:86-94 (main thread)
def main():
    rclpy.init()
    node = NtripNode()
    try:
        rclpy.spin(node)       # blocks here
    except KeyboardInterrupt:
        pass
    node.destroy_node()        # (B) destroys publisher handle
    rclpy.shutdown()
```

Race sequence:
1. `rclpy.spin()` returns (SIGTERM received)
2. `rclpy.ok()` becomes `False` — `_run` loop condition is now `False`
3. But `_run` is mid-execution between check (A) and publish (C)
4. `destroy_node()` at (B) runs, freeing the publisher handle
5. `_run` reaches (C) and calls `self.pub.publish()` on a destroyed publisher → exception or segfault in `rclpy` C extension

This is a classic TOCTOU race: the thread checks `rclpy.ok()`, passes the check, then the publisher is destroyed before the thread uses it.

## Why It Appears Harmless in Practice

`daemon=True` on the thread means the OS kills it when the main thread exits. On a fast shutdown, the main thread reaches `sys.exit()` before the daemon thread publishes again. The race window is small (~milliseconds). But:
- On a loaded system (Phase 2 with multiple nodes), the OS scheduler may context-switch between (A) and (C)
- The resulting traceback floods the journal with a misleading error that looks like an RTCM publishing failure when it's actually a shutdown artifact

## Required Fix (do not apply — analysis only)

Add a `threading.Event` to signal the thread to stop cleanly:

```python
class NtripNode(Node):
    def __init__(self):
        super().__init__("ntrip_rtcm_node")
        self._stop = threading.Event()
        self.pub = self.create_publisher(RTCM, "/mavros/gps_rtk/send_rtcm", 10)
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set() and rclpy.ok():
            ...

def main():
    ...
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()        # signal thread to exit cleanly
    time.sleep(0.1)   # let thread finish current iteration
    node.destroy_node()
    rclpy.shutdown()
```

---

**Depends on:** None
**Blocks:** Nothing directly, but clean shutdown matters for the watchdog restart loop (Task 01) — a crash on shutdown looks like a runtime failure
