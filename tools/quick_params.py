#!/usr/bin/env python3
"""One-shot parameter reader — companion RPP params AND PX4/FCU params.

Why this exists: `ros2 param get` pays full node discovery on every invocation
(~2-4 s each), so checking 30 params costs a minute or more. MAVROS2 exposes
every FCU parameter as a ROS parameter on /mavros/param, so both sources answer
the same rcl_interfaces/GetParameters service and the whole set comes back from
a single call on a single spin.

READ ONLY — never calls set_parameters. FCU params are QGC's to change.

  python3 tools/quick_params.py                    # verify the field-day set
  python3 tools/quick_params.py --fcu RO_YAW_P EKF2_GPS_POS_Y
  python3 tools/quick_params.py --rpp mission_speed
  python3 tools/quick_params.py --list-fcu RO_     # enumerate by prefix
"""
import argparse
import sys

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node

RPP_NODE = "/rpp_controller"
FCU_NODE = "/mavros/param"

_FIELDS = {
    ParameterType.PARAMETER_BOOL: "bool_value",
    ParameterType.PARAMETER_INTEGER: "integer_value",
    ParameterType.PARAMETER_DOUBLE: "double_value",
    ParameterType.PARAMETER_STRING: "string_value",
}


def unwrap(pv):
    if pv.type == ParameterType.PARAMETER_NOT_SET:
        return None
    return getattr(pv, _FIELDS.get(pv.type, "string_value"))


def _call(node, cli, names, timeout):
    fut = cli.call_async(GetParameters.Request(names=list(names)))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    return [unwrap(v) for v in fut.result().values]


def fetch(node, target, names, timeout=8.0):
    """One GetParameters call for the whole list.

    Gotcha this guards against: rclpy's GetParameters handler returns an EMPTY
    values array if ANY requested name is undeclared — so a single typo blanks
    the entire batch and looks like the node is down. On a short response, fall
    back to one call per name so the typo is localised instead of hiding
    everything else.
    """
    cli = node.create_client(GetParameters, target + "/get_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None, "service " + target + "/get_parameters not available"
    names = list(names)
    vals = _call(node, cli, names, timeout)
    if vals is None:
        return None, target + " did not answer within " + str(timeout) + "s"
    if len(vals) == len(names):
        return vals, None
    out = []
    for n in names:
        one = _call(node, cli, [n], timeout)
        out.append(one[0] if one and len(one) == 1 else None)
    return out, None


def enumerate_params(node, target, prefix="", timeout=8.0):
    cli = node.create_client(ListParameters, target + "/list_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = ListParameters.Request()
    req.depth = 0
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    return sorted(n for n in fut.result().result.names if n.startswith(prefix))


# (name, expected) — expected None means "report only, no assertion"
RPP_EXPECT = [
    ("transit_merge_max_len_m", 2.0),
    ("transit_runout_goal_tolerance_m", 0.10),
    ("transit_runout_min_speed_m_s", 0.10),
    ("spray_entry_max_heading_deg", 5.0),
    ("spray_entry_release_travel_m", 0.6),
    ("spray_heading_cut_deg", 30.0),
    ("mission_speed", 0.70),
    ("ekf_jump_threshold_m", None),
    ("max_linear_vel", None),
    ("min_lookahead_dist", None),
    ("max_lookahead_dist", None),
    ("lookahead_time", None),
    ("segment_lookahead_cross_collinear_deg", None),
    ("segment_endpoint_approach_speed", None),
    ("a_lat_max", None),
    ("approach_velocity_scaling_dist", None),
]

FCU_EXPECT = [
    ("RO_MAX_THR_SPEED", 0.96),
    ("RD_MAX_THR_YAW_R", 0.95),
    ("RO_SPEED_LIM", None),
    ("RO_ACCEL_LIM", None),
    ("RO_DECEL_LIM", None),
    ("RO_SPEED_TH", 0.1),
    ("RO_YAW_P", 1.5),
    ("RO_YAW_RATE_P", 0.13),
    ("RO_YAW_RATE_LIM", 22.0),
    ("EKF2_GPS_YAW_OFF", 180.0),
    ("EKF2_GPS_POS_X", 0.0),
    ("EKF2_GPS_POS_Y", 0.0),
    ("EKF2_GPS_POS_Z", -0.4),
    ("EKF2_WENC_CTRL", None),
    ("EKF2_WENC_NOISE", None),
    ("EKF2_WENC_GATE", None),
    ("EKF2_WENC_RAD", None),
    ("RBCLW_COUNTS_REV", None),
    ("RBCLW_QPPS_MAX", None),
    ("RD_WHEEL_TRACK", None),
    ("NAV_RCL_ACT", None),
    ("NAV_DLL_ACT", None),
    ("PWM_AUX_FUNC1", None),
    ("PWM_AUX_MAX1", None),
]


def report(title, target, spec, node):
    names = [n for n, _ in spec]
    vals, err = fetch(node, target, names)
    print("\n=== " + title + "  (" + target + ")")
    if err:
        print("  !! " + err)
        return 1
    bad = 0
    for (name, want), got in zip(spec, vals):
        if got is None:
            print("  %-34s MISSING" % name)
            bad += 1
        elif want is None:
            print("  %-34s %s" % (name, got))
        elif isinstance(got, float) and isinstance(want, float) and abs(got - want) < 1e-6:
            print("  %-34s %s   ok" % (name, got))
        elif got == want:
            print("  %-34s %s   ok" % (name, got))
        else:
            print("  %-34s %s   MISMATCH (expected %s)" % (name, got, want))
            bad += 1
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rpp", nargs="*", help="read these companion params")
    ap.add_argument("--fcu", nargs="*", help="read these PX4 params")
    ap.add_argument("--list-fcu", metavar="PREFIX", help="enumerate FCU params by prefix")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("quick_params_reader")
    rc = 0
    try:
        if args.list_fcu is not None:
            found = enumerate_params(node, FCU_NODE, args.list_fcu)
            print("\n".join(found or ["(none / node unavailable)"]))
            return
        if args.rpp is not None:
            rc += report("RPP", RPP_NODE, [(n, None) for n in args.rpp], node)
        if args.fcu is not None:
            rc += report("FCU", FCU_NODE, [(n, None) for n in args.fcu], node)
        if args.rpp is None and args.fcu is None:
            rc += report("COMPANION RPP params", RPP_NODE, RPP_EXPECT, node)
            rc += report("PX4 / FCU params (READ ONLY)", FCU_NODE, FCU_EXPECT, node)
            print("\n" + ("ALL OK" if rc == 0 else str(rc) + " problem(s)"))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(1 if rc else 0)


if __name__ == "__main__":
    main()
