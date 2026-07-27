#!/usr/bin/env python3
"""Unit tests for the production bag auto-recorder's pure helpers.

Covers the parts that run WITHOUT ROS or a live rover: secret redaction (G3),
manifest write/read + integrity (G2/G3), disk retention (G4), crash
reconciliation (G5), and the FCU/RPP param parsers (G2, via monkeypatched _run).

Run:  python3 tools/test_bag_autorecord.py      (stdlib unittest, no deps)
"""
import json
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bag_autorecord as bar  # noqa: E402


class TestRedaction(unittest.TestCase):
    def test_kv_secrets_masked(self):
        for raw in [
            "token=abc123DEF",
            "password: hunter2",
            "passwd=letmein",
            "secret = s3cr3t",
            "api_key=AK12345",
            "X-Rover-Token: rovertok987",
            "Authorization=Bearer zzz",
        ]:
            out = bar._redact(raw)
            self.assertNotIn("abc123DEF", out)
            self.assertIn("***", out, msg=f"not masked: {raw!r} -> {out!r}")

    def test_url_credentials_masked(self):
        out = bar._redact("rtsp://user:p4ss@caster.example.com:2101/mount")
        self.assertNotIn("p4ss", out)
        self.assertIn("user:***@", out)

    def test_ntrip_env_line_masked(self):
        out = bar._redact("NTRIP_PASSWORD=verysecret host=rtk2go.com")
        self.assertNotIn("verysecret", out)

    def test_benign_text_untouched(self):
        s = "mission square_2x2 xtrack=1.4cm speed=0.35"
        self.assertEqual(bar._redact(s), s)

    def test_redact_obj_recurses(self):
        obj = {"a": "token=deadbeef", "b": ["password: x", {"c": "ok"}], "n": 42}
        out = bar._redact_obj(obj)
        self.assertNotIn("deadbeef", json.dumps(out))
        self.assertNotIn("password: x", json.dumps(out))
        self.assertEqual(out["n"], 42)
        self.assertEqual(out["b"][1]["c"], "ok")


class TestIntegrityAndManifest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def test_sha256_and_bundle_integrity(self):
        bundle = os.path.join(self.tmp, "b1")
        os.makedirs(os.path.join(bundle, "bag"))
        with open(os.path.join(bundle, "bag", "data.db3"), "wb") as f:
            f.write(b"hello world")
        integ = bar._bundle_integrity(bundle, exclude={bar.MANIFEST_NAME})
        self.assertEqual(integ["file_count"], 1)
        entry = integ["files"]["bag/data.db3"]
        self.assertEqual(entry["bytes"], 11)
        # sha256("hello world")
        self.assertEqual(
            entry["sha256"],
            "b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9",
        )

    def test_manifest_write_redacts_and_roundtrips(self):
        bundle = os.path.join(self.tmp, "b2")
        os.makedirs(bundle)
        manifest = {
            "identity": {"name": "square", "note": "token=SUPERSECRET"},
            "environment": {"hostname": "jetson"},
        }
        bar._write_manifest(bundle, manifest)
        # manifest.json exists, no tmp left behind
        self.assertTrue(os.path.exists(os.path.join(bundle, bar.MANIFEST_NAME)))
        self.assertFalse(os.path.exists(os.path.join(bundle, bar.MANIFEST_NAME + ".tmp")))
        # secret masked on disk
        raw = open(os.path.join(bundle, bar.MANIFEST_NAME)).read()
        self.assertNotIn("SUPERSECRET", raw)
        # roundtrip
        back = bar._read_manifest(bundle)
        self.assertEqual(back["identity"]["name"], "square")
        self.assertIn("***", back["identity"]["note"])


class TestParamParsers(unittest.TestCase):
    def test_fcu_param_parse(self, ):
        # ParamGet responses: one real, one integer, one failure.
        responses = {
            "RO_YAW_P": "response:\nmavros_msgs.srv.ParamGet_Response(success=True, value=mavros_msgs.msg.ParamValue(integer=0, real=1.5))",
            "EKF2_WENC_CTRL": "response:\nParamGet_Response(success=True, value=ParamValue(integer=1, real=0.0))",
            "RO_YAW_RATE_LIM": "success=False",
        }
        orig = bar._run
        bar._run = lambda cmd, timeout=5.0: next(
            (v for k, v in responses.items() if any(k in c for c in cmd)), "success=False"
        )
        orig_names = bar.FCU_PARAM_NAMES
        bar.FCU_PARAM_NAMES = ["RO_YAW_P", "EKF2_WENC_CTRL", "RO_YAW_RATE_LIM"]
        try:
            out = bar._fcu_params()
        finally:
            bar._run = orig
            bar.FCU_PARAM_NAMES = orig_names
        self.assertTrue(out["captured"])
        self.assertAlmostEqual(out["values"]["RO_YAW_P"], 1.5)
        self.assertEqual(out["values"]["EKF2_WENC_CTRL"], 1)
        self.assertIsNone(out["values"]["RO_YAW_RATE_LIM"])

    def test_rpp_param_block_parse(self):
        # 40-element data array; index 35 = max_yaw_rate_body should read 0.45.
        arr = [0.0] * 40
        arr[35] = 0.45
        arr[38] = 0.35  # mission_speed
        data_str = "data:\n- " + "\n- ".join(str(x) for x in arr)
        orig = bar._run
        bar._run = lambda cmd, timeout=5.0: data_str
        try:
            out = bar._rpp_param_block()
        finally:
            bar._run = orig
        self.assertTrue(out["captured"])
        self.assertAlmostEqual(out["values"]["max_yaw_rate_body"], 0.45)
        self.assertAlmostEqual(out["values"]["mission_speed"], 0.35)


class TestRetention(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def _make_bundle(self, name, size_bytes, mtime):
        bundle = os.path.join(self.tmp, name)
        os.makedirs(os.path.join(bundle, "bag"))
        with open(os.path.join(bundle, "bag", "data.db3"), "wb") as f:
            f.write(b"x" * size_bytes)
        bar._write_manifest(bundle, {"outcome": {"recorder_end": {"utc": "z"}}})
        os.utime(bundle, (mtime, mtime))
        return bundle

    def test_oldest_removed_when_over_cap(self):
        old = self._make_bundle("old_1", 1000, time.time() - 100)
        mid = self._make_bundle("mid_1", 1000, time.time() - 50)
        new = self._make_bundle("new_1", 1000, time.time())
        orig_max = bar.MAX_TOTAL_BYTES
        bar.MAX_TOTAL_BYTES = 1500  # force rotation; newest is always kept
        try:
            bar._enforce_retention(self.tmp)
        finally:
            bar.MAX_TOTAL_BYTES = orig_max
        self.assertFalse(os.path.exists(old), "oldest bundle should be rotated out")
        self.assertTrue(os.path.exists(new), "newest bundle must be kept")
        # mid may or may not survive depending on cap; newest always kept
        _ = mid


class TestReconcile(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()

    def test_unfinalised_bundle_marked_incomplete(self):
        bundle = os.path.join(self.tmp, "crash_1")
        os.makedirs(os.path.join(bundle, "bag"))
        with open(os.path.join(bundle, "bag", "data.db3"), "wb") as f:
            f.write(b"partial")
        # manifest as written at START — no outcome.recorder_end
        bar._write_manifest(bundle, {"outcome": {"status": "RECORDING", "recorder_end": None}})
        bar.reconcile_incomplete(self.tmp)
        self.assertTrue(os.path.exists(os.path.join(bundle, bar.INCOMPLETE_SENTINEL)))
        m = bar._read_manifest(bundle)
        self.assertEqual(m["outcome"]["status"], "INCOMPLETE")
        self.assertIsNotNone(m["outcome"]["recorder_end"])
        self.assertIn("integrity", m["outcome"])

    def test_finalised_bundle_untouched(self):
        bundle = os.path.join(self.tmp, "clean_1")
        os.makedirs(bundle)
        bar._write_manifest(bundle, {"outcome": {"status": "COMPLETE",
                                                  "recorder_end": {"utc": "2026"}}})
        bar.reconcile_incomplete(self.tmp)
        self.assertFalse(os.path.exists(os.path.join(bundle, bar.INCOMPLETE_SENTINEL)))
        m = bar._read_manifest(bundle)
        self.assertEqual(m["outcome"]["status"], "COMPLETE")


class TestSourceProvenance(unittest.TestCase):
    """metadata.source arrives in three shapes on disk; all must yield a path.

    The original reader assumed a dict and got a string, which is why
    staged_mission.source_file was empty in every bundle recorded before
    2026-07-22 and §8 absolute accuracy never ran.
    """

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.missions = os.path.join(self.tmp, "missions")
        os.makedirs(self.missions)
        self.dxf = os.path.join(self.missions, "tes_cross_line.dxf")
        with open(self.dxf, "w") as f:
            f.write("0\nEOF\n")
        self._orig = bar.MISSIONS_DIR
        bar.MISSIONS_DIR = self.missions

    def tearDown(self):
        bar.MISSIONS_DIR = self._orig

    def test_source_detail_dict_preferred(self):
        out = bar._source_block({
            "source": "tes_cross_line.dxf",
            "source_detail": {"filepath": "/srv/a.dxf", "extension": ".dxf",
                              "unit_scale_m_per_unit": 0.001},
        })
        self.assertEqual(out["filepath"], "/srv/a.dxf")
        self.assertEqual(out["unit_scale_m_per_unit"], 0.001)

    def test_legacy_string_source_resolves_against_missions_dir(self):
        # This is every bundle recorded before the fix.
        out = bar._source_block({"source": "tes_cross_line.dxf"})
        self.assertEqual(out["filepath"], self.dxf)
        self.assertEqual(out["extension"], ".dxf")

    def test_legacy_string_source_without_file_has_no_filepath(self):
        out = bar._source_block({"source": "vanished.dxf"})
        self.assertIsNone(out.get("filepath"))
        self.assertEqual(out["name"], "vanished.dxf")

    def test_dict_under_source_also_accepted(self):
        out = bar._source_block({"source": {"filepath": "/srv/b.dxf"}})
        self.assertEqual(out["filepath"], "/srv/b.dxf")

    def test_builtin_and_empty_yield_nothing(self):
        for meta in ({}, {"source": ""}, {"source": "builtin:square"}):
            self.assertEqual(bar._source_block(meta), {}, msg=repr(meta))

    def test_staged_mission_never_raises_on_a_string_source(self):
        """The docstring promises "never raises" — a string source used to
        raise AttributeError on source.get(), inside no try block."""
        staging = os.path.join(self.tmp, "staging")
        os.makedirs(staging)
        with open(os.path.join(staging, "stg_x.json"), "w") as f:
            json.dump({"mission_id": "stg_x",
                       "metadata": {"source": "tes_cross_line.dxf"}}, f)
        orig = bar.STAGING_DIR
        bar.STAGING_DIR = staging
        try:
            out = bar._staged_mission("stg_x")
        finally:
            bar.STAGING_DIR = orig
        self.assertTrue(out["available"])
        self.assertEqual(out["source_file"], self.dxf)


if __name__ == "__main__":
    unittest.main(verbosity=2)
