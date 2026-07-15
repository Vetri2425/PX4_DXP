"""Unit tests for spray_fsm.SpraySafetyStateMachine (Spray Controller V2, Phase A).

Pure stdlib unittest, no ROS import anywhere in this file or in spray_fsm.py.
Runnable both as:
    python3 -m pytest src/test_spray_fsm.py -q
    python3 src/test_spray_fsm.py
"""

import random
import unittest

from spray_fsm import SprayCommand, SpraySafetyStateMachine, SprayState


class TestStartup(unittest.TestCase):
    def test_boots_off_unconfirmed(self):
        fsm = SpraySafetyStateMachine()
        self.assertEqual(fsm.state, SprayState.OFF_UNCONFIRMED)
        self.assertFalse(fsm.spraying)
        self.assertFalse(fsm.commanded)
        self.assertEqual(fsm.cmd_seq, 0)

    def test_off_unconfirmed_only_action_is_dispatch_off(self):
        fsm = SpraySafetyStateMachine()
        # Even with ON desired, OFF_UNCONFIRMED must dispatch OFF first.
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=0.0)
        self.assertIsNotNone(cmd)
        self.assertFalse(cmd.on)
        self.assertEqual(cmd.seq, 1)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)

    def test_no_on_accepted_until_off_confirmed(self):
        fsm = SpraySafetyStateMachine()
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=0.0)
        self.assertFalse(cmd.on)  # forced OFF, not the desired ON
        # Ack the OFF.
        result = fsm.on_ack(cmd.seq, success=True, now=0.1)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)
        # Now ON is accepted.
        cmd2 = fsm.tick(desired=True, safety_ok=True, enabled=True, now=0.2)
        self.assertTrue(cmd2.on)
        self.assertEqual(fsm.state, SprayState.ON_PENDING)


class TestFullTransitionTable(unittest.TestCase):
    def _to_off_confirmed(self, fsm, now=0.0):
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=now)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        fsm.on_ack(cmd.seq, success=True, now=now + 0.1)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)

    def _to_on_confirmed(self, fsm, now=0.0):
        self._to_off_confirmed(fsm, now)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=now + 0.2)
        self.assertEqual(fsm.state, SprayState.ON_PENDING)
        fsm.on_ack(cmd.seq, success=True, now=now + 0.3)
        self.assertEqual(fsm.state, SprayState.ON_CONFIRMED)

    def test_off_confirmed_desired_on_dispatches_on_pending(self):
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        self.assertEqual(cmd, SprayCommand(on=True, seq=fsm.cmd_seq, force=False))
        self.assertEqual(fsm.state, SprayState.ON_PENDING)

    def test_on_pending_ack_success_to_on_confirmed(self):
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        result = fsm.on_ack(cmd.seq, success=True, now=1.1)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.ON_CONFIRMED)
        self.assertTrue(fsm.spraying)

    def test_on_pending_ack_fail_never_latches_on_goes_to_off_pending(self):
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        result = fsm.on_ack(cmd.seq, success=False, now=1.1)
        self.assertIsNotNone(result)
        self.assertFalse(result.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        self.assertFalse(fsm.spraying)

    def test_on_pending_desired_false_changes_mind_to_off_pending(self):
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        cmd_on = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        self.assertEqual(fsm.state, SprayState.ON_PENDING)
        cmd_off = fsm.tick(desired=False, safety_ok=True, enabled=True, now=1.05)
        self.assertIsNotNone(cmd_off)
        self.assertFalse(cmd_off.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        self.assertNotEqual(cmd_off.seq, cmd_on.seq)
        # A later stale ON ack (old seq) must not resurrect ON.
        stale = fsm.on_ack(cmd_on.seq, success=True, now=1.2)
        self.assertIsNone(stale)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)

    def test_on_confirmed_desired_false_to_off_pending(self):
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.0)
        self.assertIsNotNone(cmd)
        self.assertFalse(cmd.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)

    def test_off_pending_ack_success_to_off_confirmed(self):
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.0)
        result = fsm.on_ack(cmd.seq, success=True, now=2.1)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)

    def test_off_pending_ack_fail_to_recovery(self):
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.0)
        result = fsm.on_ack(cmd.seq, success=False, now=2.1)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.RECOVERY)

    def test_recovery_backoff_elapsed_retries_off_pending(self):
        fsm = SpraySafetyStateMachine(backoff_base_s=0.5, backoff_max_s=5.0)
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.0)
        fsm.on_ack(cmd.seq, success=False, now=2.1)
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        # Backoff not elapsed yet -> no retry.
        self.assertIsNone(fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.2))
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        # Backoff elapsed (0.5s after entering recovery at now=2.1) -> retry.
        retry = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.6)
        self.assertIsNotNone(retry)
        self.assertFalse(retry.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)

    def test_recovery_ack_success_to_off_confirmed(self):
        fsm = SpraySafetyStateMachine(backoff_base_s=0.5, backoff_max_s=5.0)
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.0)
        fsm.on_ack(cmd.seq, success=False, now=2.1)
        retry = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.6)
        result = fsm.on_ack(retry.seq, success=True, now=2.7)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)

    def test_safety_loss_from_unconfirmed_or_on_forces_off_pending(self):
        # States where the actuator is (or may be) on, or its off-state is
        # unconfirmed: a safety-loss tick must dispatch a forced OFF.
        cases = []
        # OFF_UNCONFIRMED
        cases.append(SpraySafetyStateMachine())
        # ON_PENDING — created at now=5.0 so the safety-loss tick below (also
        # now=5.0) is inside the ack-timeout window and exercises the
        # safety-loss path, not the pending-ack timeout.
        fsm2 = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm2)
        fsm2.tick(desired=True, safety_ok=True, enabled=True, now=5.0)
        self.assertEqual(fsm2.state, SprayState.ON_PENDING)
        cases.append(fsm2)
        # ON_CONFIRMED
        fsm3 = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm3)
        cases.append(fsm3)

        for fsm in cases:
            cmd = fsm.tick(desired=True, safety_ok=False, enabled=True, now=5.0)
            self.assertIsNotNone(cmd, msg=f"expected forced OFF from {fsm.state}")
            self.assertFalse(cmd.on)
            self.assertTrue(cmd.force)
            self.assertEqual(fsm.state, SprayState.OFF_PENDING)
            self.assertFalse(fsm.spraying)

    def test_safety_loss_when_already_off_confirmed_stays_quiet(self):
        # Regression guard: a sustained unsafe condition (e.g. sitting
        # disarmed) must NOT re-dispatch a forced OFF every tick once OFF is
        # already confirmed — that would flood /mavros/cmd/command at the
        # ~50 Hz watchdog rate. It stays silent and remains OFF_CONFIRMED.
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        seq_before = fsm.cmd_seq
        for i in range(20):
            cmd = fsm.tick(desired=True, safety_ok=False, enabled=True, now=5.0 + i)
            self.assertIsNone(cmd, msg=f"tick {i}: expected no command while OFF_CONFIRMED+unsafe")
            self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)
            self.assertFalse(fsm.spraying)
        self.assertEqual(fsm.cmd_seq, seq_before, "cmd_seq must not advance while quiet")

    def test_safety_loss_never_returns_on_command(self):
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=False, enabled=True, now=1.0)
        # Either None or a forced OFF -- never an ON.
        if cmd is not None:
            self.assertFalse(cmd.on)

    def test_disabled_forces_off_and_settles_disabled(self):
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=False, now=3.0)
        self.assertIsNotNone(cmd)
        self.assertFalse(cmd.on)
        self.assertTrue(cmd.force)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        result = fsm.on_ack(cmd.seq, success=True, now=3.1)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.DISABLED)
        self.assertFalse(fsm.spraying)

    def test_disabled_from_off_confirmed_is_immediate_no_redispatch(self):
        fsm = SpraySafetyStateMachine()
        self._to_off_confirmed(fsm)
        seq_before = fsm.cmd_seq
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=False, now=3.0)
        self.assertIsNone(cmd)
        self.assertEqual(fsm.state, SprayState.DISABLED)
        self.assertEqual(fsm.cmd_seq, seq_before)

    def test_disabled_refuses_on_regardless_of_desired(self):
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=False, now=3.0)
        fsm.on_ack(cmd.seq, success=True, now=3.1)
        self.assertEqual(fsm.state, SprayState.DISABLED)
        # Still disabled; caller keeps asking for ON -- must never get one.
        for t in (3.2, 3.3, 3.4):
            out = fsm.tick(desired=True, safety_ok=True, enabled=False, now=t)
            self.assertIsNone(out)
            self.assertEqual(fsm.state, SprayState.DISABLED)
            self.assertFalse(fsm.spraying)

    def test_disable_then_reenable_goes_through_off_unconfirmed(self):
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=False, now=3.0)
        fsm.on_ack(cmd.seq, success=True, now=3.1)
        self.assertEqual(fsm.state, SprayState.DISABLED)

        # Re-enable: must land on OFF_UNCONFIRMED, never straight to
        # OFF_CONFIRMED.
        out = fsm.tick(desired=False, safety_ok=True, enabled=True, now=4.0)
        # The edge handler sets OFF_UNCONFIRMED, then the same tick's
        # normal-path dispatches OFF (OFF_UNCONFIRMED's only action).
        self.assertIsNotNone(out)
        self.assertFalse(out.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)

        # And ON must not be accepted until that fresh OFF confirms.
        cmd2 = fsm.tick(desired=True, safety_ok=True, enabled=True, now=4.1)
        self.assertIsNone(cmd2)  # awaiting the OFF ack, no ON dispatched
        fsm.on_ack(out.seq, success=True, now=4.2)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)
        cmd3 = fsm.tick(desired=True, safety_ok=True, enabled=True, now=4.3)
        self.assertTrue(cmd3.on)

    def test_disable_reenable_state_progression_never_off_confirmed_directly(self):
        """Regression test: at no point during the disable->re-enable cycle
        does the FSM report OFF_CONFIRMED without having gone through a
        fresh OFF_UNCONFIRMED/OFF_PENDING re-confirmation.
        """
        fsm = SpraySafetyStateMachine()
        self._to_on_confirmed(fsm)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=False, now=3.0)
        fsm.on_ack(cmd.seq, success=True, now=3.1)
        self.assertEqual(fsm.state, SprayState.DISABLED)

        out = fsm.tick(desired=False, safety_ok=True, enabled=True, now=4.0)
        # Must never have jumped straight to OFF_CONFIRMED.
        self.assertNotEqual(fsm.state, SprayState.OFF_CONFIRMED)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)


class TestInvariants(unittest.TestCase):
    def test_invariant1_spraying_only_true_in_on_confirmed(self):
        fsm = SpraySafetyStateMachine()
        for state in SprayState:
            fsm._state = state  # white-box: force every state, check property
            self.assertEqual(fsm.spraying, state == SprayState.ON_CONFIRMED)

    def test_invariant1_dispatching_on_does_not_set_spraying(self):
        fsm = SpraySafetyStateMachine()
        cmd0 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=0.0)
        fsm.on_ack(cmd0.seq, success=True, now=0.1)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=0.2)
        self.assertTrue(cmd.on)
        # ON dispatched, but not yet acked -- must NOT be spraying.
        self.assertFalse(fsm.spraying)
        self.assertEqual(fsm.state, SprayState.ON_PENDING)

    def test_invariant2_stale_seq_ignored(self):
        fsm = SpraySafetyStateMachine()
        cmd0 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=0.0)
        stale_seq = cmd0.seq
        fsm.on_ack(cmd0.seq, success=True, now=0.1)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)
        cmd1 = fsm.tick(desired=True, safety_ok=True, enabled=True, now=0.2)
        self.assertNotEqual(cmd1.seq, stale_seq)
        # A stale ack (old seq) after a new command was dispatched must be a no-op.
        result = fsm.on_ack(stale_seq, success=False, now=0.3)
        self.assertIsNone(result)
        self.assertEqual(fsm.state, SprayState.ON_PENDING)  # unchanged

    def test_invariant2_every_dispatch_carries_current_cmd_seq(self):
        fsm = SpraySafetyStateMachine()
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=0.0)
        self.assertEqual(cmd.seq, fsm.cmd_seq)


class TestRecoveryBackoffTiming(unittest.TestCase):
    def _fail_off_from_on_confirmed(self, fsm, now):
        cmd_off = fsm.tick(desired=False, safety_ok=True, enabled=True, now=now)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        fsm.on_ack(cmd_off.seq, success=False, now=now + 0.01)
        self.assertEqual(fsm.state, SprayState.RECOVERY)

    def _make_on_confirmed(self, fsm):
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=0.0)
        fsm.on_ack(cmd.seq, success=True, now=0.01)
        cmd = fsm.tick(desired=True, safety_ok=True, enabled=True, now=0.02)
        fsm.on_ack(cmd.seq, success=True, now=0.03)
        self.assertEqual(fsm.state, SprayState.ON_CONFIRMED)

    def test_bounded_exponential_backoff(self):
        fsm = SpraySafetyStateMachine(backoff_base_s=0.5, backoff_max_s=5.0)
        self._make_on_confirmed(fsm)

        t = 1.0
        self._fail_off_from_on_confirmed(fsm, t)  # fail-ack at t+0.01 -> deadline t+0.51
        fail_ack_time = t + 0.01
        deadline1 = fail_ack_time + 0.5
        # Not yet elapsed just before deadline.
        self.assertIsNone(
            fsm.tick(desired=False, safety_ok=True, enabled=True, now=deadline1 - 0.01)
        )
        retry1 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=deadline1)
        self.assertIsNotNone(retry1)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)

        # Fail again -> attempt 1 -> backoff should be 1.0s this time.
        fail_ack_time2 = deadline1 + 0.01
        fsm.on_ack(retry1.seq, success=False, now=fail_ack_time2)
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        deadline2 = fail_ack_time2 + 1.0
        self.assertIsNone(
            fsm.tick(desired=False, safety_ok=True, enabled=True, now=deadline2 - 0.01)
        )
        retry2 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=deadline2)
        self.assertIsNotNone(retry2)

        # Fail again -> attempt 2 -> backoff 2.0s.
        fail_ack_time3 = deadline2 + 0.01
        fsm.on_ack(retry2.seq, success=False, now=fail_ack_time3)
        deadline3 = fail_ack_time3 + 2.0
        self.assertIsNone(
            fsm.tick(desired=False, safety_ok=True, enabled=True, now=deadline3 - 0.01)
        )
        retry3 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=deadline3)
        self.assertIsNotNone(retry3)

    def test_backoff_clamped_to_max(self):
        fsm = SpraySafetyStateMachine(backoff_base_s=0.5, backoff_max_s=2.0)
        self._make_on_confirmed(fsm)
        t = 0.0
        # Drive several consecutive failures to blow past backoff_max_s.
        cmd_off = fsm.tick(desired=False, safety_ok=True, enabled=True, now=t)
        for _ in range(6):
            fsm.on_ack(cmd_off.seq, success=False, now=t + 0.001)
            self.assertEqual(fsm.state, SprayState.RECOVERY)
            deadline = fsm._recovery_deadline
            self.assertLessEqual(deadline - (t + 0.001), 2.0 + 1e-9)
            t = deadline
            cmd_off = fsm.tick(desired=False, safety_ok=True, enabled=True, now=t)
            self.assertIsNotNone(cmd_off)

    def test_safety_loss_edge_resets_backoff_for_fast_first_retry(self):
        fsm = SpraySafetyStateMachine(backoff_base_s=0.5, backoff_max_s=5.0)
        self._make_on_confirmed(fsm)
        t = 1.0
        self._fail_off_from_on_confirmed(fsm, t)
        retry1 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=t + 0.51)
        fsm.on_ack(retry1.seq, success=False, now=t + 0.52)
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        # Attempt counter is now 2 (next backoff would be 2.0s). A
        # safety-loss edge should reset it and force an immediate dispatch,
        # bypassing the backoff wait entirely (force=True).
        forced = fsm.tick(desired=False, safety_ok=False, enabled=True, now=t + 0.6)
        self.assertIsNotNone(forced)
        self.assertTrue(forced.force)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        # Fail once more (safety still lost) -- since the edge reset
        # attempt to 0, the next recovery backoff should be back to
        # backoff_base_s, not the escalated value.
        fsm.on_ack(forced.seq, success=False, now=t + 0.61)
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        deadline = fsm._recovery_deadline
        self.assertAlmostEqual(deadline - (t + 0.61), 0.5, places=6)

    def test_note_event_reset_gives_fast_first_retry(self):
        fsm = SpraySafetyStateMachine(backoff_base_s=0.5, backoff_max_s=5.0)
        self._make_on_confirmed(fsm)
        t = 1.0
        self._fail_off_from_on_confirmed(fsm, t)
        retry1 = fsm.tick(desired=False, safety_ok=True, enabled=True, now=t + 0.51)
        fsm.on_ack(retry1.seq, success=False, now=t + 0.52)
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        # Config/mode-change signal arrives.
        fsm.note_event_reset(now=t + 0.6)
        # Next tick should retry immediately (deadline snapped to "now").
        out = fsm.tick(desired=False, safety_ok=True, enabled=True, now=t + 0.6)
        self.assertIsNotNone(out)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)


class TestPropertyStyleRandomSequences(unittest.TestCase):
    def test_random_event_sequences_hold_invariants(self):
        seed = 1234567
        rng = random.Random(seed)

        for trial in range(50):
            fsm = SpraySafetyStateMachine(backoff_base_s=0.1, backoff_max_s=1.0)
            now = 0.0
            pending_seqs = []  # seqs currently awaiting an ack from us

            for _ in range(300):
                now += rng.uniform(0.01, 0.05)
                desired = rng.random() < 0.5
                safety_ok = rng.random() < 0.85
                enabled = rng.random() < 0.9

                cmd = fsm.tick(desired=desired, safety_ok=safety_ok, enabled=enabled, now=now)

                # INVARIANT 1, every single tick.
                self.assertEqual(
                    fsm.spraying,
                    fsm.state == SprayState.ON_CONFIRMED,
                    msg=f"trial={trial} state={fsm.state} spraying={fsm.spraying}",
                )
                # commanded contract.
                self.assertEqual(
                    fsm.commanded,
                    fsm.state in (SprayState.ON_PENDING, SprayState.ON_CONFIRMED),
                )
                # Never an ON command while safety_ok is False.
                if not safety_ok and cmd is not None:
                    self.assertFalse(cmd.on)
                # Never an ON command while disabled/DISABLED.
                if not enabled and cmd is not None:
                    self.assertFalse(cmd.on)
                if fsm.state == SprayState.DISABLED:
                    self.assertFalse(fsm.spraying)

                if cmd is not None:
                    self.assertEqual(cmd.seq, fsm.cmd_seq)
                    pending_seqs.append(cmd.seq)

                # Occasionally deliver an ack -- sometimes for a stale seq,
                # sometimes for the latest.
                if rng.random() < 0.6:
                    now += rng.uniform(0.001, 0.02)
                    if pending_seqs and rng.random() < 0.7:
                        seq = pending_seqs.pop() if rng.random() < 0.5 else rng.choice(pending_seqs)
                    else:
                        # Possibly-stale/garbage seq.
                        seq = rng.randint(0, fsm.cmd_seq + 3)
                    success = rng.random() < 0.7
                    state_before = fsm.state
                    seq_before = fsm.cmd_seq
                    fsm.on_ack(seq, success=success, now=now)
                    if seq != seq_before:
                        # A non-matching seq must never change state (stale
                        # replies -- INVARIANT 2).
                        self.assertEqual(fsm.state, state_before)
                    # INVARIANT 1 must still hold immediately after any ack.
                    self.assertEqual(fsm.spraying, fsm.state == SprayState.ON_CONFIRMED)


class TestAckTimeout(unittest.TestCase):
    """A dispatched ON/OFF whose MAVROS ack never arrives must not wedge the
    FSM — plan §4 `ack(timeout)`. Closes the never-resolving-future hole."""

    def _to_off_confirmed(self, fsm, now=0.0):
        cmd = fsm.tick(desired=False, safety_ok=True, enabled=True, now=now)
        fsm.on_ack(cmd.seq, success=True, now=now + 0.1)
        self.assertEqual(fsm.state, SprayState.OFF_CONFIRMED)

    def test_off_pending_timeout_enters_recovery_then_retries(self):
        fsm = SpraySafetyStateMachine(ack_timeout_s=1.0, backoff_base_s=0.5)
        self._to_off_confirmed(fsm)
        # Turn ON, confirm, then command OFF whose ack never comes.
        on = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        fsm.on_ack(on.seq, success=True, now=1.1)
        off = fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.0)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        # No ack. Within the timeout window: still pending, no new command.
        self.assertIsNone(fsm.tick(desired=False, safety_ok=True, enabled=True, now=2.5))
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        # Past the timeout: synthesize the failure -> RECOVERY (no cmd yet).
        self.assertIsNone(fsm.tick(desired=False, safety_ok=True, enabled=True, now=3.01))
        self.assertEqual(fsm.state, SprayState.RECOVERY)
        # Backoff elapsed -> retry OFF.
        retry = fsm.tick(desired=False, safety_ok=True, enabled=True, now=3.6)
        self.assertIsNotNone(retry)
        self.assertFalse(retry.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        self.assertNotEqual(retry.seq, off.seq)

    def test_on_pending_timeout_drives_off_never_latches_on(self):
        fsm = SpraySafetyStateMachine(ack_timeout_s=1.0)
        self._to_off_confirmed(fsm)
        on = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        self.assertEqual(fsm.state, SprayState.ON_PENDING)
        # ON ack never arrives; past the timeout the FSM must drive OFF and
        # must NEVER report spraying (INVARIANT 1) for an unconfirmed ON.
        self.assertFalse(fsm.spraying)
        off = fsm.tick(desired=True, safety_ok=True, enabled=True, now=2.01)
        self.assertIsNotNone(off)
        self.assertFalse(off.on)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        self.assertFalse(fsm.spraying)
        # A late/stale ON ack for the timed-out command is ignored (seq bumped).
        fsm.on_ack(on.seq, success=True, now=2.1)
        self.assertEqual(fsm.state, SprayState.OFF_PENDING)
        self.assertFalse(fsm.spraying)

    def test_no_false_timeout_on_prompt_ack(self):
        fsm = SpraySafetyStateMachine(ack_timeout_s=1.0)
        self._to_off_confirmed(fsm)
        on = fsm.tick(desired=True, safety_ok=True, enabled=True, now=1.0)
        fsm.on_ack(on.seq, success=True, now=1.05)  # prompt ack, well inside window
        self.assertEqual(fsm.state, SprayState.ON_CONFIRMED)
        # Many later ticks: no spurious timeout (pending stamp was cleared).
        for i in range(10):
            self.assertIsNone(
                fsm.tick(desired=True, safety_ok=True, enabled=True, now=5.0 + i)
            )
            self.assertEqual(fsm.state, SprayState.ON_CONFIRMED)


if __name__ == "__main__":
    unittest.main()
