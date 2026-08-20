"""Unit tests for the shared, ROS-independent RTK quality policy."""

from rtk_quality import evaluate_rtk_quality


def _quality(**overrides):
    values = dict(
        fix_type=6,
        h_acc_m=0.015,
        sample_age_s=0.1,
        timeout_s=0.5,
        min_fix_type=6,
        max_h_acc_m=0.10,
        require_accuracy=True,
    )
    values.update(overrides)
    return evaluate_rtk_quality(**values)


def test_fresh_fixed_accurate_sample_passes():
    assert _quality().acceptable is True


def test_old_fixed_sample_fails_stale():
    result = _quality(sample_age_s=0.51)
    assert result.fresh is False
    assert result.acceptable is False
    assert "stale" in result.reason


def test_missing_sample_fails_closed():
    result = _quality(sample_age_s=None)
    assert result.fresh is False
    assert result.acceptable is False


def test_float_fix_fails_even_with_tight_accuracy():
    result = _quality(fix_type=5, h_acc_m=0.005)
    assert result.acceptable is False
    assert result.reason == "gps fix 5 < required 6"


def test_static_and_ppp_enum_values_do_not_pass_as_better_than_rtk_fixed():
    for fix_type in (7, 8):
        result = _quality(fix_type=fix_type)
        assert result.acceptable is False
        assert result.reason == (
            f"gps fix {fix_type} is not rover RTK_FLOAT/RTK_FIXED"
        )


def test_unknown_accuracy_fails_closed_by_default():
    result = _quality(h_acc_m=None)
    assert result.acceptable is False
    assert result.reason == "gps horizontal accuracy unknown"


def test_unknown_accuracy_requires_explicit_escape_hatch():
    assert _quality(h_acc_m=None, require_accuracy=False).acceptable is True


def test_reported_accuracy_over_limit_fails():
    result = _quality(h_acc_m=0.101)
    assert result.acceptable is False
    assert "0.101m > 0.100m" in result.reason


def test_zero_limit_only_disables_upper_bound():
    assert _quality(h_acc_m=5.0, max_h_acc_m=0.0).acceptable is True
    assert _quality(h_acc_m=None, max_h_acc_m=0.0).acceptable is False
