import math
import pytest

# Test the static helper directly without needing ROS2 runtime
def alignment_accel_scale(
    heading_error_rad: float,
    curvature_m_inv: float,
    full_accel_heading_deg: float = 2.0,
    no_accel_heading_deg: float = 5.0,
    full_accel_curvature: float = 0.05,
    no_accel_curvature: float = 0.15,
) -> float:
    heading_error_deg = math.degrees(abs(heading_error_rad))
    abs_curvature = abs(curvature_m_inv)

    if no_accel_heading_deg <= full_accel_heading_deg:
        heading_scale = 1.0 if heading_error_deg <= full_accel_heading_deg else 0.0
    elif heading_error_deg <= full_accel_heading_deg:
        heading_scale = 1.0
    elif heading_error_deg >= no_accel_heading_deg:
        heading_scale = 0.0
    else:
        heading_scale = (no_accel_heading_deg - heading_error_deg) / (
            no_accel_heading_deg - full_accel_heading_deg
        )

    if no_accel_curvature <= full_accel_curvature:
        curvature_scale = 1.0 if abs_curvature <= full_accel_curvature else 0.0
    elif abs_curvature <= full_accel_curvature:
        curvature_scale = 1.0
    elif abs_curvature >= no_accel_curvature:
        curvature_scale = 0.0
    else:
        curvature_scale = (no_accel_curvature - abs_curvature) / (
            no_accel_curvature - full_accel_curvature
        )

    return max(0.0, min(1.0, min(heading_scale, curvature_scale)))


def test_straight_and_aligned():
    # 0 heading error, 0 curvature -> 1.0
    scale = alignment_accel_scale(0.0, 0.0)
    assert scale == pytest.approx(1.0)


def test_small_heading_error():
    # 1.5 deg heading error, 0 curvature -> 1.0
    scale = alignment_accel_scale(math.radians(1.5), 0.0)
    assert scale == pytest.approx(1.0)


def test_large_heading_error():
    # 6.0 deg heading error -> 0.0
    scale = alignment_accel_scale(math.radians(6.0), 0.0)
    assert scale == pytest.approx(0.0)


def test_interpolated_heading_error():
    # 3.5 deg heading error (midway between 2.0 and 5.0) -> 0.5
    scale = alignment_accel_scale(math.radians(3.5), 0.0)
    assert scale == pytest.approx(0.5)


def test_high_curvature_blocks_accel():
    # 0 heading error, but in sharp turn (kappa = 0.35) -> 0.0
    scale = alignment_accel_scale(0.0, 0.35)
    assert scale == pytest.approx(0.0)


def test_gentle_curvature_allows_accel():
    # 0 heading error, curvature = 0.02 (R=50m) -> 1.0
    scale = alignment_accel_scale(0.0, 0.02)
    assert scale == pytest.approx(1.0)


def test_mid_curvature():
    # 0 heading error, curvature = 0.10 (midway between 0.05 and 0.15) -> 0.5
    scale = alignment_accel_scale(0.0, 0.10)
    assert scale == pytest.approx(0.5)


def test_combined_worst_case():
    # Heading scale = 0.5 (3.5 deg), Curvature scale = 0.2 (0.13 kappa) -> min is 0.2
    scale = alignment_accel_scale(math.radians(3.5), 0.13)
    assert scale == pytest.approx(0.2)
