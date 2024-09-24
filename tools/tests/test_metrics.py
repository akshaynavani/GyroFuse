import numpy as np
import pandas as pd
import pytest

from gyrofuse_tools.metrics import angle_error, axis_stats, rmse, summarize, wrap


def test_wrap_range():
    a = wrap(np.array([0.0, np.pi, -np.pi, 3 * np.pi / 2, -5 * np.pi / 2]))
    np.testing.assert_allclose(a, [0.0, np.pi, np.pi, -np.pi / 2, -np.pi / 2], atol=1e-12)


def test_angle_error_crosses_pi():
    err = angle_error(np.array([np.pi - 0.01]), np.array([-np.pi + 0.01]))
    assert err[0] == pytest.approx(-0.02, abs=1e-12)


def test_rmse_constant_offset():
    ref = np.linspace(-1, 1, 100)
    assert rmse(ref + 0.1, ref) == pytest.approx(0.1)


def test_axis_stats_skip_transient():
    ref = np.zeros(100)
    est = np.zeros(100)
    est[:10] = np.deg2rad(45.0)
    est[10:] = np.deg2rad(1.0)
    st = axis_stats(est, ref, skip=10)
    assert st.rmse_deg == pytest.approx(1.0)
    assert st.max_abs_deg == pytest.approx(1.0)
    assert st.mean_deg == pytest.approx(1.0)


def test_summarize_only_axes_with_reference():
    df = pd.DataFrame({"roll": [0.0, 0.1], "pitch": [0.0, 0.0], "yaw": [0.0, 0.0],
                       "roll_ref": [0.0, 0.0], "pitch_ref": [0.0, 0.0]})
    out = summarize(df)
    assert set(out) == {"roll", "pitch"}
    assert out["pitch"]["rmse_deg"] == pytest.approx(0.0)
