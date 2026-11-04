import numpy as np
import pytest

pytest.importorskip("evo")

from monocular_depth_slam.datasets.tum import PoseRecord
from monocular_depth_slam.slam.evaluation import aligned_errors, as_evo, tracking_summary, window_scale_diagnostic


def poses(positions, angles=None):
    angles = angles if angles is not None else [0]*len(positions)
    return [PoseRecord(float(i), str(i), tuple(position),
                       (0.0, 0.0, np.sin(angle/2), np.cos(angle/2)))
            for i, (position, angle) in enumerate(zip(positions, angles))]


def test_sim3_removes_global_scale_while_se3_preserves_scale_error():
    positions = np.array([[0, 0, 0], [1, 0, 0], [1, 2, 0], [1, 2, 3]], dtype=float)
    reference = poses(positions)
    estimated = poses(positions/2 + [2, 3, 4])
    se3 = aligned_errors(reference, estimated, [(0, 1), (1, 2), (2, 3)], False)[0]
    sim3 = aligned_errors(reference, estimated, [(0, 1), (1, 2), (2, 3)], True)[0]
    assert se3["scale"] == 1
    assert se3["ate_translation_m"]["rmse"] > .1
    assert sim3["scale"] == pytest.approx(2)
    assert sim3["ate_translation_m"]["rmse"] < 1e-12
    assert sim3["rpe_translation_m"]["rmse"] < 1e-12


def test_quaternion_order_and_rotation_rpe():
    positions = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0]], dtype=float)
    reference = poses(positions)
    estimated = poses(positions, [0, 0, np.deg2rad(10)])
    result = aligned_errors(reference, estimated, [(0, 1), (1, 2)], True)[0]
    assert result["rpe_rotation_deg"]["rmse"] == pytest.approx(np.sqrt(50))
    ninety = as_evo(poses([[0, 0, 0]], [np.pi/2]))
    np.testing.assert_allclose(ninety.poses_se3[0][:3, :3], [[0,-1,0],[1,0,0],[0,0,1]], atol=1e-15)


def test_tracking_initialization_and_recovered_failure_are_separate():
    rows = [{"state": str(state), "timestamp": str(i), "tracking_seconds": ".01"}
            for i, state in enumerate([1, 1, 2, 3, 4, 2, 2])]
    result = tracking_summary(rows)
    assert result["initialization_frames_before_first_ok"] == 2
    assert result["tracked_frames"] == 3
    assert result["recently_lost_frames"] == result["lost_frames"] == 1
    assert result["post_initialization_failure_events"] == [{
        "first_input_index": 3, "last_input_index": 4, "start_timestamp": "3",
        "end_timestamp": "4", "recovered": True}]


def test_scale_window_is_neutral_after_perfect_global_scale_correction():
    positions = np.array([[0,0,0],[1,0,0],[1,2,0],[1,2,3]], dtype=float)
    _, reference, aligned, _, _, _ = aligned_errors(poses(positions), poses(positions/2), [(0,1)], True)
    window, = window_scale_diagnostic(reference, aligned, [0,1,2,3], 5)
    assert window["path_length_ratio"] == pytest.approx(1)
    assert window["exploratory_local_sim3_scale_fit"]["residual_estimate_scale_percent"] == pytest.approx(0, abs=1e-12)
