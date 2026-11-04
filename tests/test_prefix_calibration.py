from types import SimpleNamespace

import numpy as np
import pytest

from monocular_depth_slam.depth.calibration import scale_prepared_depth, temporal_partition
from monocular_depth_slam.depth.metrics import fit_prefix_scale


def test_calibration_boundary_cannot_overlap_evaluation_or_consume_later_ratios():
    frames = [SimpleNamespace(rgb=SimpleNamespace(timestamp=i)) for i in range(4)]
    prefix, later = temporal_partition(frames, 2)
    assert prefix[-1].rgb.timestamp < later[0].rgb.timestamp
    # A pathological held-out value cannot change the frozen prefix estimate.
    rows = [{"median_gt_over_pred": ratio} for ratio in (1.0,1.2,float("nan"),999)]
    assert fit_prefix_scale(rows, len(prefix)) == pytest.approx(1.1)
    with pytest.raises(ValueError):
        temporal_partition(frames, len(frames))


def test_uniform_depth_calibration_preserves_invalid_zeros_and_relative_geometry():
    original = np.array([[0,1],[2,3]], dtype=np.float32)
    corrected = scale_prepared_depth(original, 1.1)
    assert corrected.dtype == np.float32 and corrected[0,0] == 0
    np.testing.assert_allclose(corrected[original>0]/original[original>0], 1.1)
    np.testing.assert_array_equal(original, [[0,1],[2,3]])


@pytest.mark.parametrize("scale", [0,-1,float("inf"),float("nan")])
def test_invalid_depth_correction_fails(scale):
    with pytest.raises(ValueError):
        scale_prepared_depth(np.ones((2,2), dtype=np.float32), scale)
