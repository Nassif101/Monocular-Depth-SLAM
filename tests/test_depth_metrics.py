import numpy as np
import pytest

from monocular_depth_slam.depth.metrics import (
    aggregate_metrics, canonical_to_metres, depth_metrics, erode_mask_one_pixel,
    fit_prefix_scale, resize_depth_to_reference,
)


def test_conversion_uses_processed_focal_and_canonical_300():
    matrix = np.diag([400.0, 410.0, 1.0])
    result = canonical_to_metres(np.array([[1.0, 2.0]], dtype=np.float32), matrix)
    np.testing.assert_allclose(result, [[1.35, 2.70]])
    assert result.dtype == np.float32


def test_identity_predictions_and_masking_invalid_sensor_depth():
    gt = np.array([[0.0, 1.0], [2.0, np.nan]])
    metrics = depth_metrics(gt, np.array([[100.0, 1.0], [2.0, 100.0]]))
    assert metrics["reference_pixels"] == metrics["scored_pixels"] == 2
    assert metrics["abs_rel"] == metrics["rmse_m"] == metrics["bias_m"] == 0
    assert metrics["delta1"] == metrics["prediction_coverage"] == 1


def test_scale_bias_is_measured_before_oracle_correction():
    gt = np.array([[1.0, 2.0]])
    pred = gt * 2
    metrics = depth_metrics(gt, pred)
    assert metrics["abs_rel"] == 1.0
    assert metrics["mae_m"] == 1.5
    assert metrics["rmse_m"] == pytest.approx(np.sqrt(2.5))
    assert metrics["bias_m"] == 1.5
    assert metrics["delta1"] == 0.0
    assert metrics["median_gt_over_pred"] == 0.5
    assert depth_metrics(gt, pred * metrics["median_gt_over_pred"])["abs_rel"] == 0


def test_invalid_predictions_reduce_coverage_and_threshold_accuracy():
    metrics = depth_metrics(np.ones((1, 3)), np.array([[1.0, np.nan, -1.0]]))
    assert metrics["prediction_coverage"] == pytest.approx(1/3)
    assert metrics["delta1"] == pytest.approx(1/3)
    assert metrics["scored_pixels"] == 1


def test_bad_predictions_are_not_filtered_by_reference_range():
    metrics = depth_metrics(np.array([[1.0]]), np.array([[100.0]]), max_depth=10)
    assert metrics["abs_rel"] == 99
    assert metrics["prediction_coverage"] == 1


def test_pooled_rmse_is_not_average_frame_rmse():
    a = depth_metrics(np.ones((1, 1)), np.ones((1, 1)))
    b = depth_metrics(np.ones((1, 3)), np.ones((1, 3))*3)
    summary = aggregate_metrics([a, b])
    assert summary["frame_mean"]["rmse_m"] == 1
    assert summary["pixel_pooled"]["rmse_m"] == pytest.approx(np.sqrt(3))


def test_bilinear_resize_preserves_constant_depth_and_dimensions():
    depth = np.full((2, 3), 2.5, dtype=np.float32)
    result = resize_depth_to_reference(depth, (4, 6))
    assert result.shape == (4, 6)
    np.testing.assert_array_equal(result, np.full((4, 6), 2.5))


@pytest.mark.parametrize("gt,pred", [(np.ones((2, 2)), np.ones((1, 2))),
                                    (np.zeros((2, 2)), np.ones((2, 2)))])
def test_invalid_reference_or_shape_fails(gt, pred):
    with pytest.raises(ValueError):
        depth_metrics(gt, pred)


def test_no_valid_predictions_are_reported_and_not_silently_aggregated():
    metrics = depth_metrics(np.ones((2, 2)), np.zeros((2, 2)))
    assert metrics["prediction_coverage"] == 0
    assert metrics["rmse_m"] is None
    with pytest.raises(ValueError):
        aggregate_metrics([metrics])


def test_groundtruth_scale_fit_does_not_use_heldout_frames():
    rows = [{"median_gt_over_pred": value} for value in (0.5, 0.7, 10.0, 20.0)]
    assert fit_prefix_scale(rows, 2) == 0.6


def test_hole_erosion_excludes_adjacent_pixels_and_image_borders():
    mask = np.ones((7, 7), dtype=bool)
    mask[3, 3] = False
    result = erode_mask_one_pixel(mask)
    assert not result[2:5, 2:5].any()
    assert not result[0].any()
    assert result.sum() == 16


def test_resize_pixel_centres_on_known_ramp():
    result = resize_depth_to_reference(np.array([[1, 2], [3, 4]], dtype=np.float32), (4, 4))
    np.testing.assert_allclose(result[0], [1, 1.25, 1.75, 2])
    np.testing.assert_allclose(result[-1], [3, 3.25, 3.75, 4])
