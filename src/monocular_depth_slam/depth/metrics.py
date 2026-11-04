"""Depth metrics in metres, with explicit reference masks and prediction coverage."""
import math

import numpy as np
from PIL import Image


METRICS = ("abs_rel", "sq_rel_m", "mae_m", "rmse_m", "rmse_log", "bias_m",
           "delta1", "delta2", "delta3")


def canonical_to_metres(canonical, processed_intrinsics):
    depth = np.asarray(canonical, dtype=np.float32)
    matrix = np.asarray(processed_intrinsics, dtype=np.float64)
    if depth.ndim != 2 or matrix.shape != (3, 3):
        raise ValueError("Expected a 2D canonical depth map and a 3x3 processed camera matrix")
    focal = (matrix[0, 0] + matrix[1, 1]) / 2
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("Processed intrinsics must be finite with positive focal lengths")
    return depth * np.float32(focal / 300.0)


def resize_depth_to_reference(depth, reference_shape):
    """Bilinear pixel-centre resize, without interpolating reference sensor depth."""
    depth = np.asarray(depth, dtype=np.float32)
    if depth.ndim != 2 or len(reference_shape) != 2 or min(reference_shape) <= 0:
        raise ValueError("Expected 2D source and reference image shapes")
    height, width = reference_shape
    if depth.shape == (height, width):
        return depth.copy()
    return np.array(Image.fromarray(depth).resize((width, height), Image.Resampling.BILINEAR))


def paired_values(reference, prediction, *, min_depth=0.1, max_depth=10.0, mask=None):
    reference = np.asarray(reference, dtype=np.float64)
    prediction = np.asarray(prediction, dtype=np.float64)
    if reference.ndim != 2 or reference.shape != prediction.shape:
        raise ValueError("Reference and prediction must have matching 2D shapes")
    if not 0 < min_depth < max_depth or not np.isfinite([min_depth, max_depth]).all():
        raise ValueError("Expected finite positive depth limits with min < max")
    reference_mask = np.isfinite(reference) & (reference >= min_depth) & (reference <= max_depth)
    if mask is not None:
        if np.shape(mask) != reference.shape:
            raise ValueError("Mask shape must match reference depth")
        reference_mask &= np.asarray(mask, dtype=bool)
    n_reference = int(reference_mask.sum())
    if not n_reference:
        raise ValueError("No valid reference pixels in the declared depth range")
    valid = reference_mask & np.isfinite(prediction) & (prediction > 0)
    return reference[valid], prediction[valid], n_reference


def depth_metrics(reference, prediction, **kwargs):
    groundtruth, predicted, n_reference = paired_values(reference, prediction, **kwargs)
    n = len(groundtruth)
    result = {"reference_pixels": n_reference, "scored_pixels": n,
              "prediction_coverage": n / n_reference}
    if not n:
        return {**result, **{key: None for key in METRICS}, "median_pred_over_gt": None,
                "median_gt_over_pred": None}
    error = predicted - groundtruth
    ratio = np.maximum(predicted / groundtruth, groundtruth / predicted)
    result.update(
        abs_rel=float(np.mean(np.abs(error) / groundtruth)),
        sq_rel_m=float(np.mean(error**2 / groundtruth)),
        mae_m=float(np.mean(np.abs(error))), rmse_m=float(np.sqrt(np.mean(error**2))),
        rmse_log=float(np.sqrt(np.mean((np.log(predicted) - np.log(groundtruth))**2))),
        bias_m=float(np.mean(error)),
        # Invalid predictions count as failures for threshold accuracy.
        delta1=float(np.count_nonzero(ratio < 1.25) / n_reference),
        delta2=float(np.count_nonzero(ratio < 1.25**2) / n_reference),
        delta3=float(np.count_nonzero(ratio < 1.25**3) / n_reference),
        median_pred_over_gt=float(np.median(predicted / groundtruth)),
        median_gt_over_pred=float(np.median(groundtruth / predicted)),
    )
    return result


def aggregate_metrics(rows):
    if not rows or any(row["scored_pixels"] == 0 for row in rows):
        raise ValueError("Every requested frame must have scored pixels; report failures explicitly")
    n = sum(row["scored_pixels"] for row in rows)
    n_reference = sum(row["reference_pixels"] for row in rows)
    frame_mean = {key: float(np.mean([row[key] for row in rows])) for key in METRICS}
    pooled = {}
    for key in METRICS:
        if key.startswith("delta"):
            pooled[key] = sum(row[key] * row["reference_pixels"] for row in rows) / n_reference
        elif key in ("rmse_m", "rmse_log"):
            pooled[key] = math.sqrt(sum(row[key]**2 * row["scored_pixels"] for row in rows) / n)
        else:
            pooled[key] = sum(row[key] * row["scored_pixels"] for row in rows) / n
    ratios = [row["median_pred_over_gt"] for row in rows]
    return {"frames": len(rows), "reference_pixels": n_reference, "scored_pixels": n,
            "prediction_coverage": n / n_reference, "frame_mean": frame_mean,
            "pixel_pooled": pooled,
            "frame_median_pred_over_gt_quantiles": {
                str(q): float(np.quantile(ratios, q)) for q in (0, 0.25, 0.5, 0.75, 1)}}


def fit_prefix_scale(rows, calibration_count):
    """GT-anchored diagnostic fit using only a declared calibration prefix."""
    if not 0 < calibration_count <= len(rows):
        raise ValueError("Invalid calibration prefix length")
    values = [row["median_gt_over_pred"] for row in rows[:calibration_count]]
    if any(value is None or not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError("Calibration frames require valid positive scale ratios")
    return float(np.median(values))


def erode_mask_one_pixel(mask):
    """Exclude image borders and pixels adjacent to holes; sensitivity check only."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        raise ValueError("Expected a 2D validity mask")
    result = np.zeros_like(mask)
    if min(mask.shape) < 3:
        return result
    core = np.ones((mask.shape[0]-2, mask.shape[1]-2), dtype=bool)
    for dy in range(3):
        for dx in range(3):
            core &= mask[dy:dy+core.shape[0], dx:dx+core.shape[1]]
    result[1:-1, 1:-1] = core
    return result
