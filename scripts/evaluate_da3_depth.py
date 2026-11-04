#!/usr/bin/env python3
"""Evaluate cached DA3 against TUM sensor depth; ground-truth fits stay diagnostic."""
import argparse
import csv
from datetime import datetime, timezone
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from monocular_depth_slam.datasets.tum import load_config, read_sensor_depth
from monocular_depth_slam.depth.metrics import (
    aggregate_metrics, canonical_to_metres, depth_metrics, erode_mask_one_pixel,
    fit_prefix_scale, paired_values, resize_depth_to_reference,
)


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path,
                        default=ROOT / "configs/evaluations/da3_tum_fr1_depth.yaml")
    args = parser.parse_args()
    protocol = yaml.safe_load(args.protocol.read_text())
    sequence = load_config(ROOT / protocol["dataset_config"], project_root=ROOT)
    frames = [frame for frame in sequence.frames if frame.depth is not None]
    cache = ROOT / protocol["prediction_directory"]
    manifest = json.loads((cache / "manifest.json").read_text())
    if manifest["status"] != "complete" or manifest["requested_frames"] != len(frames):
        raise ValueError("A complete cache for every matched RGB frame is required")
    if manifest["failures"] or len(manifest["frames"]) != len(frames):
        raise ValueError("Prediction failures/missing frames must be resolved before this evaluation")
    if manifest["contract"]["dataset_config_sha256"] != sha256(ROOT / protocol["dataset_config"]):
        raise ValueError("Dataset configuration differs from the prediction cache")
    output = ROOT / protocol["output_directory"]
    output.mkdir(parents=True, exist_ok=True)
    limits = dict(min_depth=protocol["reference_min_depth_m"],
                  max_depth=protocol["reference_max_depth_m"])
    fraction = protocol["calibration_prefix_fraction"]
    if not 0 < fraction < 1:
        raise ValueError("Calibration fraction must be between 0 and 1")
    n_calibration = int(len(frames) * fraction)
    if not 0 < n_calibration < len(frames):
        raise ValueError("Calibration and held-out partitions must both be nonempty")
    raw_rows, oracle_rows, eroded_rows, nominal_rows, heldout_scaled_rows = [], [], [], [], []
    per_frame, global_scale = [], None
    bins = np.asarray(protocol["depth_bin_edges_m"], dtype=np.float64)
    bin_counts = np.zeros(len(bins)-1, dtype=np.int64)
    bin_abs = np.zeros(len(bins)-1)
    bin_rel = np.zeros(len(bins)-1)
    bin_bias = np.zeros(len(bins)-1)
    histogram = np.zeros((120, 120), dtype=np.int64)
    hist_edges = np.linspace(0, limits["max_depth"], 121)
    relative_histogram = np.zeros(100, dtype=np.int64)
    relative_edges = np.linspace(-1, 1, 101)
    nominal_scale = 525 / ((sequence.camera.fx + sequence.camera.fy) / 2)
    exemplars = set(protocol["illustration_matched_frame_indices"])
    for i, frame in enumerate(frames):
        stamp = frame.rgb.timestamp_text
        cache_record = manifest["frames"][stamp]
        file = cache / f"{stamp}.npz"
        if cache_record["cache_sha256"] != sha256(file):
            raise ValueError(f"Cache SHA-256 mismatch for {stamp}")
        if cache_record["rgb_sha256"] != sha256(frame.rgb.path):
            raise ValueError(f"RGB input changed for {stamp}")
        if cache_record["depth_timestamp_for_evaluation"] != frame.depth.timestamp_text:
            raise ValueError(f"Sensor association changed for {stamp}")
        data = np.load(file)
        canonical, matrix = data["raw_canonical_depth"], data["processed_intrinsics"]
        expected_matrix = sequence.camera.matrix.copy()
        expected_matrix[0] *= canonical.shape[1] / sequence.camera.width
        expected_matrix[1] *= canonical.shape[0] / sequence.camera.height
        np.testing.assert_allclose(matrix, expected_matrix, rtol=1e-6, atol=1e-5)
        prediction = resize_depth_to_reference(canonical_to_metres(canonical, matrix),
                                                (sequence.camera.height, sequence.camera.width))
        sensor = read_sensor_depth(frame.depth, sequence.camera)
        raw = depth_metrics(sensor.metres, prediction, **limits)
        if raw["scored_pixels"] == 0:
            raise ValueError(f"No valid prediction/reference intersection for frame {stamp}")
        raw_rows.append(raw)
        oracle_scale = raw["median_gt_over_pred"]
        oracle = depth_metrics(sensor.metres, prediction * oracle_scale, **limits)
        oracle_rows.append(oracle)
        reference_mask = (sensor.valid_mask & (sensor.metres >= limits["min_depth"]) &
                          (sensor.metres <= limits["max_depth"]))
        eroded_rows.append(depth_metrics(sensor.metres, prediction,
                                         mask=erode_mask_one_pixel(reference_mask), **limits))
        nominal_rows.append(depth_metrics(sensor.metres, prediction * nominal_scale, **limits))
        if i == n_calibration:
            global_scale = fit_prefix_scale(raw_rows, n_calibration)
        if i >= n_calibration:
            heldout_scaled_rows.append(depth_metrics(sensor.metres, prediction * global_scale,
                                                       **limits))
        groundtruth, predicted, _ = paired_values(sensor.metres, prediction, **limits)
        indices = np.minimum(np.searchsorted(bins, groundtruth, side="right")-1, len(bins)-2)
        error = predicted - groundtruth
        bin_counts += np.bincount(indices, minlength=len(bin_counts))
        bin_abs += np.bincount(indices, weights=np.abs(error), minlength=len(bin_counts))
        bin_rel += np.bincount(indices, weights=np.abs(error)/groundtruth,
                               minlength=len(bin_counts))
        bin_bias += np.bincount(indices, weights=error, minlength=len(bin_counts))
        histogram += np.histogram2d(groundtruth, predicted, bins=(hist_edges, hist_edges))[0].astype(int)
        relative_histogram += np.histogram(error/groundtruth, bins=relative_edges)[0]
        if i in exemplars:
            np.savez_compressed(output / f"example_{i:03d}.npz", reference_metres=sensor.metres,
                                prediction_metres=prediction, valid_reference=reference_mask)
        per_frame.append({"matched_frame_index": i, "rgb_timestamp": stamp,
                          "depth_timestamp": frame.depth.timestamp_text,
                          "time_seconds": float(Decimal(stamp)-Decimal(frames[0].rgb.timestamp_text)),
                          "timestamp_difference_s": float(abs(Decimal(stamp)-Decimal(frame.depth.timestamp_text))),
                          "sensor_depth_sha256": sha256(frame.depth.path),
                          "raw": raw, "oracle_per_frame": oracle,
                          "oracle_per_frame_scale": oracle_scale,
                          "canonical_to_metres_factor": float((matrix[0, 0]+matrix[1, 1])/600),
                          "inference_wall_seconds": cache_record["inference_wall_seconds"]})
        if (i+1) % 100 == 0 or i+1 == len(frames):
            print(f"Evaluated {i+1}/{len(frames)} matched frames", flush=True)
    depth_bins = [
        {"lower_m": float(bins[i]), "upper_m": float(bins[i+1]), "pixels": int(count),
         "mae_m": float(bin_abs[i]/count) if count else None,
         "abs_rel": float(bin_rel[i]/count) if count else None,
         "bias_m": float(bin_bias[i]/count) if count else None}
        for i, count in enumerate(bin_counts)
    ]
    inference_times = [row["inference_wall_seconds"] for row in per_frame]
    summary = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "protocol": protocol, "protocol_sha256": sha256(args.protocol),
        "evaluator_sha256": sha256(Path(__file__)),
        "metrics_module_sha256": sha256(ROOT / "src/monocular_depth_slam/depth/metrics.py"),
        "cache_contract": manifest["contract"],
        "dataset_index_sha256": {name: sha256(sequence.root/name)
                                 for name in ("rgb.txt", "depth.txt", "groundtruth.txt")},
        "camera": sequence.summary()["camera"],
        "primary_unfitted_metric": aggregate_metrics(raw_rows),
        "oracle_per_frame_median_scale_DIAGNOSTIC_USES_TEST_GT": aggregate_metrics(oracle_rows),
        "heldout_gt_anchored_scale_DIAGNOSTIC": {
            "calibration_frames": n_calibration, "evaluation_frames": len(frames)-n_calibration,
            "calibration_last_rgb_timestamp": frames[n_calibration-1].rgb.timestamp_text,
            "evaluation_first_rgb_timestamp": frames[n_calibration].rgb.timestamp_text,
            "fitted_global_scale": global_scale,
            "fit": "median of per-frame median GT/pred ratios in calibration prefix only",
            "raw_heldout": aggregate_metrics(raw_rows[n_calibration:]),
            "scaled_heldout": aggregate_metrics(heldout_scaled_rows),
            "independent_sequence_validation": False,
        },
        "reference_mask_eroded_one_pixel_sensitivity": aggregate_metrics(eroded_rows),
        "nominal_525px_focal_sensitivity": {"scale_relative_to_primary": nominal_scale,
                                          "metrics": aggregate_metrics(nominal_rows)},
        "by_sensor_depth": depth_bins,
        "timestamp_difference_s": {"median": float(np.median([r["timestamp_difference_s"] for r in per_frame])),
                                    "max": max(r["timestamp_difference_s"] for r in per_frame)},
        "timing": {"total_inference_seconds": sum(inference_times),
                   "median_frame_seconds": float(np.median(inference_times)),
                   "not_a_realtime_benchmark": True},
        "illustration_frames": [per_frame[i] for i in sorted(exemplars)],
        "metric_conversion_implemented_and_checked": True,
        "general_metric_accuracy_validated": False,
        "groundtruth_used_for_primary_scale_fitting": False,
    }
    (output/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    (output/"per_frame.json").write_text(json.dumps(per_frame, indent=2, allow_nan=False)+"\n")
    with (output/"per_frame.csv").open("w", newline="") as stream:
        rows = [{**{k: row[k] for k in ("rgb_timestamp", "depth_timestamp", "time_seconds",
                                         "oracle_per_frame_scale")}, **row["raw"]}
                for row in per_frame]
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(output/"histograms.npz", gt_pred_counts=histogram,
                        gt_pred_edges=hist_edges, signed_relative_counts=relative_histogram,
                        signed_relative_edges=relative_edges)
    print(json.dumps({"primary": summary["primary_unfitted_metric"]["frame_mean"],
                      "prefix_scale": global_scale}, indent=2))
