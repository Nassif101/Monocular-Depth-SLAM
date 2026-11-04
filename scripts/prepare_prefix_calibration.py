#!/usr/bin/env python3
"""Reproduce a prefix-only depth correction and prepare labeled calibrated DA3 maps."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from monocular_depth_slam.datasets.tum import load_config, read_sensor_depth
from monocular_depth_slam.depth.calibration import scale_prepared_depth, temporal_partition
from monocular_depth_slam.depth.metrics import fit_prefix_scale, paired_values
from monocular_depth_slam.depth.orb_adapter import save_float_depth
from monocular_depth_slam.slam.inputs import selected_frames
from run_monocular import sha256


ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"configs/evaluations/prefix_calibration.yaml")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    dataset_config = ROOT/config["dataset_config"]
    sequence = load_config(dataset_config, project_root=ROOT)
    frames = selected_frames(sequence, config)
    prefix, later = temporal_partition(frames, config["calibration_frames"])
    if len(frames) != config["expected_input_frames"] or len(later) != config["expected_evaluation_frames"]:
        raise ValueError("Calibration cohort differs from the frozen protocol")
    source = ROOT/config["base_prepared_depth"]
    base = json.loads((source/"manifest.json").read_text())
    if base["status"] != "complete" or base["contract"]["dataset_config_sha256"] != sha256(dataset_config):
        raise ValueError("Base predicted depth is incomplete or uses different calibration")
    prior_path = ROOT/config["prior_depth_summary"]
    prior = json.loads(prior_path.read_text())["heldout_gt_anchored_scale_DIAGNOSTIC"]
    if prior["calibration_frames"] != len(prefix) or prior["calibration_last_rgb_timestamp"] != prefix[-1].rgb.timestamp_text:
        raise ValueError("Original depth-study calibration prefix differs")
    output = ROOT/config["prepared_depth_directory"]
    output.mkdir(parents=True, exist_ok=True)
    fit_rows = []
    # Only this loop opens sensor-depth pixels, and only for the frozen prefix.
    for frame in prefix:
        record = base["frames"][frame.rgb.timestamp_text]
        if sha256(ROOT/record["depth_path"]) != record["depth_sha256"] or sha256(frame.rgb.path) != record["rgb_sha256"]:
            raise ValueError("Prefix prediction/RGB changed")
        sensor = read_sensor_depth(frame.depth, sequence.camera)
        predicted = np.asarray(Image.open(ROOT/record["depth_path"]), dtype=np.float32)
        gt, pred, pixels = paired_values(sensor.metres, predicted,
                                          min_depth=config["reference_min_depth_m"], max_depth=config["reference_max_depth_m"])
        ratio = float(np.median(gt/pred))
        fit_rows.append({"rgb_timestamp": frame.rgb.timestamp_text, "sensor_timestamp": frame.depth.timestamp_text,
                         "sensor_path": str(frame.depth.path.relative_to(ROOT)), "sensor_sha256": sha256(frame.depth.path),
                         "prediction_sha256": record["depth_sha256"], "reference_pixels": pixels,
                         "median_gt_over_pred": ratio})
    scale = fit_prefix_scale(fit_rows, len(prefix))
    if not math.isclose(scale, config["frozen_depth_scale"], rel_tol=0, abs_tol=1e-12) or scale != prior["fitted_global_scale"]:
        raise ValueError(f"Prefix estimate differs from the previously frozen value: {scale}")
    calibration = {"scale": scale, "calibration_frames": len(prefix), "later_frames": len(later),
                   "fit": config["fit"], "prefix_last_timestamp": prefix[-1].rgb.timestamp_text,
                   "evaluation_first_timestamp": later[0].rgb.timestamp_text,
                   "heldout_sensor_pixels_used_for_fit": False, "groundtruth_world_poses_used_for_fit": False,
                   "offline_replay_after_calibration": True, "prior_summary_sha256": sha256(prior_path), "prefix_records": fit_rows}
    calibration_path = output/"calibration.json"
    calibration_text = json.dumps(calibration, indent=2, allow_nan=False)+"\n"
    if calibration_path.exists() and calibration_path.read_text() != calibration_text:
        raise ValueError("Existing calibration evidence changed; choose a new directory")
    calibration_path.write_text(calibration_text)
    protocol_path = ROOT/config["replay_protocol"]
    contract = {"protocol_sha256": sha256(protocol_path), "config_sha256": sha256(args.config),
                "dataset_config_sha256": sha256(dataset_config), "depth_source": "da3",
                "groundtruth_calibration": True, "depth_scale": scale,
                "calibration_path": str(calibration_path.relative_to(ROOT)), "calibration_sha256": sha256(calibration_path),
                "base_prepared_manifest": str((source/"manifest.json").relative_to(ROOT)),
                "base_prepared_manifest_sha256": sha256(source/"manifest.json"),
                "prediction_manifest_sha256": base["contract"]["prediction_manifest_sha256"],
                "preparer_sha256": sha256(Path(__file__)),
                "calibration_module_sha256": sha256(ROOT/"src/monocular_depth_slam/depth/calibration.py"),
                "units": "DA3 nominal metres multiplied by fixed GT-prefix scale; offline diagnostic",
                "format": "single-channel float32 TIFF, uncompressed; invalid=0",
                "gt_depth_or_poses_used_for_conversion": True,
                "gt_depth_scope": "first 158 selected frames only; no GT world poses"}
    manifest_path = output/"manifest.json"
    result = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"contract": contract, "frames": {}}
    if result["contract"] != contract:
        raise ValueError("Calibrated cache provenance changed; choose a new directory")
    result.update(status="running", requested_frames=len(frames))
    manifest_path.write_text(json.dumps(result, indent=2)+"\n")
    for i, frame in enumerate(frames):
        stamp = frame.rgb.timestamp_text
        record = base["frames"][stamp]
        source_file = ROOT/record["depth_path"]
        if sha256(source_file) != record["depth_sha256"] or sha256(frame.rgb.path) != record["rgb_sha256"]:
            raise ValueError("Original predicted depth/RGB changed")
        destination = output/f"{stamp}.tiff"
        if stamp in result["frames"]:
            if sha256(destination) != result["frames"][stamp]["depth_sha256"]:
                raise ValueError("Calibrated output changed")
            continue
        with Image.open(source_file) as image:
            corrected = scale_prepared_depth(np.asarray(image), scale)
        save_float_depth(destination, corrected)
        result["frames"][stamp] = {
            "rgb_path": frame.rgb.relative_path, "rgb_sha256": record["rgb_sha256"],
            "base_depth_path": record["depth_path"], "base_depth_sha256": record["depth_sha256"],
            "depth_path": str(destination.relative_to(ROOT)), "depth_sha256": sha256(destination),
            "depth_min_m": float(corrected.min()), "depth_max_m": float(corrected.max()),
            "depth_center_m": float(corrected[corrected.shape[0]//2, corrected.shape[1]//2]),
            "invalid_pixels": int(np.count_nonzero(corrected == 0)),
            "inference_wall_seconds": record["inference_wall_seconds"]}
        if (i+1) % 100 == 0 or i+1 == len(frames):
            manifest_path.write_text(json.dumps(result, indent=2)+"\n")
            print(f"{i+1}/{len(frames)} prefix-calibrated depth maps; fixed scale {scale:.15f}", flush=True)
    result.update(status="complete", completed_utc=datetime.now(timezone.utc).isoformat(),
                  total_depth_bytes=sum((ROOT/row["depth_path"]).stat().st_size for row in result["frames"].values()))
    manifest_path.write_text(json.dumps(result, indent=2)+"\n")
