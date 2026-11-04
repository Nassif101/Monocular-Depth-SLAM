#!/usr/bin/env python3
"""Verify DA3 caches and prepare float32 depth for every RGB input, without GT fitting."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from monocular_depth_slam.datasets.tum import load_config
from monocular_depth_slam.depth.orb_adapter import aligned_orb_depth, save_float_depth


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=ROOT/"configs/orbslam3/da3_rgbd_baseline.yaml")
    args = parser.parse_args()
    protocol = yaml.safe_load(args.protocol.read_text())
    if protocol["depth_scale"] != 1 or protocol["groundtruth_calibration"] or protocol["depth_range_clipping"] != "none" or protocol["confidence_filtering"] != "none":
        raise ValueError("The initial pipeline only implements fixed, uncalibrated depth")
    if protocol["depth_conversion"] != "canonical_times_processed_mean_focal_over_300" or protocol["depth_upsampling"] != "bilinear_pixel_centres" or protocol["invalid_depth"] != "nonfinite_or_nonpositive_to_zero":
        raise ValueError("Unsupported depth conversion protocol")
    dataset_config = ROOT/protocol["dataset_config"]
    checkpoint_config = ROOT/protocol["checkpoint_config"]
    checkpoint = yaml.safe_load(checkpoint_config.read_text())
    sequence = load_config(dataset_config, project_root=ROOT)
    records, sources, contracts = {}, {}, []
    for directory in protocol["prediction_directories"]:
        path = ROOT/directory
        manifest = json.loads((path/"manifest.json").read_text())
        contract = manifest["contract"]
        if manifest["status"] != "complete":
            raise ValueError(f"Incomplete prediction cache: {directory}")
        if contract["dataset_config_sha256"] != sha256(dataset_config) or contract["checkpoint_config_sha256"] != sha256(checkpoint_config):
            raise ValueError("Cached calibration/checkpoint differs from the protocol")
        if contract["source_revision"] != checkpoint["source_revision"] or contract["checkpoint_revision"] != checkpoint["checkpoint_revision"]:
            raise ValueError("Prediction source/checkpoint pin mismatch")
        if contract["process_res_method"] != "upper_bound_resize" or contract["raw_depth_convention"] != "canonical_z_depth":
            raise ValueError("Unsupported cached depth/grid convention")
        comparable = {key: value for key, value in contract.items()
                      if key not in ("runner_sha256", "base_manifest_sha256", "selection")}
        if contracts and comparable != contracts[0]:
            raise ValueError("Prediction environments/settings differ across caches")
        contracts.append(comparable)
        sources[directory] = sha256(path/"manifest.json")
        for stamp, record in manifest["frames"].items():
            if stamp in records:
                raise ValueError(f"Duplicate prediction timestamp: {stamp}")
            records[stamp] = (path, record)
    output = ROOT/protocol["prepared_depth_directory"]
    output.mkdir(parents=True, exist_ok=True)
    contract = {"protocol_sha256": sha256(args.protocol), "prediction_manifest_sha256": sources,
                "preparer_sha256": sha256(Path(__file__)),
                "adapter_sha256": sha256(ROOT/"src/monocular_depth_slam/depth/orb_adapter.py"),
                "depth_metrics_sha256": sha256(ROOT/"src/monocular_depth_slam/depth/metrics.py"),
                "dataset_config_sha256": sha256(dataset_config),
                "units": "nominal metres from fixed DA3 conversion; no GT scale fit",
                "format": "single-channel float32 TIFF, uncompressed; invalid=0",
                "gt_depth_or_poses_used_for_conversion": False}
    manifest_path = output/"manifest.json"
    result = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"contract": contract, "frames": {}}
    if result["contract"] != contract:
        raise ValueError("Prepared cache provenance differs; choose a new directory")
    result.update(status="running", requested_frames=len(sequence.frames))
    manifest_path.write_text(json.dumps(result, indent=2)+"\n")
    for i, frame in enumerate(sequence.frames):
        stamp = frame.rgb.timestamp_text
        directory, record = records[stamp]
        cached = directory/f"{stamp}.npz"
        if record["rgb_path"] != frame.rgb.relative_path or record["rgb_sha256"] != sha256(frame.rgb.path) or record["cache_sha256"] != sha256(cached):
            raise ValueError(f"Cached RGB or prediction changed: {stamp}")
        file = output/f"{stamp}.tiff"
        if stamp in result["frames"]:
            if sha256(file) != result["frames"][stamp]["depth_sha256"]:
                raise ValueError(f"Prepared depth changed: {stamp}")
            continue
        with np.load(cached, allow_pickle=False) as archive:
            depth = aligned_orb_depth(archive["raw_canonical_depth"], archive["processed_intrinsics"],
                                      sequence.camera.matrix, (sequence.camera.height, sequence.camera.width))
        save_float_depth(file, depth)
        result["frames"][stamp] = {
            "rgb_path": frame.rgb.relative_path, "rgb_sha256": record["rgb_sha256"],
            "canonical_path": str(cached.relative_to(ROOT)), "canonical_sha256": record["cache_sha256"],
            "depth_path": str(file.relative_to(ROOT)), "depth_sha256": sha256(file),
            "depth_min_m": float(depth.min()), "depth_max_m": float(depth.max()),
            "depth_center_m": float(depth[depth.shape[0]//2, depth.shape[1]//2]),
            "invalid_pixels": int(np.count_nonzero(depth == 0)),
            "inference_wall_seconds": record["inference_wall_seconds"]}
        if (i+1) % 100 == 0 or i+1 == len(sequence.frames):
            manifest_path.write_text(json.dumps(result, indent=2)+"\n")
            print(f"{i+1}/{len(sequence.frames)} RGB-aligned float32 depth maps", flush=True)
    result.update(status="complete", completed_utc=datetime.now(timezone.utc).isoformat(),
                  total_depth_bytes=sum((ROOT/record["depth_path"]).stat().st_size for record in result["frames"].values()))
    manifest_path.write_text(json.dumps(result, indent=2)+"\n")
    print(f"Prepared {len(result['frames'])} learned depth maps; no GT calibration", flush=True)
