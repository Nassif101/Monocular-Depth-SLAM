#!/usr/bin/env python3
"""Prepare explicit sensor-control float32 TIFF depth; preserve holes, units and timing."""
import argparse
from datetime import datetime, timezone
from decimal import Decimal
import json
from pathlib import Path

import numpy as np
import yaml

from monocular_depth_slam.datasets.tum import load_config, read_sensor_depth
from monocular_depth_slam.depth.orb_adapter import save_float_depth
from monocular_depth_slam.slam.inputs import selected_frames
from run_monocular import sha256


ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=ROOT/"configs/orbslam3/control_792/sensor_rgbd.yaml")
    args = parser.parse_args()
    protocol = yaml.safe_load(args.protocol.read_text())
    if protocol["depth_source"] != "sensor" or protocol["frame_selection"] != "rgb_with_sensor_depth_match":
        raise ValueError("Only the explicit sensor-matched control is implemented")
    dataset_config = ROOT/protocol["dataset_config"]
    sequence = load_config(dataset_config, project_root=ROOT)
    frames = selected_frames(sequence, protocol)
    output = ROOT/protocol["prepared_depth_directory"]
    output.mkdir(parents=True, exist_ok=True)
    contract = {"protocol_sha256": sha256(args.protocol), "depth_source": "sensor",
                "dataset_config_sha256": sha256(dataset_config),
                "preparer_sha256": sha256(Path(__file__)),
                "sensor_reader_sha256": sha256(ROOT/"src/monocular_depth_slam/datasets/tum.py"),
                "writer_sha256": sha256(ROOT/"src/monocular_depth_slam/depth/orb_adapter.py"),
                "units": "sensor z-depth metres; PNG / 5000; holes=0; no additional Freiburg correction",
                "format": "single-channel float32 TIFF, uncompressed; invalid=0",
                "groundtruth_pose_inputs": False, "depth_fitting_or_hole_filling": False}
    manifest_path = output/"manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {"contract": contract, "frames": {}}
    if manifest["contract"] != contract:
        raise ValueError("Sensor preparation provenance changed; choose a new output directory")
    manifest.update(status="running", requested_frames=len(frames))
    manifest_path.write_text(json.dumps(manifest, indent=2)+"\n")
    for i, frame in enumerate(frames):
        stamp = frame.rgb.timestamp_text
        source_hash = sha256(frame.depth.path)
        rgb_hash = sha256(frame.rgb.path)
        destination = output/f"{stamp}.tiff"
        if stamp in manifest["frames"]:
            record = manifest["frames"][stamp]
            if record["sensor_sha256"] != source_hash or record["rgb_sha256"] != rgb_hash or record["depth_sha256"] != sha256(destination):
                raise ValueError(f"Sensor control input/output changed: {stamp}")
            continue
        sensor = read_sensor_depth(frame.depth, sequence.camera)
        depth = np.where(sensor.valid_mask, sensor.metres, 0).astype(np.float32)
        save_float_depth(destination, depth)
        manifest["frames"][stamp] = {
            "rgb_path": frame.rgb.relative_path, "rgb_sha256": rgb_hash,
            "sensor_path": str(frame.depth.path.relative_to(ROOT)), "sensor_sha256": source_hash,
            "sensor_timestamp": frame.depth.timestamp_text,
            "rgb_sensor_time_difference_s": float(abs(Decimal(stamp)-Decimal(frame.depth.timestamp_text))),
            "depth_path": str(destination.relative_to(ROOT)), "depth_sha256": sha256(destination),
            "depth_min_m": float(depth.min()), "depth_max_m": float(depth.max()),
            "depth_center_m": float(depth[depth.shape[0]//2, depth.shape[1]//2]),
            "invalid_pixels": int(np.count_nonzero(~sensor.valid_mask))}
        if (i+1) % 100 == 0 or i+1 == len(frames):
            manifest_path.write_text(json.dumps(manifest, indent=2)+"\n")
            print(f"{i+1}/{len(frames)} sensor depth maps; holes retained", flush=True)
    manifest.update(status="complete", completed_utc=datetime.now(timezone.utc).isoformat(),
                    total_depth_bytes=sum((ROOT/row["depth_path"]).stat().st_size for row in manifest["frames"].values()))
    manifest_path.write_text(json.dumps(manifest, indent=2)+"\n")
