#!/usr/bin/env python3
"""Validate TUM indices, paths, calibration and sensor images without model inference."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from monocular_depth_slam.datasets.tum import load_config, read_rgb, read_sensor_depth


ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/datasets/tum_rgbd.example.yaml")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--write-associations", type=Path)
    parser.add_argument("--all-images", action="store_true", help="Decode every RGB and depth PNG")
    args = parser.parse_args()
    sequence = load_config(args.config, project_root=ROOT)
    report = sequence.summary()
    config = yaml.safe_load(args.config.read_text())
    report["index_sha256"] = {
        config[key]: hashlib.sha256((sequence.root / config[key]).read_bytes()).hexdigest()
        for key in ("rgb_index", "depth_index", "groundtruth_index")
    }
    if args.all_images:
        rgb = sequence.rgb_records
        depth = sequence.depth_records
    else:
        rgb = tuple(sequence.rgb_records[i] for i in sorted({0, len(sequence.rgb_records)//2,
                                                           len(sequence.rgb_records)-1}))
        depth = tuple(sequence.depth_records[i] for i in sorted({0, len(sequence.depth_records)//2,
                                                               len(sequence.depth_records)-1}))
    for record in rgb:
        read_rgb(record, sequence.camera)
    valid_pixels, total_pixels = 0, 0
    minimum, maximum = float("inf"), 0.0
    for record in depth:
        image = read_sensor_depth(record, sequence.camera)
        valid_pixels += int(image.valid_mask.sum())
        total_pixels += image.valid_mask.size
        if image.valid_mask.any():
            values = image.metres[image.valid_mask]
            minimum = min(minimum, float(values.min()))
            maximum = max(maximum, float(values.max()))
    report["image_validation"] = {
        "rgb_decoded": len(rgb), "depth_decoded": len(depth),
        "all_images": args.all_images,
        "valid_sensor_depth_fraction": valid_pixels / total_pixels,
        "sensor_depth_range_metres": [minimum if np.isfinite(minimum) else None,
                                      maximum if valid_pixels else None],
    }
    if args.write_associations:
        sequence.write_rgb_depth_associations(args.write_associations)
    result = json.dumps(report, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(result)
    print(result, end="")
