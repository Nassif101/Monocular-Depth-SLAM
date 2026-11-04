#!/usr/bin/env python3
"""Run the pinned local DA3 checkpoint on one TUM RGB frame, without ground truth."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import torch
import yaml
from depth_anything_3.api import DepthAnything3

from monocular_depth_slam.datasets.tum import load_config


ROOT = Path(__file__).resolve().parents[1]


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-config", type=Path,
                        default=ROOT / "configs/datasets/tum_rgbd.example.yaml")
    parser.add_argument("--checkpoint-config", type=Path,
                        default=ROOT / "configs/da3/metric_large.yaml")
    parser.add_argument("--device", choices=("auto", "cpu", "mps", "cuda"), default="auto")
    parser.add_argument("--frame-index", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/checkpoint_smoke")
    args = parser.parse_args()
    config = yaml.safe_load(args.checkpoint_config.read_text())
    sequence = load_config(args.dataset_config, project_root=ROOT)
    if not 0 <= args.frame_index < len(sequence.rgb_records):
        parser.error("frame-index must select an existing RGB frame")
    frame = sequence.rgb_records[args.frame_index]
    directory = ROOT / config["local_directory"]
    hashes = {}
    for name, metadata in config["files"].items():
        path = directory / name
        if path.stat().st_size != metadata["size_bytes"]:
            raise ValueError(f"Unexpected checkpoint file size: {path}")
        hashes[name] = file_sha256(path)
        if metadata.get("sha256") and hashes[name] != metadata["sha256"]:
            raise ValueError(f"Checkpoint SHA-256 mismatch: {path}")
    source = ROOT / "third_party/Depth-Anything-3"
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"],
                                       text=True).strip()
    if revision != config["source_revision"]:
        raise ValueError(f"DA3 source revision does not match the pin: {revision}")
    source_diff = subprocess.check_output(["git", "-C", str(source), "diff"])
    device = args.device
    if device == "auto":
        device = ("cuda" if torch.cuda.is_available() else
                  "mps" if torch.backends.mps.is_available() else "cpu")
    torch.manual_seed(0)
    np.random.seed(0)
    model = DepthAnything3.from_pretrained(str(directory), local_files_only=True).to(device)
    model.eval()
    if model.model_name != config["model_variant"]:
        raise ValueError(f"Unexpected model variant: {model.model_name}")
    if device == "mps":
        torch.mps.synchronize()
    elif device == "cuda":
        torch.cuda.synchronize()
    start = time.perf_counter()
    prediction = model.inference(
        [str(frame.path)], extrinsics=None, intrinsics=None,
        align_to_input_ext_scale=False, infer_gs=False,
        process_res=config["process_res"], process_res_method=config["process_res_method"],
    )
    if device == "mps":
        torch.mps.synchronize()
    elif device == "cuda":
        torch.cuda.synchronize()
    elapsed = time.perf_counter() - start
    depth = np.asarray(prediction.depth, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0)
    if depth.ndim != 3 or depth.shape[0] != 1 or not valid.any():
        raise ValueError("Checkpoint did not produce a valid single-frame depth map")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"{frame.timestamp_text}.npz"
    np.savez_compressed(output, raw_canonical_depth=depth[0], valid_mask=valid[0])
    report = {
        "dataset": "tum_rgbd", "sequence": sequence.root.name,
        "rgb_timestamp": frame.timestamp_text, "rgb_path": frame.relative_path,
        "rgb_sha256": file_sha256(frame.path),
        "checkpoint": config["checkpoint"], "checkpoint_revision": config["checkpoint_revision"],
        "checkpoint_file_sha256": hashes, "source_revision": revision,
        "source_diff_sha256": hashlib.sha256(source_diff).hexdigest(),
        "model_variant": model.model_name, "device": device,
        "torch_version": torch.__version__, "seed": 0,
        "parameter_count": sum(parameter.numel() for parameter in model.parameters()),
        "process_res": config["process_res"], "process_res_method": config["process_res_method"],
        "original_image_size_hw": [sequence.camera.height, sequence.camera.width],
        "processed_depth_shape": list(depth.shape),
        "original_intrinsics": sequence.camera.matrix.tolist(),
        "output_convention": config["raw_output_convention"],
        "metre_conversion_applied": False,
        "metre_conversion_required": config["metre_conversion"],
        "metric_scale_validated": False,
        "groundtruth_passed_to_model": False,
        "camera_poses_passed_to_model": False,
        "sensor_depth_passed_to_model": False,
        "valid_depth_fraction": float(valid.mean()),
        "raw_canonical_depth_range": [float(depth[valid].min()), float(depth[valid].max())],
        "inference_wall_seconds_including_preprocessing": elapsed,
        "output_file": str(output.relative_to(ROOT)) if output.is_relative_to(ROOT) else str(output),
        "depth_accuracy_evaluated": False,
    }
    result = json.dumps(report, indent=2, allow_nan=False) + "\n"
    (args.output_dir / "metadata.json").write_text(result)
    print(result, end="")
