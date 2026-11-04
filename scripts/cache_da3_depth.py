#!/usr/bin/env python3
"""Cache single-image canonical DA3 depth and the exact processed camera matrix."""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np
import torch
import yaml
from depth_anything_3.api import DepthAnything3

from monocular_depth_slam.datasets.tum import load_config


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def synchronize(device):
    if device == "mps":
        torch.mps.synchronize()
    elif device == "cuda":
        torch.cuda.synchronize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-config", type=Path,
                        default=ROOT / "configs/datasets/tum_rgbd.example.yaml")
    parser.add_argument("--checkpoint-config", type=Path,
                        default=ROOT / "configs/da3/metric_large.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "predictions/da3_metric_large/fr1_xyz")
    parser.add_argument("--device", choices=("mps", "cuda", "cpu"), default="mps")
    parser.add_argument("--limit", type=int, help="Debug only: process the first N matched RGB frames")
    args = parser.parse_args()
    config = yaml.safe_load(args.checkpoint_config.read_text())
    if config["process_res_method"] != "upper_bound_resize":
        raise ValueError("This comparison supports pure resize, without cropping")
    sequence = load_config(args.dataset_config, project_root=ROOT)
    frames = [frame for frame in sequence.frames if frame.depth is not None]
    if args.limit is not None:
        if args.limit <= 0:
            parser.error("limit must be positive")
        frames = frames[:args.limit]
    source = ROOT / "third_party/Depth-Anything-3"
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"],
                                       text=True).strip()
    if revision != config["source_revision"]:
        raise ValueError("Upstream source does not match checkpoint configuration")
    source_diff = subprocess.check_output(["git", "-C", str(source), "diff"])
    directory = ROOT / config["local_directory"]
    file_hashes = {}
    for name, metadata in config["files"].items():
        file = directory / name
        file_hashes[name] = sha256(file)
        if file.stat().st_size != metadata["size_bytes"] or file_hashes[name] != metadata["sha256"]:
            raise ValueError(f"Checkpoint file mismatch: {file}")
    contract = {
        "dataset_config_sha256": sha256(args.dataset_config),
        "checkpoint_config_sha256": sha256(args.checkpoint_config),
        "runner_sha256": sha256(Path(__file__)), "source_revision": revision,
        "source_diff_sha256": hashlib.sha256(source_diff).hexdigest(),
        "checkpoint": config["checkpoint"], "checkpoint_revision": config["checkpoint_revision"],
        "checkpoint_files": file_hashes, "device": args.device, "seed": 0,
        "packages": {name: importlib.metadata.version(name)
                     for name in ("torch", "torchvision", "numpy", "pillow", "depth-anything-3")},
        "platform": platform.platform(), "raw_depth_convention": "canonical_z_depth",
        "process_res": config["process_res"], "process_res_method": config["process_res_method"],
        "inference_inputs": "one RGB image; no sensor depth, poses or intrinsics passed to network",
        "processed_intrinsics": "upstream InputProcessor transforms known RGB K for unit conversion",
    }
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.output / "manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest["contract"] != contract:
            raise ValueError("Cache provenance mismatch; use a new output directory")
    else:
        manifest = {"contract": contract, "frames": {}, "failures": {}}
    manifest.update(requested_frames=len(frames), status="running")
    write_json(manifest_path, manifest)
    torch.manual_seed(0)
    np.random.seed(0)
    model = DepthAnything3.from_pretrained(str(directory), local_files_only=True).to(args.device).eval()
    if model.model_name != config["model_variant"]:
        raise ValueError("Checkpoint model variant mismatch")
    for i, frame in enumerate(frames):
        stamp = frame.rgb.timestamp_text
        file = args.output / f"{stamp}.npz"
        image_hash = sha256(frame.rgb.path)
        if stamp in manifest["frames"] and file.exists():
            record = manifest["frames"][stamp]
            if record["rgb_sha256"] != image_hash or record["cache_sha256"] != sha256(file):
                raise ValueError(f"Cached input/output hash mismatch for {stamp}")
            continue
        try:
            synchronize(args.device)
            start = time.perf_counter()
            images, _, matrices = model.input_processor(
                [str(frame.rgb.path)], intrinsics=sequence.camera.matrix[None],
                process_res=config["process_res"], process_res_method=config["process_res_method"],
                num_workers=1, sequential=True,
            )
            height, width = images.shape[-2:]
            matrix = matrices[0].numpy()
            expected = sequence.camera.matrix.copy()
            expected[0] *= width / sequence.camera.width
            expected[1] *= height / sequence.camera.height
            np.testing.assert_allclose(matrix, expected, rtol=1e-6, atol=1e-5)
            with torch.inference_mode():
                raw = model(images[None].to(args.device).float(), export_feat_layers=[])
                prediction = model.output_processor(raw)
            synchronize(args.device)
            elapsed = time.perf_counter() - start
            canonical = np.asarray(prediction.depth[0], dtype=np.float32)
            if canonical.shape != (height, width):
                raise ValueError("Output grid does not match the processed image")
            temporary = file.with_suffix(".npz.tmp")
            with temporary.open("wb") as stream:
                np.savez_compressed(stream, raw_canonical_depth=canonical,
                                    processed_intrinsics=matrix)
            temporary.replace(file)
            manifest["frames"][stamp] = {
                "rgb_timestamp": stamp, "rgb_path": frame.rgb.relative_path,
                "rgb_sha256": image_hash, "cache_sha256": sha256(file),
                "depth_timestamp_for_evaluation": frame.depth.timestamp_text,
                "processed_shape_hw": [height, width], "inference_wall_seconds": elapsed,
            }
            manifest["failures"].pop(stamp, None)
        except Exception as error:
            manifest["failures"][stamp] = {"error": f"{type(error).__name__}: {error}"}
            write_json(manifest_path, manifest)
            raise
        if (i + 1) % 10 == 0 or i == 0 or i + 1 == len(frames):
            manifest["updated_utc"] = datetime.now(timezone.utc).isoformat()
            write_json(manifest_path, manifest)
            print(f"{i+1}/{len(frames)} cached; last inference {elapsed:.3f}s", flush=True)
    manifest.update(status="complete", completed_utc=datetime.now(timezone.utc).isoformat())
    write_json(manifest_path, manifest)
    print(f"Completed {len(frames)} matched-frame predictions", flush=True)
