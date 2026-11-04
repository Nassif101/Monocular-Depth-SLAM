#!/usr/bin/env python3
"""Supplement a verified matched-frame cache with missing RGB-only predictions."""
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

from cache_da3_depth import sha256, synchronize, write_json
from monocular_depth_slam.datasets.tum import load_config


ROOT = Path(__file__).resolve().parents[1]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=ROOT/"predictions/da3_metric_large/fr1_xyz")
    parser.add_argument("--output", type=Path, default=ROOT/"predictions/da3_metric_large/fr1_xyz_extra_rgb")
    args = parser.parse_args()
    base = json.loads((args.base/"manifest.json").read_text())
    if base["status"] != "complete":
        raise ValueError("The original matched-frame cache must be complete")
    config_path = ROOT/"configs/da3/metric_large.yaml"
    dataset_path = ROOT/"configs/datasets/tum_rgbd.example.yaml"
    config = yaml.safe_load(config_path.read_text())
    sequence = load_config(dataset_path, project_root=ROOT)
    contract = base["contract"]
    if sha256(config_path) != contract["checkpoint_config_sha256"] or sha256(dataset_path) != contract["dataset_config_sha256"]:
        raise ValueError("Dataset/checkpoint configuration differs from the original cache")
    source = ROOT/"third_party/Depth-Anything-3"
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    diff = subprocess.check_output(["git", "-C", str(source), "diff"])
    packages = {name: importlib.metadata.version(name) for name in contract["packages"]}
    if revision != contract["source_revision"] or hashlib.sha256(diff).hexdigest() != contract["source_diff_sha256"]:
        raise ValueError("DA3 source differs from the original cache")
    if packages != contract["packages"] or platform.platform() != contract["platform"]:
        raise ValueError("Use the same inference environment/device or create a separate full cache")
    device = contract["device"]
    directory = ROOT/config["local_directory"]
    for name, expected in contract["checkpoint_files"].items():
        if sha256(directory/name) != expected:
            raise ValueError(f"Checkpoint mismatch: {name}")
    frames = []
    for frame in sequence.frames:
        stamp = frame.rgb.timestamp_text
        if stamp in base["frames"]:
            record = base["frames"][stamp]
            if record["rgb_sha256"] != sha256(frame.rgb.path) or record["cache_sha256"] != sha256(args.base/f"{stamp}.npz"):
                raise ValueError(f"Original cached input/output differs: {stamp}")
        else:
            frames.append(frame)
    supplemental_contract = {**contract, "runner_sha256": sha256(Path(__file__)),
                             "base_manifest_sha256": sha256(args.base/"manifest.json"),
                             "selection": "all RGB timestamps absent from the original cache"}
    args.output.mkdir(parents=True, exist_ok=True)
    output_manifest = args.output/"manifest.json"
    manifest = json.loads(output_manifest.read_text()) if output_manifest.exists() else {
        "contract": supplemental_contract, "frames": {}, "failures": {}}
    if manifest["contract"] != supplemental_contract:
        raise ValueError("Supplemental cache provenance differs; choose a new directory")
    manifest.update(status="running", requested_frames=len(frames))
    write_json(output_manifest, manifest)
    torch.manual_seed(contract["seed"])
    np.random.seed(contract["seed"])
    model = DepthAnything3.from_pretrained(str(directory), local_files_only=True).to(device).eval()
    if model.model_name != config["model_variant"]:
        raise ValueError("Checkpoint variant mismatch")
    for i, frame in enumerate(frames):
        stamp = frame.rgb.timestamp_text
        output = args.output/f"{stamp}.npz"
        image_hash = sha256(frame.rgb.path)
        if stamp in manifest["frames"]:
            record = manifest["frames"][stamp]
            if record["rgb_sha256"] != image_hash or record["cache_sha256"] != sha256(output):
                raise ValueError(f"Supplemental cached input/output differs: {stamp}")
            continue
        try:
            synchronize(device)
            start = time.perf_counter()
            images, _, matrices = model.input_processor(
                [str(frame.rgb.path)], intrinsics=sequence.camera.matrix[None],
                process_res=config["process_res"], process_res_method=config["process_res_method"],
                num_workers=1, sequential=True)
            height, width = images.shape[-2:]
            matrix = matrices[0].numpy()
            expected = sequence.camera.matrix.copy()
            expected[0] *= width/sequence.camera.width
            expected[1] *= height/sequence.camera.height
            np.testing.assert_allclose(matrix, expected, rtol=1e-6, atol=1e-5)
            with torch.inference_mode():
                prediction = model.output_processor(model(images[None].to(device).float(), export_feat_layers=[]))
            synchronize(device)
            elapsed = time.perf_counter()-start
            canonical = np.asarray(prediction.depth[0], dtype=np.float32)
            if canonical.shape != (height, width):
                raise ValueError("Prediction/processed image shapes differ")
            temporary = output.with_suffix(".npz.tmp")
            with temporary.open("wb") as stream:
                np.savez_compressed(stream, raw_canonical_depth=canonical, processed_intrinsics=matrix)
            temporary.replace(output)
            manifest["frames"][stamp] = {
                "rgb_timestamp": stamp, "rgb_path": frame.rgb.relative_path,
                "rgb_sha256": image_hash, "cache_sha256": sha256(output),
                "processed_shape_hw": [height, width], "inference_wall_seconds": elapsed}
            print(f"{i+1}/{len(frames)} supplemental RGB predictions; {elapsed:.3f}s", flush=True)
            manifest["failures"].pop(stamp, None)
        except Exception as error:
            manifest["failures"][stamp] = {"error": f"{type(error).__name__}: {error}"}
            write_json(output_manifest, manifest)
            raise
        write_json(output_manifest, manifest)
    manifest.update(status="complete", completed_utc=datetime.now(timezone.utc).isoformat())
    write_json(output_manifest, manifest)
