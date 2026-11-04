#!/usr/bin/env python3
"""Replay all TUM RGB inputs, optionally with prepared learned depth, in cached Linux."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import subprocess
import time

import yaml

from monocular_depth_slam.datasets.tum import load_config
from monocular_depth_slam.slam.inputs import selected_frames
from monocular_depth_slam.slam.trajectory import convert_orb_export


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(default_protocol="configs/orbslam3/monocular_baseline.yaml",
         default_run_dir="results/orbslam3_mono_fr1_xyz/run_001"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path,
                        default=ROOT/default_protocol)
    parser.add_argument("--run-dir", type=Path, default=ROOT/default_run_dir)
    args = parser.parse_args()
    run = args.run_dir.resolve()
    if not run.is_relative_to(ROOT):
        parser.error("run-dir must be inside the mounted project")
    if run.exists():
        raise FileExistsError(f"Existing run is preserved; choose a new run-dir: {run}")
    protocol = yaml.safe_load(args.protocol.read_text())
    sensor = protocol.get("sensor", "monocular")
    if sensor not in ("monocular", "rgbd"):
        raise ValueError("Only monocular and prepared learned-depth RGB-D replays are implemented")
    rgbd = sensor == "rgbd"
    depth_source = protocol.get("depth_source", "da3" if rgbd else "none")
    gt_calibrated = bool(protocol.get("groundtruth_calibration", False))
    if depth_source not in (("da3", "sensor") if rgbd else ("none",)):
        raise ValueError("Unsupported depth source for sensor mode")
    if gt_calibrated and depth_source != "da3":
        raise ValueError("Only the labeled DA3 prefix calibration diagnostic is implemented")
    for name, value in {"opencv_threads": 1, "viewer": False,
                        "replay": "paced_all_rgb_no_frame_drops"}.items():
        if protocol[name] != value:
            raise ValueError(f"Replay harness does not implement this {name} setting")
    sequence = load_config(ROOT/protocol["dataset_config"], project_root=ROOT)
    frames = selected_frames(sequence, protocol)
    rgb_records = tuple(frame.rgb for frame in frames)
    settings = ROOT/protocol["camera_settings"]
    camera = yaml.safe_load(settings.read_text().replace("%YAML:1.0", ""))
    for name in ("fx", "fy", "cx", "cy"):
        if camera[f"Camera1.{name}"] != getattr(sequence.camera, name):
            raise ValueError(f"RGB calibration mismatch: {name}")
    if camera["Camera.type"] != "PinHole":
        raise ValueError("Baseline camera model must be PinHole")
    for name, value in zip(("k1", "k2", "p1", "p2", "k3"), sequence.camera.distortion):
        if camera[f"Camera1.{name}"] != value:
            raise ValueError(f"RGB distortion mismatch: {name}")
    if camera["Camera.RGB"] != 1 or camera["Camera.width"] != sequence.camera.width or camera["Camera.height"] != sequence.camera.height:
        raise ValueError("Camera color order/dimensions do not match the replay harness")
    prepared, depth_manifest = None, None
    if rgbd:
        if camera["RGBD.DepthMapFactor"] != 1 or camera["Stereo.b"] != 0.07732 or camera["Stereo.ThDepth"] != 40:
            raise ValueError("The initial learned-depth baseline requires metre input and upstream TUM1 RGB-D parameters")
        depth_manifest = ROOT/protocol["prepared_depth_directory"]/"manifest.json"
        prepared = json.loads(depth_manifest.read_text())
        preparation_protocol = ROOT/protocol["prepared_depth_protocol"] if "prepared_depth_protocol" in protocol else args.protocol
        if prepared["status"] != "complete" or prepared["contract"]["protocol_sha256"] != sha256(preparation_protocol):
            raise ValueError("Prepared depth is incomplete or belongs to another protocol")
        if prepared["contract"]["dataset_config_sha256"] != sha256(ROOT/protocol["dataset_config"]):
            raise ValueError("Prepared depth has different dataset calibration")
        if depth_source == "sensor" and prepared["contract"].get("depth_source") != "sensor":
            raise ValueError("Sensor control requires explicitly recorded sensor depth")
        if depth_source == "da3" and prepared["contract"].get("depth_source", "da3") != "da3":
            raise ValueError("Learned-depth run cannot consume a sensor depth cache")
        if bool(prepared["contract"].get("groundtruth_calibration", False)) != gt_calibrated:
            raise ValueError("GT-derived depth correction must be explicitly labeled in the replay protocol")
        if gt_calibrated:
            calibration_file = ROOT/prepared["contract"]["calibration_path"]
            if sha256(calibration_file) != prepared["contract"]["calibration_sha256"]:
                raise ValueError("Prefix calibration evidence changed")
            calibration = json.loads(calibration_file.read_text())
            if calibration["scale"] != protocol["depth_scale"] or calibration["heldout_sensor_pixels_used_for_fit"]:
                raise ValueError("Depth scale differs from the frozen prefix-only correction")
            if sha256(ROOT/protocol["calibration_config"]) != prepared["contract"]["config_sha256"]:
                raise ValueError("Prefix calibration configuration changed")
        if preparation_protocol != args.protocol and depth_source == "da3":
            original = yaml.safe_load(preparation_protocol.read_text())
            for key in ("depth_scale", "depth_conversion", "depth_upsampling", "invalid_depth",
                        "depth_range_clipping", "confidence_filtering", "groundtruth_calibration"):
                if protocol[key] != original[key]:
                    raise ValueError(f"Prepared DA3 conversion differs from replay protocol: {key}")
        for directory, expected in prepared["contract"].get("prediction_manifest_sha256", {}).items():
            if sha256(ROOT/directory/"manifest.json") != expected:
                raise ValueError("Prediction cache manifest changed after depth preparation")
        for frame in rgb_records:
            record = prepared["frames"][frame.timestamp_text]
            if record["rgb_path"] != frame.relative_path or record["rgb_sha256"] != sha256(frame.path):
                raise ValueError("Prepared depth belongs to different RGB inputs")
            if sha256(ROOT/record["depth_path"]) != record["depth_sha256"]:
                raise ValueError("Prepared depth changed after conversion")
    pins = json.loads((ROOT/"configs/dependencies.json").read_text())
    revisions, diffs = {}, {}
    for name in ("ORB_SLAM3", "Pangolin"):
        directory = ROOT/"third_party"/name
        revision = subprocess.check_output(["git", "-C", str(directory), "rev-parse", "HEAD"], text=True).strip()
        if revision != pins[name]["revision"]:
            raise ValueError(f"Unexpected {name} revision")
        revisions[name] = revision
        diffs[name] = subprocess.check_output(["git", "-C", str(directory), "diff"])
        for patch in (ROOT/"patches"/name).glob("*.patch"):
            subprocess.run(["git", "-C", str(directory), "apply", "--reverse", "--check", str(patch)], check=True)
    if revisions["ORB_SLAM3"] != protocol["source_revision"]:
        raise ValueError("Protocol/upstream source pin mismatch")
    image = json.loads(subprocess.check_output([
        "docker", "image", "inspect", protocol["docker_image"], "--format", "{{json .}}"], text=True))
    binary, vocabulary = ROOT/protocol["runner"], ROOT/protocol["vocabulary"]
    architecture = {"arm64": "aarch64", "amd64": "x86_64"}[image["Architecture"]]
    libraries = ["third_party/ORB_SLAM3/lib/libORB_SLAM3.so",
                 "third_party/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so",
                 "third_party/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so",
                 f"third_party/install/Linux-{architecture}/lib/libpangolin.so"]
    run.mkdir(parents=True)
    (run/"camera.yaml").write_bytes(settings.read_bytes())
    (run/"rgb_input.txt").write_text("# Exact RGB input timeline, with optional learned-depth path\n" + "".join(
        f"{frame.timestamp_text} {frame.relative_path}" +
        (" /workspace/"+prepared["frames"][frame.timestamp_text]["depth_path"] if rgbd else "") + "\n"
        for frame in rgb_records))
    for name, diff in diffs.items():
        (run/f"{name}-local.patch").write_bytes(diff)
    if gt_calibrated:
        (run/"calibration.json").write_bytes(calibration_file.read_bytes())
    runner_source = "cpp/da3_rgbd_tum_runner.cc" if rgbd else "cpp/mono_tum_runner.cc"
    if rgbd and binary.name == "control_rgbd_tum":
        runner_source = "cpp/control_rgbd_tum_runner.cc"
    source_files = ["scripts/run_monocular.py", runner_source, "cpp/CMakeLists.txt",
                    "src/monocular_depth_slam/slam/trajectory.py", "src/monocular_depth_slam/slam/inputs.py"]
    if depth_source == "da3":
        source_files += ["scripts/run_da3_rgbd.py", "scripts/prepare_da3_rgbd.py",
                         "scripts/cache_da3_missing_rgb.py", "src/monocular_depth_slam/depth/orb_adapter.py",
                         "src/monocular_depth_slam/depth/metrics.py"]
        if gt_calibrated:
            source_files += ["scripts/prepare_prefix_calibration.py", "src/monocular_depth_slam/depth/calibration.py",
                             "configs/evaluations/prefix_calibration.yaml"]
    elif depth_source == "sensor":
        source_files += ["scripts/prepare_sensor_rgbd.py", "src/monocular_depth_slam/datasets/tum.py",
                         "src/monocular_depth_slam/depth/orb_adapter.py"]
    source_snapshots = {}
    for name in source_files:
        destination = run/"source"/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((ROOT/name).read_bytes())
        source_snapshots[name] = sha256(destination)
    engine = json.loads(subprocess.check_output(["docker", "info", "--format", "{{json .}}"], text=True))
    def mounted(path):
        return "/workspace/" + str(path.relative_to(ROOT))
    command = ["docker", "run", "--pull=never", "--rm", "--network=none",
               "--mount", f"type=bind,source={ROOT},target=/workspace",
               "-e", "OMP_NUM_THREADS=1", "-e", "OPENBLAS_NUM_THREADS=1",
               protocol["docker_image"], mounted(binary), mounted(vocabulary),
               mounted(run/"camera.yaml"), mounted(sequence.root), mounted(run/"rgb_input.txt"),
               mounted(run), str(protocol["seed"])]
    manifest = {
        "status": "running", "started_utc": datetime.now(timezone.utc).isoformat(),
        "protocol": protocol, "protocol_sha256": sha256(args.protocol),
        "orchestrator_sha256": sha256(Path(__file__)),
        "runner_source_sha256": sha256(ROOT/runner_source),
        "binary_sha256": sha256(binary), "vocabulary_sha256": sha256(vocabulary),
        "shared_library_sha256": {name: sha256(ROOT/name) for name in libraries},
        "camera_sha256": sha256(run/"camera.yaml"), "revisions": revisions,
        "local_patch_sha256": {name: hashlib.sha256(diff).hexdigest() for name, diff in diffs.items()},
        "dataset": "tum_rgbd", "sequence": sequence.root.name,
        "dataset_config_sha256": sha256(ROOT/protocol["dataset_config"]),
        "input_rgb_frames": len(rgb_records), "dataset_rgb_frames": len(sequence.rgb_records),
        "rgb_index_sha256": sha256(sequence.root/"rgb.txt"),
        "rgb_inputs": [{"timestamp": record.timestamp_text, "path": record.relative_path,
                        "sha256": sha256(record.path)} for record in rgb_records],
        "host": platform.platform(), "docker_image": {key: image[key] for key in ("Id", "Os", "Architecture")},
        "docker_runtime": {key: engine[key] for key in ("NCPU", "MemTotal", "KernelVersion", "ServerVersion")},
        "source_snapshots": source_snapshots,
        "docker_command": command, "depth_or_groundtruth_inputs_to_slam": rgbd,
        "sensor_depth_or_groundtruth_inputs_to_slam": depth_source == "sensor" or gt_calibrated,
        "sensor_depth_inputs_to_slam": depth_source == "sensor",
        "groundtruth_pose_inputs_to_slam": False, "learned_depth_inputs_to_slam": depth_source == "da3",
        "groundtruth_depth_used_for_calibration": gt_calibrated,
        "depth_source": depth_source,
        "seed_note": "cv RNG and C RNG fixed; upstream two-view RANSAC seeds 0; worker scheduling remains nondeterministic",
        "export": "upstream largest-map EuRoC-named exporter (ns/xyz/xyzw), converted to exact RGB stamps; primary includes online OK/OK_KLT only",
    }
    if rgbd:
        manifest["depth_input"] = {
            "prepared_manifest": str(depth_manifest.relative_to(ROOT)),
            "prepared_manifest_sha256": sha256(depth_manifest),
            "depth_source": depth_source, "submitted_depth_frames": len(rgb_records),
            "inference_frames": len(rgb_records) if depth_source == "da3" else 0,
            "cached_inference_wall_seconds_sum": sum(prepared["frames"][row.timestamp_text].get("inference_wall_seconds", 0) for row in rgb_records),
            "units": prepared["contract"]["units"], "groundtruth_calibration": gt_calibrated}
        if gt_calibrated:
            manifest["depth_input"]["calibration_sha256"] = sha256(run/"calibration.json")
            manifest["depth_input"]["calibration_prefix_frames"] = calibration["calibration_frames"]
            manifest["depth_input"]["frozen_depth_scale"] = calibration["scale"]
            manifest["depth_input"]["offline_replay_after_calibration"] = True
    path = run/"manifest.json"
    path.write_text(json.dumps(manifest, indent=2)+"\n")
    start = time.perf_counter()
    with (run/"run.log").open("w") as log:
        try:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=900,
                                    env={**os.environ})
            manifest["returncode"] = result.returncode
            manifest["status"] = "export_pending" if result.returncode == 0 else "failed"
        except Exception as error:
            manifest.update(status="failed", error=f"{type(error).__name__}: {error}")
            raise
        finally:
            manifest["wall_seconds"] = time.perf_counter()-start
            manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
            path.write_text(json.dumps(manifest, indent=2)+"\n")
    if manifest["status"] != "export_pending":
        raise SystemExit(f"Run failed; artifacts preserved in {run}")
    with (run/"tracking.csv").open() as stream:
        tracking = list(csv.DictReader(stream))
    if [row["timestamp"] for row in tracking] != [record.timestamp_text for record in rgb_records]:
        raise ValueError("Tracking log does not cover the exact input timeline")
    if rgbd:
        for row in tracking:
            record = prepared["frames"][row["timestamp"]]
            for key in ("depth_min_m", "depth_max_m", "depth_center_m"):
                if not math.isclose(float(row[key]), record[key], rel_tol=1e-7, abs_tol=1e-8):
                    raise ValueError(f"OpenCV depth decoding changed nominal metre values: {key}")
        manifest["opencv_depth_decode_check"] = "float32 type, 640x480; min/max/centre match prepared depth for all RGB frames"
    allowed = {row["timestamp"] for row in tracking if int(row["state"]) in (2, 5)}
    manifest["exported_all_map_poses"] = convert_orb_export(
        run/"trajectory_orb_ns.txt", run/"trajectory_exported_all.tum", rgb_records)
    manifest["primary_trajectory_poses"] = convert_orb_export(
        run/"trajectory_orb_ns.txt", run/"trajectory.tum", rgb_records, allowed)
    manifest["largest_map_keyframes"] = convert_orb_export(
        run/"keyframes_orb_ns.txt", run/"keyframes_largest_map.tum", rgb_records)
    maps = re.findall(r"There are (\d+) maps in the atlas", (run/"run.log").read_text())
    manifest["atlas_map_count_at_export"] = int(maps[-1]) if maps else None
    manifest["trajectory_sha256"] = sha256(run/"trajectory.tum")
    manifest["status"] = "complete"
    path.write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps({key: manifest[key] for key in ("status", "input_rgb_frames", "primary_trajectory_poses",
                                                   "largest_map_keyframes", "atlas_map_count_at_export", "wall_seconds")}, indent=2))


if __name__ == "__main__":
    main()
