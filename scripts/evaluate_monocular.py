#!/usr/bin/env python3
"""Evaluate optimized TUM poses under SE(3)/Sim(3), optionally on common timestamps."""
import argparse
import csv
from dataclasses import replace
from datetime import datetime, timezone
from decimal import Decimal
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import zipfile

from monocular_depth_slam.datasets.tum import load_config, read_groundtruth
from monocular_depth_slam.slam.evaluation import aligned_errors, tracking_summary, window_scale_diagnostic
from monocular_depth_slam.slam.inputs import selected_frames, validate_submitted_timeline
from monocular_depth_slam.slam.trajectory import time_interval_pairs


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_poses(path, stamps, trajectory):
    quaternions = trajectory.orientations_quat_wxyz
    with path.open("w") as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")
        for stamp, position, quaternion in zip(stamps, trajectory.positions_xyz, quaternions):
            values = [*position, *quaternion[[1, 2, 3, 0]]]
            stream.write(stamp + " " + " ".join(f"{value:.12g}" for value in values) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=ROOT/"results/orbslam3_mono_fr1_xyz/run_001")
    parser.add_argument("--common-with", type=Path, help="Restrict scores/alignment to poses also exported by this run")
    parser.add_argument("--common-stamps", type=Path, help="JSON timestamp set shared by a repeated-run study")
    parser.add_argument("--output-dir", type=Path, help="Override output directory (required with --common-with)")
    parser.add_argument("--translation-scale", type=float, default=1.0,
                        help="Explicit post-hoc translation-only rescaling diagnostic; requires separate output")
    args = parser.parse_args()
    if (args.common_with or args.common_stamps) and not args.output_dir:
        parser.error("Common-timestamp scores need a separate --output-dir to preserve full-run results")
    if args.common_with and args.common_stamps:
        parser.error("Choose either a comparison run or a common timestamp file")
    if not math.isfinite(args.translation_scale) or args.translation_scale <= 0:
        parser.error("translation-scale must be finite and positive")
    if args.translation_scale != 1 and not args.output_dir:
        parser.error("Pose-only rescaling requires a separate --output-dir")
    run = args.run_dir.resolve()
    manifest = json.loads((run/"manifest.json").read_text())
    if manifest["status"] != "complete":
        raise ValueError("A complete successful replay/export is required")
    protocol = manifest["protocol"]
    if sha256(run/"trajectory.tum") != manifest["trajectory_sha256"]:
        raise ValueError("Trajectory differs from run manifest")
    if sha256(ROOT/protocol["dataset_config"]) != manifest["dataset_config_sha256"]:
        raise ValueError("Dataset configuration changed after the run")
    sequence = load_config(ROOT/protocol["dataset_config"], project_root=ROOT)
    frames = selected_frames(sequence, protocol)
    validate_submitted_timeline(frames, manifest)
    original_indices = {frame.rgb.timestamp_text: i for i, frame in enumerate(sequence.frames)}
    if sequence.max_pose_difference_s != protocol["reference_association_tolerance_s"]:
        raise ValueError("Run protocol and dataset pose-association tolerances differ")
    estimates = read_groundtruth(run/"trajectory.tum")
    if args.translation_scale != 1:
        estimates = tuple(replace(pose, translation=tuple(value*args.translation_scale for value in pose.translation))
                          for pose in estimates)
    by_stamp = {pose.timestamp_text: pose for pose in estimates}
    if not set(by_stamp).issubset({frame.rgb.timestamp_text for frame in frames}):
        raise ValueError("Export contains poses outside the submitted RGB timeline")
    common_stamps, comparison = None, None
    if args.common_with:
        other = args.common_with.resolve()
        other_manifest = json.loads((other/"manifest.json").read_text())
        if other_manifest["status"] != "complete" or sha256(other/"trajectory.tum") != other_manifest["trajectory_sha256"]:
            raise ValueError("Comparison trajectory must be complete and unmodified")
        if manifest["rgb_inputs"] != other_manifest["rgb_inputs"] or manifest["dataset_config_sha256"] != other_manifest["dataset_config_sha256"]:
            raise ValueError("Common comparison requires identical RGB inputs and dataset calibration")
        common_stamps = set(by_stamp) & {pose.timestamp_text for pose in read_groundtruth(other/"trajectory.tum")}
        comparison = {"other_run": str(other.relative_to(ROOT)),
                      "other_manifest_sha256": sha256(other/"manifest.json"),
                      "selection": "Intersection of primary exported RGB timestamps with fixed dataset GT matches; alignment refitted per method"}
    elif args.common_stamps:
        data = json.loads(args.common_stamps.read_text())
        common_stamps = set(data["rgb_timestamps"])
        if not common_stamps.issubset(by_stamp):
            raise ValueError("Study timestamp set includes a pose absent from this run")
        comparison = {"common_stamps": str(args.common_stamps.resolve().relative_to(ROOT)),
                      "common_stamps_sha256": sha256(args.common_stamps),
                      "selection": "Study-wide intersection of evaluable submitted RGB timestamps; each alignment refitted"}
    reference, estimated, indices, stamps, differences = [], [], [], [], []
    for index, frame in enumerate(frames):
        if (frame.rgb.timestamp_text in by_stamp and frame.pose is not None
                and (common_stamps is None or frame.rgb.timestamp_text in common_stamps)):
            reference.append(frame.pose)
            estimated.append(by_stamp[frame.rgb.timestamp_text])
            indices.append(index)
            stamps.append(frame.rgb.timestamp_text)
            differences.append(float(abs(Decimal(frame.rgb.timestamp_text)-Decimal(frame.pose.timestamp_text))))
    if len(reference) < 3:
        raise ValueError("Fewer than three matched poses; no alignment score is defensible")
    pairs = time_interval_pairs([pose.timestamp for pose in estimated], indices,
                                 protocol["rpe_interval_s"], protocol["rpe_interval_tolerance_s"])
    if not pairs:
        raise ValueError("No RPE pairs at the declared interval without bridging gaps")
    evaluation = args.output_dir.resolve() if args.output_dir else run/"evaluation"
    evaluation.mkdir(parents=True, exist_ok=True)
    source_snapshots = {}
    for name in ("scripts/evaluate_monocular.py", "src/monocular_depth_slam/slam/evaluation.py",
                 "src/monocular_depth_slam/slam/trajectory.py", "src/monocular_depth_slam/datasets/tum.py",
                 "src/monocular_depth_slam/slam/inputs.py"):
        destination = evaluation/"source"/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((ROOT/name).read_bytes())
        source_snapshots[name] = sha256(destination)
    summary = {"created_utc": datetime.now(timezone.utc).isoformat(),
               "evo_version": importlib.metadata.version("evo"),
               "package_versions": {name: importlib.metadata.version(name) for name in ("evo", "numpy", "scipy")},
               "python_lock_sha256": sha256(ROOT/"uv.lock"),
               "run_manifest_sha256": sha256(run/"manifest.json"),
               "trajectory_sha256": sha256(run/"trajectory.tum"),
               "groundtruth_sha256": sha256(sequence.root/"groundtruth.txt"),
               "evaluator_sha256": sha256(Path(__file__)),
               "evaluation_module_sha256": sha256(ROOT/"src/monocular_depth_slam/slam/evaluation.py"),
               "pairing_module_sha256": sha256(ROOT/"src/monocular_depth_slam/slam/trajectory.py"),
               "source_snapshots": source_snapshots,
               "tracking": None,
               "alignment_scope": "All selected matched poses; see execution calibration and evaluation pretransform metadata",
               "execution_groundtruth_depth_calibration": bool(protocol.get("groundtruth_calibration", False)),
               "evaluation_pretransform": {"translation_scale": args.translation_scale,
                                           "meaning": "pose-only scalar before alignment; not an additional SLAM replay"},
               "native_scale": ("Source execution units with explicit evaluation translation rescaling; see evaluation_pretransform"
                                if args.translation_scale != 1 else
                                "Sensor z-depth metres; explicit sensor-control run"
                                if protocol.get("depth_source") == "sensor" else
                                "DA3 depth scaled using a GT prefix before offline SLAM replay"
                                if protocol.get("groundtruth_calibration") else
                                "Nominal metres from fixed DA3 conversion; no GT depth calibration"
                                if protocol.get("sensor") == "rgbd" else
                                "Arbitrary monocular units; unscaled SE(3) comparison treats their numeric values as metres"),
               "method": protocol.get("method", "ORB-SLAM3 monocular"),
               "common_comparison": comparison,
               "reference_association": "Dataset loader one-to-one closest-first, 20ms, no interpolation",
               "rpe": {"target_interval_s": protocol["rpe_interval_s"],
                       "tolerance_s": protocol["rpe_interval_tolerance_s"], "pairs": len(pairs),
                       "pairing": "closest future pose per anchor; no missing poses/GT in submitted RGB timeline indices",
                       "actual_interval_s_min": min(estimated[j].timestamp-estimated[i].timestamp for i,j in pairs),
                       "actual_interval_s_max": max(estimated[j].timestamp-estimated[i].timestamp for i,j in pairs)},
               "independent_sequence_replicates": 1,
               "repeated_run_variability_evaluated": False}
    with (run/"tracking.csv").open() as stream:
        tracking = list(csv.DictReader(stream))
    summary["tracking"] = tracking_summary(tracking)
    summary["coverage"] = {
        "submitted_rgb_frames": len(frames), "dataset_rgb_frames": len(sequence.frames), "estimated_primary_poses": len(estimates),
        "trajectory_fraction_of_inputs": len(estimates)/len(frames),
        "reference_associable_rgb_frames": sum(frame.pose is not None for frame in frames),
        "matched_for_evaluation": len(reference),
        "exported_without_reference_match": [frame.rgb.timestamp_text for frame in frames
                                              if frame.rgb.timestamp_text in by_stamp and frame.pose is None],
        "excluded_from_common_comparison": [frame.rgb.timestamp_text for frame in frames
                                              if frame.rgb.timestamp_text in by_stamp and frame.pose is not None
                                              and common_stamps is not None and frame.rgb.timestamp_text not in common_stamps],
        "reference_max_time_difference_s": max(differences),
        "primary_estimate_first_timestamp": estimates[0].timestamp_text,
        "primary_estimate_last_timestamp": estimates[-1].timestamp_text,
    }
    error_rows = [{"input_index": index, "dataset_rgb_index": original_indices[stamp], "rgb_timestamp": stamp,
                   "reference_timestamp": ref.timestamp_text,
                   "reference_time_difference_s": diff}
                  for index, stamp, ref, diff in zip(indices, stamps, reference, differences)]
    for alignment, scale in (("se3", False), ("sim3", True)):
        result, gt, aligned, ape, trans, angles = aligned_errors(reference, estimated, pairs, scale)
        summary[alignment] = result
        write_poses(evaluation/f"trajectory_{alignment}_aligned.tum", stamps, aligned)
        if alignment == "sim3":
            summary["window_scale_diagnostic"] = window_scale_diagnostic(
                gt, aligned, indices, protocol["scale_diagnostic_window_s"])
            write_poses(evaluation/"reference_matched.tum", [pose.timestamp_text for pose in reference], gt)
        for row, error in zip(error_rows, ape):
            row[f"ate_{alignment}_m"] = float(error)
        with (evaluation/f"rpe_{alignment}.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["start_timestamp", "end_timestamp", "translation_error_m", "rotation_error_deg"])
            for (i,j), translation, angle in zip(pairs, trans, angles):
                writer.writerow([stamps[i], stamps[j], translation, angle])
    with (evaluation/"per_pose.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(error_rows[0]))
        writer.writeheader()
        writer.writerows(error_rows)
    summary["evo_cli_crosscheck"] = {}
    for alignment in ("se3", "sim3"):
        result_file = evaluation/f"evo_ape_{alignment}.zip"
        if result_file.exists():
            with zipfile.ZipFile(result_file) as archive:
                cli_stats = json.loads(archive.read("stats.json"))
            difference = abs(cli_stats["rmse"]-summary[alignment]["ate_translation_m"]["rmse"])
            if difference > 1e-12:
                raise ValueError(f"evo CLI ATE disagrees for {alignment}")
            summary["evo_cli_crosscheck"][alignment] = {
                "result_sha256": sha256(result_file), "rmse_m": cli_stats["rmse"],
                "absolute_rmse_difference_m": difference}
    (evaluation/"summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"coverage": summary["coverage"],
                      "SE3_ATE_RMSE_m": summary["se3"]["ate_translation_m"]["rmse"],
                      "Sim3_ATE_RMSE_m": summary["sim3"]["ate_translation_m"]["rmse"],
                      "Sim3_scale": summary["sim3"]["scale"],
                      "Sim3_RPE_1s_RMSE_m": summary["sim3"]["rpe_translation_m"]["rmse"],
                      "RPE_1s_rotation_RMSE_deg": summary["sim3"]["rpe_rotation_deg"]["rmse"]}, indent=2))
