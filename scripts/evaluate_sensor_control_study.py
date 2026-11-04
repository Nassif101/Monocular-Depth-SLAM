#!/usr/bin/env python3
"""Score all preserved control runs on one common pose/pair set and plot every repeat."""
import argparse
import csv
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from monocular_depth_slam.datasets.tum import load_config, read_groundtruth
from monocular_depth_slam.slam.inputs import selected_frames, validate_submitted_timeline
from run_monocular import sha256


ROOT = Path(__file__).resolve().parents[1]
METRICS = {
    "se3_ate_rmse_m": ("se3", "ate_translation_m", "rmse"),
    "sim3_ate_rmse_m": ("sim3", "ate_translation_m", "rmse"),
    "se3_rpe_translation_rmse_m": ("se3", "rpe_translation_m", "rmse"),
    "sim3_rpe_translation_rmse_m": ("sim3", "rpe_translation_m", "rmse"),
    "rpe_rotation_rmse_deg": ("sim3", "rpe_rotation_deg", "rmse"),
    "gt_fitted_scale": ("sim3", "scale"),
}


def value_at(data, keys):
    for key in keys:
        data = data[key]
    return data


def descriptive(values):
    values = np.asarray(values, dtype=float)
    return {"count": len(values), "values": values.tolist(), "median": float(np.median(values)),
            "min": float(values.min()), "max": float(values.max()), "mean": float(values.mean()),
            "sample_sd": float(values.std(ddof=1)) if len(values) > 1 else None}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study-dir", type=Path, default=ROOT/"results/sensor_depth_control_fr1_xyz")
    args = parser.parse_args()
    output = args.study_dir.resolve()
    study = json.loads((output/"study.json").read_text())
    if study["status"] not in ("complete", "complete_with_failures"):
        raise ValueError("Finish the scheduled replays before computing the common comparison")
    config_path = ROOT/"configs/evaluations/sensor_depth_control.yaml"
    if sha256(config_path) != study["contract"]["config_sha256"]:
        raise ValueError("Study configuration changed after scheduling")
    config = yaml.safe_load(config_path.read_text())
    sequence = load_config(ROOT/config["dataset_config"], project_root=ROOT)
    frames = selected_frames(sequence, config)
    successful = [job for job in study["jobs"] if job["status"] == "complete"]
    if not successful:
        raise ValueError("No successful runs; inspect preserved failures")
    manifests, common, identical_inputs, base_camera, libraries = {}, None, None, None, None
    rgbd_binary = None
    for job in successful:
        directory = ROOT/job["run_dir"]
        manifest = json.loads((directory/"manifest.json").read_text())
        if sha256(directory/"manifest.json") != job["manifest_sha256"] or manifest["status"] != "complete":
            raise ValueError("Run manifest changed or is incomplete")
        if sha256(directory/"trajectory.tum") != manifest["trajectory_sha256"]:
            raise ValueError("Trajectory changed after replay")
        validate_submitted_timeline(frames, manifest)
        if identical_inputs is not None and manifest["rgb_inputs"] != identical_inputs:
            raise ValueError("Compared runs used different RGB inputs")
        identical_inputs = manifest["rgb_inputs"]
        if libraries is not None and manifest["shared_library_sha256"] != libraries:
            raise ValueError("Compiled libraries differ across control runs")
        libraries = manifest["shared_library_sha256"]
        source = {"monocular": "none", "da3_rgbd": "da3", "sensor_rgbd": "sensor"}[job["method"]]
        if manifest["depth_source"] != source or manifest["groundtruth_pose_inputs_to_slam"]:
            raise ValueError("Recorded depth/pose inputs do not match the scheduled method")
        if source != "none":
            if rgbd_binary is not None and manifest["binary_sha256"] != rgbd_binary:
                raise ValueError("DA3 and sensor runs must use the identical RGB-D executable")
            rgbd_binary = manifest["binary_sha256"]
        camera = yaml.safe_load((directory/"camera.yaml").read_text().replace("%YAML:1.0", ""))
        comparable = {key:value for key,value in camera.items() if not key.startswith(("Stereo.", "RGBD."))}
        if base_camera is not None and comparable != base_camera:
            raise ValueError("RGB/ORB camera parameters differ between control runs")
        base_camera = comparable
        stamps = {pose.timestamp_text for pose in read_groundtruth(directory/"trajectory.tum")}
        common = stamps if common is None else common & stamps
        manifests[job["run_dir"]] = manifest
    common &= {frame.rgb.timestamp_text for frame in frames if frame.pose is not None}
    ordered_stamps = [frame.rgb.timestamp_text for frame in frames if frame.rgb.timestamp_text in common]
    if len(ordered_stamps) < 3:
        raise ValueError("Insufficient study-wide common reference matches")
    common_file = output/"common_stamps.json"
    common_file.write_text(json.dumps({"rgb_timestamps": ordered_stamps,
                                      "study_sha256": sha256(output/"study.json"),
                                      "selection": "Intersection across all successful runs and fixed full-loader GT matches",
                                      "rpe_gap_indices": "792 submitted frame indices, not 798 source frame indices"}, indent=2)+"\n")
    rows, summaries, association, rpe_pairs = [], {}, None, None
    for job in successful:
        run = ROOT/job["run_dir"]
        common_output = run/"common_evaluation"
        for options, log_name in (([], "evaluation.log"),
                                  (["--common-stamps", str(common_file), "--output-dir", str(common_output)], "common_evaluation.log")):
            with (run/log_name).open("w") as stream:
                subprocess.run([sys.executable, str(ROOT/"scripts/evaluate_monocular.py"), "--run-dir", str(run), *options],
                               stdout=stream, stderr=subprocess.STDOUT, check=True)
        summary = json.loads((common_output/"summary.json").read_text())
        with (common_output/"per_pose.csv").open() as stream:
            poses = list(csv.DictReader(stream))
        current = [(row["input_index"],row["dataset_rgb_index"],row["rgb_timestamp"],row["reference_timestamp"]) for row in poses]
        if association is not None and current != association:
            raise ValueError("Reference/RGB associations differ across the common comparison")
        association = current
        with (common_output/"rpe_sim3.csv").open() as stream:
            pairs = [(row["start_timestamp"],row["end_timestamp"]) for row in csv.DictReader(stream)]
        if rpe_pairs is not None and pairs != rpe_pairs:
            raise ValueError("RPE pairs differ across the common comparison")
        rpe_pairs = pairs
        manifest = manifests[job["run_dir"]]
        row = {key: job[key] for key in ("method", "repeat", "block", "position", "order", "run_dir")}
        row.update({name: value_at(summary, path) for name,path in METRICS.items()})
        row.update(input_frames=manifest["input_rgb_frames"], exported_poses=manifest["primary_trajectory_poses"],
                   initialization_frames=summary["tracking"]["initialization_frames_before_first_ok"],
                   tracked_frames=summary["tracking"]["tracked_frames"],
                   failure_events=len(summary["tracking"]["post_initialization_failure_events"]),
                   maps=manifest["atlas_map_count_at_export"], keyframes=manifest["largest_map_keyframes"],
                   common_poses=len(ordered_stamps), common_rpe_pairs=len(pairs),
                   replay_wall_seconds=manifest["wall_seconds"],
                   mean_tracking_call_ms=summary["tracking"]["tracking_call_seconds_all_frames"]["mean"]*1000,
                   manifest_sha256=sha256(run/"manifest.json"),
                   full_summary_sha256=sha256(run/"evaluation/summary.json"),
                   common_summary_sha256=sha256(common_output/"summary.json"))
        rows.append(row)
        summaries[job["run_dir"]] = summary
    for method in config["methods"]:
        if not any(row["method"] == method for row in rows):
            raise ValueError(f"No successful {method} run; failures are preserved but no three-method figure can be drawn")
    methods = {name: {"attempted": sum(job["method"] == name for job in study["jobs"]),
                      "successful": sum(row["method"] == name for row in rows),
                      "metrics": {metric: descriptive([row[metric] for row in rows if row["method"] == name]) for metric in METRICS}}
               for name in config["methods"]}
    source_snapshots = {}
    for name in ("scripts/evaluate_sensor_control_study.py", "scripts/run_sensor_control_study.py",
                 "configs/evaluations/sensor_depth_control.yaml", *config["methods"].values()):
        destination = output/"source"/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((ROOT/name).read_bytes())
        source_snapshots[name] = sha256(destination)
    result = {"study_sha256": sha256(output/"study.json"), "analysis_sha256": sha256(Path(__file__)),
              "evaluator_sha256": sha256(ROOT/"scripts/evaluate_monocular.py"),
              "common_stamps_sha256": sha256(common_file),
              "input_frames_per_run": len(frames), "common_matched_poses": len(ordered_stamps),
              "common_rpe_pairs": len(rpe_pairs), "independent_sequences": 1,
              "replication": "three fixed-seed processes per method; asynchronous scheduling can vary",
              "statistics": "individual values and median/min/max; no significance tests or confidence intervals",
              "rpe_gap_convention": "submitted timeline, intentional source-image omissions are shared inputs, not missing estimates",
              "alignment": "Each run independently refitted on identical study-wide common matched poses",
              "failed_jobs": [job for job in study["jobs"] if job["status"] != "complete"],
              "source_snapshots": source_snapshots, "methods": methods, "runs": rows}
    (output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    with (output/"per_run.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    labels = {"monocular": "Monocular", "da3_rgbd": "DA3 RGB-D", "sensor_rgbd": "Sensor RGB-D"}
    colors = {"monocular": "#0072B2", "da3_rgbd": "#D55E00", "sensor_rgbd": "#009E73"}
    plotted = [("se3_ate_rmse_m", 100, "SE(3) ATE RMSE (cm)", "No scale correction; monocular units arbitrary"),
               ("sim3_ate_rmse_m", 1000, "Sim(3) ATE RMSE (mm)", "One GT-fitted scale per run"),
               ("sim3_rpe_translation_rmse_m", 100, "Sim(3) 1 s translation RPE RMSE (cm)", "Identical time-interval pairs"),
               ("gt_fitted_scale", 1, "GT-fitted global scale correction", "1 means no scale correction")]
    with plt.rc_context({"font.size": 9, "font.family": "DejaVu Sans", "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42, "savefig.facecolor": "white"}):
        fig, axes = plt.subplots(2,2,figsize=(10,7),layout="constrained")
        for axis, (metric, factor, ylabel, title) in zip(axes.flat, plotted):
            for x, name in enumerate(config["methods"]):
                values = np.array([row[metric]*factor for row in sorted(rows, key=lambda row:row["repeat"]) if row["method"] == name])
                axis.vlines(x, values.min(), values.max(), color=colors[name], linewidth=1)
                axis.hlines(np.median(values), x-.16, x+.16, color=colors[name], linewidth=2)
                for y, offset, marker in zip(values, (-.07,0,.07), ("o","s","^")):
                    axis.scatter(x+offset, y, marker=marker, color=colors[name], s=30, zorder=3)
            axis.set(xticks=range(3), xticklabels=list(labels.values()), ylabel=ylabel, title=title)
            axis.set_xlim(-.45,2.45)
            if metric == "gt_fitted_scale":
                axis.axhline(1, color="black", linewidth=.7)
            else:
                axis.set_ylim(bottom=0)
        fig.suptitle(f"Sensor-depth control: {len(ordered_stamps)} common poses, {len(rpe_pairs)} RPE pairs; one sequence", fontsize=12)
        fig.supxlabel("Markers: repeats 1/2/3 (circle/square/triangle). Thick line: median. Thin line: observed range; no confidence interval.", fontsize=8)
        for extension in ("png", "pdf"):
            fig.savefig(output/f"repeat_comparison.{extension}", dpi=180)
        plt.close(fig)
    (output/"figure_provenance.json").write_text(json.dumps({
        "script_sha256": sha256(Path(__file__)), "summary_sha256": sha256(output/"summary.json"),
        "csv_sha256": sha256(output/"per_run.csv"),
        "packages": {name:importlib.metadata.version(name) for name in ("matplotlib","numpy","pillow")},
        "normalization": "Shared pose/pair sets; separately fitted SE3 and Sim3 per process",
        "missing_data": "All attempted/successful counts retained; no best-run selection",
        "uncertainty": "All repeats shown; range is descriptive, no independent-scene CI",
        "formats": ["png","pdf"], "png_dpi": 180, "publisher_requirements": "none specified; internal research figure"}, indent=2)+"\n")
    print(json.dumps({"common_poses": len(ordered_stamps), "rpe_pairs": len(rpe_pairs), "methods": methods}, indent=2))
