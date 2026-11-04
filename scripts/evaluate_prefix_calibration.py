#!/usr/bin/env python3
"""Compare later-frame scores for GT-prefix depth replay and pose-only rescaling."""
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

from evaluate_sensor_control_study import METRICS, descriptive, value_at
from monocular_depth_slam.datasets.tum import load_config, read_groundtruth
from monocular_depth_slam.depth.calibration import temporal_partition
from monocular_depth_slam.slam.inputs import selected_frames, validate_submitted_timeline
from run_monocular import sha256


ROOT = Path(__file__).resolve().parents[1]
LABELS = {"monocular": "Monocular", "da3_rgbd": "DA3\nraw",
          "da3_pose_rescale": "DA3\npose rescale", "da3_prefix_rgbd": "DA3 GT-prefix\ndepth replay",
          "sensor_rgbd": "Sensor\nRGB-D"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"configs/evaluations/prefix_calibration.yaml")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    output = ROOT/config["output_directory"]
    study = json.loads((output/"study.json").read_text())
    if study["status"] not in ("complete", "complete_with_failures"):
        raise ValueError("Complete the calibrated replays first")
    if study["contract"]["config_sha256"] != sha256(args.config):
        raise ValueError("Calibration study configuration changed")
    baseline_path = ROOT/config["baseline_study"]
    baseline = json.loads(baseline_path.read_text())
    if sha256(baseline_path) != study["contract"]["baseline_study_sha256"]:
        raise ValueError("Historical control schedule/attempts changed")
    sequence = load_config(ROOT/config["dataset_config"], project_root=ROOT)
    frames = selected_frames(sequence, config)
    prefix, later = temporal_partition(frames, config["calibration_frames"])
    scale = config["frozen_depth_scale"]
    jobs = [dict(job, execution_kind="historical_replay") for job in baseline["jobs"]]
    jobs += [dict(job, execution_kind="new_calibrated_replay") for job in study["jobs"]]
    failures = [job for job in jobs if job["status"] != "complete"]
    executed = [job for job in jobs if job["status"] == "complete"]
    if not any(job["method"] == "da3_prefix_rgbd" for job in executed):
        raise ValueError("No successful calibrated replays; inspect preserved failures")
    common = {frame.rgb.timestamp_text for frame in later if frame.pose is not None}
    inputs, rgbd_binary, libraries, camera_base = None, None, None, None
    manifests = {}
    for job in executed:
        path = ROOT/job["run_dir"]
        manifest = json.loads((path/"manifest.json").read_text())
        if sha256(path/"manifest.json") != job["manifest_sha256"] or sha256(path/"trajectory.tum") != manifest["trajectory_sha256"]:
            raise ValueError("Historical or new input trajectory/manifest changed")
        validate_submitted_timeline(frames, manifest)
        if inputs is not None and manifest["rgb_inputs"] != inputs:
            raise ValueError("Executed methods did not use identical RGB frames")
        inputs = manifest["rgb_inputs"]
        if libraries is not None and manifest["shared_library_sha256"] != libraries:
            raise ValueError("Compiled libraries differ across methods")
        libraries = manifest["shared_library_sha256"]
        with (path/"camera.yaml").open() as stream:
            camera = yaml.safe_load(stream.read().replace("%YAML:1.0", ""))
        comparable = {key:value for key,value in camera.items() if not key.startswith(("Stereo.", "RGBD."))}
        if camera_base is not None and comparable != camera_base:
            raise ValueError("RGB/ORB settings differ between runs")
        camera_base = comparable
        if manifest["protocol"].get("sensor") == "rgbd":
            if rgbd_binary is not None and manifest["binary_sha256"] != rgbd_binary:
                raise ValueError("Depth methods did not use the same RGB-D executable")
            rgbd_binary = manifest["binary_sha256"]
        calibrated = job["method"] == "da3_prefix_rgbd"
        if bool(manifest["protocol"].get("groundtruth_calibration", False)) != calibrated:
            raise ValueError("Calibration status is mislabeled")
        if calibrated:
            if not manifest["groundtruth_depth_used_for_calibration"] or manifest["groundtruth_pose_inputs_to_slam"]:
                raise ValueError("GT-prefix calibration inputs are mislabeled")
            if manifest["depth_input"]["frozen_depth_scale"] != scale:
                raise ValueError("A replay used a different scalar")
        common &= {pose.timestamp_text for pose in read_groundtruth(path/"trajectory.tum")}
        manifests[job["run_dir"]] = manifest
    common_stamps = [frame.rgb.timestamp_text for frame in later if frame.rgb.timestamp_text in common]
    if len(common_stamps) < 3:
        raise ValueError("Insufficient later-frame common reference matches")
    common_file = output/"heldout_common_stamps.json"
    common_file.write_text(json.dumps({"rgb_timestamps": common_stamps,
                                      "selection": "post-prefix intersection across every successful historical/new replay",
                                      "prefix_frames": len(prefix), "evaluation_frames_before_GT_matching": len(later),
                                      "prefix_last_timestamp": prefix[-1].rgb.timestamp_text,
                                      "evaluation_first_timestamp": later[0].rgb.timestamp_text,
                                      "study_sha256": sha256(output/"study.json")}, indent=2)+"\n")
    derived = [dict(job, method="da3_pose_rescale", execution_kind="derived_pose_rescale")
               for job in executed if job["method"] == "da3_rgbd"]
    rows, association, rpe_pairs = [], None, None
    for job in executed+derived:
        source = ROOT/job["run_dir"]
        directory = output/"evaluations"/job["method"]/f"repeat_{job['repeat']:03d}"
        directory.mkdir(parents=True, exist_ok=True)
        if job["method"] == "da3_prefix_rgbd":
            with (source/"evaluation.log").open("w") as log:
                subprocess.run([sys.executable, str(ROOT/"scripts/evaluate_monocular.py"),
                                "--run-dir", str(source)], stdout=log, stderr=subprocess.STDOUT, check=True)
        transformation = scale if job["execution_kind"] == "derived_pose_rescale" else 1.0
        with (directory/"evaluation.log").open("w") as log:
            subprocess.run([sys.executable, str(ROOT/"scripts/evaluate_monocular.py"),
                            "--run-dir", str(source), "--common-stamps", str(common_file),
                            "--output-dir", str(directory), "--translation-scale", str(transformation)],
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        summary = json.loads((directory/"summary.json").read_text())
        with (directory/"per_pose.csv").open() as stream:
            poses = list(csv.DictReader(stream))
        selected = [(row["input_index"],row["dataset_rgb_index"],row["rgb_timestamp"],row["reference_timestamp"]) for row in poses]
        if association is not None and selected != association:
            raise ValueError("Evaluated timestamps/reference matches differ")
        association = selected
        with (directory/"rpe_sim3.csv").open() as stream:
            pairs = [(row["start_timestamp"],row["end_timestamp"]) for row in csv.DictReader(stream)]
        if rpe_pairs is not None and pairs != rpe_pairs:
            raise ValueError("Evaluated RPE pairs differ")
        rpe_pairs = pairs
        manifest = manifests[job["run_dir"]]
        row = {key:job[key] for key in ("method","repeat","run_dir","execution_kind")}
        row.update({metric:value_at(summary, path) for metric,path in METRICS.items()})
        row.update(evaluation_directory=str(directory.relative_to(ROOT)),
                   evaluation_summary_sha256=sha256(directory/"summary.json"),
                   source_manifest_sha256=sha256(source/"manifest.json"),
                   source_trajectory_sha256=manifest["trajectory_sha256"],
                   pose_pretransform_scale=transformation,
                   exported_poses=manifest["primary_trajectory_poses"],
                   tracked_frames=summary["tracking"]["tracked_frames"],
                   failure_events=len(summary["tracking"]["post_initialization_failure_events"]),
                   keyframes=manifest["largest_map_keyframes"], maps=manifest["atlas_map_count_at_export"],
                   common_poses=len(common_stamps), common_rpe_pairs=len(pairs))
        rows.append(row)
    for derived_row in [row for row in rows if row["method"] == "da3_pose_rescale"]:
        original = next(row for row in rows if row["method"] == "da3_rgbd" and row["repeat"] == derived_row["repeat"])
        for key in ("sim3_ate_rmse_m", "sim3_rpe_translation_rmse_m", "rpe_rotation_rmse_deg"):
            if abs(derived_row[key]-original[key]) > 1e-10:
                raise ValueError("Uniform pose-only scaling changed a scale-invariant metric")
    methods = {name:{"kind": "derived_pose_rescale" if name == "da3_pose_rescale" else "executed_replay",
                     "successful": sum(row["method"] == name for row in rows),
                     "metrics": {metric:descriptive([row[metric] for row in rows if row["method"] == name]) for metric in METRICS}}
               for name in LABELS}
    snapshots = {}
    for name in ("scripts/evaluate_prefix_calibration.py", "scripts/run_prefix_calibration_study.py",
                 "scripts/prepare_prefix_calibration.py", "scripts/evaluate_sensor_control_study.py",
                 "configs/evaluations/prefix_calibration.yaml", config["replay_protocol"]):
        destination = output/"source"/name
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes((ROOT/name).read_bytes())
        snapshots[name] = sha256(destination)
    result = {"config_sha256": sha256(args.config), "study_sha256": sha256(output/"study.json"),
              "historical_study_sha256": sha256(baseline_path), "analysis_sha256": sha256(Path(__file__)),
              "evaluator_sha256": sha256(ROOT/"scripts/evaluate_monocular.py"),
              "heldout_common_stamps_sha256": sha256(common_file),
              "frozen_depth_scale": scale, "calibration_frames": len(prefix), "later_input_frames": len(later),
              "common_matched_poses": len(common_stamps), "common_rpe_pairs": len(rpe_pairs),
              "new_executed_runs": len([job for job in executed if job["execution_kind"] == "new_calibrated_replay"]),
              "historical_executed_runs": len([job for job in executed if job["execution_kind"] == "historical_replay"]),
              "derived_pose_only_scores": len(derived), "independent_sequences": 1,
              "statistics": "Individual scores, median/range and mean/sample SD; no significance test or scene-level CI",
              "calibration_scope": "GT sensor-depth first 158 frames; offline replay; no later sensor pixels fit the correction",
              "alignment_scope": "Each SE3/Sim3 alignment refitted using the same later matched poses only",
              "pose_rescale_sim3_invariance_checked": True, "failed_jobs": failures,
              "source_snapshots": snapshots, "methods": methods, "runs": rows}
    (output/"summary.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    with (output/"per_run.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    colors = {"monocular":"#0072B2", "da3_rgbd":"#D55E00", "da3_pose_rescale":"#666666",
              "da3_prefix_rgbd":"#CC79A7", "sensor_rgbd":"#009E73"}
    panels = [("se3_ate_rmse_m",100,"SE(3) ATE RMSE (cm)","No scale fitting; monocular units arbitrary"),
              ("sim3_ate_rmse_m",1000,"Sim(3) ATE RMSE (mm)","One GT-fitted scale per run"),
              ("sim3_rpe_translation_rmse_m",100,"Sim(3) 1 s translation RPE RMSE (cm)","Identical later-frame RPE pairs"),
              ("gt_fitted_scale",1,"GT-fitted global scale correction","1 means no additional scale correction")]
    with plt.rc_context({"font.size":9,"font.family":"DejaVu Sans","axes.spines.top":False,
                         "axes.spines.right":False,"pdf.fonttype":42,"savefig.facecolor":"white"}):
        fig, axes = plt.subplots(2,2,figsize=(12,8),layout="constrained")
        for axis,(metric,factor,ylabel,title) in zip(axes.flat,panels):
            for x,name in enumerate(LABELS):
                values=np.array([row[metric]*factor for row in sorted(rows,key=lambda row:row['repeat']) if row['method']==name])
                axis.vlines(x,values.min(),values.max(),color=colors[name],linewidth=1)
                axis.hlines(np.median(values),x-.15,x+.15,color=colors[name],linewidth=2)
                for y,offset,marker in zip(values,(-.07,0,.07),("o","s","^")):
                    axis.scatter(x+offset,y,marker=marker,color=colors[name],s=28,zorder=3)
            axis.set(xticks=range(len(LABELS)),xticklabels=list(LABELS.values()),ylabel=ylabel,title=title)
            axis.set_xlim(-.45,len(LABELS)-.55)
            if metric=="gt_fitted_scale":
                axis.axhline(1,color="black",linewidth=.7)
            else:
                axis.set_ylim(bottom=0)
        fig.suptitle(f"GT-prefix calibration diagnostic: {len(common_stamps)} later poses, {len(rpe_pairs)} RPE pairs; one sequence",fontsize=12)
        fig.supxlabel("Circles/squares/triangles: repeats 1/2/3. Thick line: median; thin line: observed range, not CI. Pose rescale is derived, not replayed.",fontsize=8)
        for extension in ("png","pdf"):
            fig.savefig(output/f"heldout_comparison.{extension}",dpi=180)
        plt.close(fig)
    (output/"figure_provenance.json").write_text(json.dumps({
        "script_sha256":sha256(Path(__file__)),"summary_sha256":sha256(output/"summary.json"),
        "csv_sha256":sha256(output/"per_run.csv"),
        "packages":{name:importlib.metadata.version(name) for name in ("matplotlib","numpy","pillow")},
        "normalization":"Later poses only; independent SE3/Sim3 per evaluation; GT-prefix scale applied before replay or to poses as labeled",
        "replication":"3 new and 9 historical real processes, plus 3 derived pose-only scores; one scene, no confidence intervals",
        "formats":["png","pdf"],"png_dpi":180,"publisher_requirements":"none specified; internal research figure"},indent=2)+"\n")
    print(json.dumps({"common_poses":len(common_stamps),"rpe_pairs":len(rpe_pairs),"methods":methods},indent=2))
