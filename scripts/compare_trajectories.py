#!/usr/bin/env python3
"""Refit and score both trajectories on identical RGB/reference timestamps."""
import argparse
import csv
import hashlib
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

from monocular_depth_slam.datasets.tum import read_groundtruth
from plot_monocular import broken_xy


ROOT = Path(__file__).resolve().parents[1]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--monocular", type=Path, default=ROOT/"results/orbslam3_mono_fr1_xyz/run_001")
    parser.add_argument("--depth-run", type=Path, default=ROOT/"results/orbslam3_da3_rgbd_fr1_xyz/run_001")
    parser.add_argument("--output", type=Path, default=ROOT/"results/trajectory_comparison_fr1_xyz")
    args = parser.parse_args()
    runs = {"monocular": args.monocular.resolve(), "da3_rgbd": args.depth_run.resolve()}
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifests = {name: json.loads((path/"manifest.json").read_text()) for name,path in runs.items()}
    cameras = {name: yaml.safe_load((path/"camera.yaml").read_text().replace("%YAML:1.0", ""))
               for name,path in runs.items()}
    for key, value in cameras["monocular"].items():
        if cameras["da3_rgbd"].get(key) != value:
            raise ValueError(f"RGB/ORB parameter differs between methods: {key}")
    if manifests["monocular"]["revisions"] != manifests["da3_rgbd"]["revisions"]:
        raise ValueError("Upstream source pins differ between methods")
    for name, other in (("monocular", "da3_rgbd"), ("da3_rgbd", "monocular")):
        subprocess.run([sys.executable, str(ROOT/"scripts/evaluate_monocular.py"),
                        "--run-dir", str(runs[name]), "--common-with", str(runs[other]),
                        "--output-dir", str(output/name)], check=True)
    summaries = {name: json.loads((output/name/"summary.json").read_text()) for name in runs}
    errors = {name: read_csv(output/name/"per_pose.csv") for name in runs}
    keys = ("input_index", "rgb_timestamp", "reference_timestamp")
    association = {name: [tuple(row[key] for key in keys) for row in rows] for name,rows in errors.items()}
    if association["monocular"] != association["da3_rgbd"]:
        raise ValueError("The two comparisons do not use identical RGB/reference associations")
    rpe = {name: read_csv(output/name/"rpe_sim3.csv") for name in runs}
    if [(r["start_timestamp"],r["end_timestamp"]) for r in rpe["monocular"]] != [(r["start_timestamp"],r["end_timestamp"]) for r in rpe["da3_rgbd"]]:
        raise ValueError("RPE pairs differ between methods")
    summary = {
        "question": "Single-sequence feasibility of fixed-conversion DA3 depth through upstream ORB-SLAM3 RGB-D",
        "comparison_script_sha256": sha256(Path(__file__)),
        "evaluator_sha256": sha256(ROOT/"scripts/evaluate_monocular.py"),
        "common_matched_poses": len(errors["monocular"]),
        "common_rpe_pairs": len(rpe["monocular"]),
        "independent_sequences": 1, "replays_per_method": 1,
        "confidence_intervals": "none; correlated frames/pairs are not independent replications",
        "alignment": "Each method refitted independently on the identical full common matched set; Sim3 uses GT scale",
        "runtime": "Offline cached depth; paced SLAM timings are descriptive, not end-to-end real-time measurements",
        "methods": {name: {"run": str(path.relative_to(ROOT)),
                           "run_manifest_sha256": sha256(path/"manifest.json"),
                           "common_summary_sha256": sha256(output/name/"summary.json"),
                           "coverage": summaries[name]["coverage"],
                           "tracking": summaries[name]["tracking"],
                           "native_scale": summaries[name]["native_scale"],
                           "se3": summaries[name]["se3"], "sim3": summaries[name]["sim3"]}
                    for name,path in runs.items()}}
    (output/"comparison.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    reference = read_groundtruth(output/"monocular/reference_matched.tum")
    gt = np.array([pose.translation for pose in reference])
    positions = {name: np.array([pose.translation for pose in read_groundtruth(output/name/"trajectory_sim3_aligned.tum")])
                 for name in runs}
    indices = [int(row["input_index"]) for row in errors["monocular"]]
    origin = float(manifests["monocular"]["rgb_inputs"][0]["timestamp"])
    times = np.array([float(row["rgb_timestamp"])-origin for row in errors["monocular"]])
    labels = {"monocular": "ORB monocular", "da3_rgbd": "DA3 + ORB RGB-D"}
    styles = {"monocular": ("#0072B2", "--"), "da3_rgbd": ("#D55E00", "-")}
    with plt.rc_context({"font.size": 9, "font.family": "DejaVu Sans", "axes.spines.top": False,
                         "axes.spines.right": False, "pdf.fonttype": 42, "savefig.facecolor": "white"}):
        fig, axes = plt.subplots(2, 2, figsize=(11.5,8), layout="constrained")
        axes[0,0].plot(*broken_xy(gt[:,0], gt[:,1], indices), color="black", linewidth=1.3, label="Motion-capture reference")
        for name in runs:
            color, line = styles[name]
            axes[0,0].plot(*broken_xy(positions[name][:,0], positions[name][:,1], indices),
                           color=color, linestyle=line, linewidth=1, label=labels[name])
            for axis, alignment in ((axes[0,1], "se3"), (axes[1,0], "sim3")):
                values = np.array([float(row[f"ate_{alignment}_m"])*100 for row in errors[name]])
                rmse = summaries[name][alignment]["ate_translation_m"]["rmse"]*100
                axis.plot(*broken_xy(times, values, indices), color=color, linestyle=line,
                          linewidth=.9, label=f"{labels[name]} ({rmse:.2f} cm RMSE)")
            pair_times = [(float(row["start_timestamp"])+float(row["end_timestamp"]))/2-origin for row in rpe[name]]
            pair_indices = [next(i for i,row in enumerate(errors[name]) if row["rgb_timestamp"] == pair["start_timestamp"]) for pair in rpe[name]]
            # Break lines at missing anchor indices, including the RPE exclusion around gaps.
            axes[1,1].plot(*broken_xy(pair_times, [float(row["translation_error_m"])*100 for row in rpe[name]], pair_indices),
                           color=color, linestyle=line, linewidth=.9, label=labels[name])
        axes[0,0].set(xlabel="x (m)", ylabel="y (m)", title="Sim(3) XY trajectories — GT-fitted scales")
        axes[0,0].set_aspect("equal", adjustable="datalim")
        axes[0,1].set(title="SE(3) position errors — no scale correction", xlabel="RGB time since first input (s)", ylabel="Position error (cm)")
        axes[1,0].set(title="Sim(3) position errors — GT-fitted scales", xlabel="RGB time since first input (s)", ylabel="Position error (cm)")
        axes[1,1].set(title="Sim(3) 1 s translation RPE — identical pairs", xlabel="Pair midpoint since first RGB input (s)", ylabel="Translation error (cm)")
        for axis in axes.flat:
            axis.legend(fontsize=7)
        fig.suptitle(f"freiburg1_xyz: {len(times)} common poses, {len(rpe['monocular'])} RPE pairs; one run per method", fontsize=12)
        fig.supxlabel("SE(3) uses monocular native numeric units and DA3 nominal metres. Missing observations are not interpolated.", fontsize=9)
        for extension in ("png", "pdf"):
            fig.savefig(output/f"comparison.{extension}", dpi=180)
        plt.close(fig)
    provenance = {
        "comparison_sha256": sha256(output/"comparison.json"),
        "script_sha256": sha256(Path(__file__)),
        "packages": {name: importlib.metadata.version(name) for name in ("matplotlib", "numpy", "pillow")},
        "inputs": {str((output/name/file).relative_to(ROOT)): sha256(output/name/file)
                   for name in runs for file in ("per_pose.csv", "rpe_sim3.csv", "trajectory_sim3_aligned.tum")},
        "formats": ["png", "pdf"], "png_dpi": 180,
        "normalization": "SE3 / Sim3 independently refitted on identical common poses",
        "missing_data": "NaN line breaks at missing indices; no interpolation",
        "replication": "one run per method, one sequence; no confidence intervals",
        "publisher_requirements": "none specified; internal research figure"}
    (output/"figure_provenance.json").write_text(json.dumps(provenance, indent=2)+"\n")
    print(json.dumps({name: {"SE3_ATE_RMSE_m": s["se3"]["ate_translation_m"]["rmse"],
                            "Sim3_ATE_RMSE_m": s["sim3"]["ate_translation_m"]["rmse"],
                            "GT_fitted_scale": s["sim3"]["scale"],
                            "Sim3_RPE_RMSE_m": s["sim3"]["rpe_translation_m"]["rmse"],
                            "RPE_rotation_RMSE_deg": s["sim3"]["rpe_rotation_deg"]["rmse"]}
                      for name,s in summaries.items()}, indent=2))
