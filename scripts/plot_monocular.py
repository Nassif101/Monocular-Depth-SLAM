#!/usr/bin/env python3
"""Render trajectory alignment, per-pose errors, tracking states and scale diagnostics."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from monocular_depth_slam.datasets.tum import load_config, read_groundtruth


ROOT = Path(__file__).resolve().parents[1]


def broken_xy(x, y, indices):
    """Insert NaNs instead of drawing through unavailable observations."""
    xs, ys = [], []
    for i in range(len(x)):
        if i and indices[i] != indices[i-1]+1:
            xs.append(np.nan)
            ys.append(np.nan)
        xs.append(x[i])
        ys.append(y[i])
    return xs, ys


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=ROOT/"results/orbslam3_mono_fr1_xyz/run_001")
    args = parser.parse_args()
    evaluation = args.run_dir/"evaluation"
    summary = json.loads((evaluation/"summary.json").read_text())
    manifest = json.loads((args.run_dir/"manifest.json").read_text())
    reference = read_groundtruth(evaluation/"reference_matched.tum")
    estimates = {alignment: read_groundtruth(evaluation/f"trajectory_{alignment}_aligned.tum")
                 for alignment in ("se3", "sim3")}
    gt = np.array([pose.translation for pose in reference])
    positions = {name: np.array([pose.translation for pose in records]) for name,records in estimates.items()}
    with (evaluation/"per_pose.csv").open() as stream:
        errors = list(csv.DictReader(stream))
    with (args.run_dir/"tracking.csv").open() as stream:
        tracking = list(csv.DictReader(stream))
    sequence = load_config(ROOT/manifest["protocol"]["dataset_config"], project_root=ROOT)
    origin = sequence.rgb_records[0].timestamp
    time = np.array([float(row["rgb_timestamp"])-origin for row in errors])
    indices = [int(row["input_index"]) for row in errors]
    figures = evaluation/"figures"
    figures.mkdir(exist_ok=True)
    style = {"font.size": 9, "font.family": "DejaVu Sans", "axes.spines.top": False,
             "axes.spines.right": False, "pdf.fonttype": 42, "savefig.facecolor": "white"}
    with plt.rc_context(style):
        fig, axes = plt.subplots(2, 2, figsize=(11.5, 8), layout="constrained")
        for axis, dims, label in ((axes[0,0], (0,1), "XY"), (axes[0,1], (0,2), "XZ")):
            a,b = dims
            axis.plot(*broken_xy(gt[:,a], gt[:,b], indices), color="black", linewidth=1.5,
                      label="Matched motion-capture reference")
            axis.plot(*broken_xy(positions["sim3"][:,a], positions["sim3"][:,b], indices),
                      color="#0072B2", linestyle="--", linewidth=1.1, label="ORB-SLAM3, Sim(3) aligned")
            axis.scatter(positions["sim3"][0,a], positions["sim3"][0,b], color="#0072B2", marker="s", s=24)
            axis.set(xlabel=f"{'xyz'[a]} (m)", ylabel=f"{'xyz'[b]} (m)", title=f"{label} projection — GT-fitted scale")
            axis.set_aspect("equal", adjustable="datalim")
            axis.legend(fontsize=7)
        for alignment, color, line in (("se3", "#D55E00", "-"), ("sim3", "#0072B2", "--")):
            error = np.array([float(row[f"ate_{alignment}_m"])*100 for row in errors])
            rmse = summary[alignment]["ate_translation_m"]["rmse"]*100
            axes[1,0].plot(*broken_xy(time, error, indices), color=color, linestyle=line, linewidth=0.9,
                            label=f"{alignment.upper()} (RMSE {rmse:.2f} cm)")
        axes[1,0].set(xlabel="RGB time since first input (s)", ylabel="Absolute position error (cm)",
                       title="Identical matched poses; gaps retained")
        axes[1,0].legend(fontsize=8)
        timeline = [float(row["timestamp"])-origin for row in tracking]
        states = [int(row["state"]) for row in tracking]
        n_init, n_ok = states.count(1), sum(state in (2,5) for state in states)
        axes[1,1].step(timeline, states, where="post", color="#009E73", linewidth=1.2)
        axes[1,1].set(yticks=[0,1,2,3,4,5], yticklabels=["No image","Initializing","OK","Recently lost","Lost","KLT"],
                       ylim=(-.3, 5.3), xlabel="RGB time since first input (s)",
                       title=f"{len(tracking)} inputs: {n_init} initializing, {n_ok} tracked, {len(states)-n_init-n_ok} other")
        coverage = summary["coverage"]["trajectory_fraction_of_inputs"]*100
        method = manifest["protocol"].get("method", "ORB-SLAM3 monocular")
        fig.suptitle(f"{method} — freiburg1_xyz, {args.run_dir.name}; {coverage:.2f}% trajectory coverage", fontsize=12)
        for extension in ("png", "pdf"):
            fig.savefig(figures/f"trajectory_and_tracking.{extension}", dpi=180)
        plt.close(fig)

        windows = summary["window_scale_diagnostic"]
        centres = [(row["start_seconds"]+row["end_seconds"])/2 for row in windows]
        local = [row["exploratory_local_sim3_scale_fit"].get("residual_estimate_scale_percent", np.nan)
                 for row in windows]
        length = [row["relative_difference_percent"] for row in windows]
        fig, axes = plt.subplots(1, 2, figsize=(11.5, 3.7), layout="constrained")
        axes[0].plot(centres, local, "o-", color="#0072B2", linewidth=1)
        axes[1].plot(centres, length, "s--", color="#D55E00", linewidth=1)
        for axis in axes:
            axis.axhline(0, color="black", linewidth=.7)
            axis.set(xlabel="Window centre from first evaluated pose (s)", ylabel="Difference (%)")
        axes[0].set_title("Exploratory local scale residual after global Sim(3)")
        axes[1].set_title("Path-length difference (includes pose jitter)")
        axes[1].annotate("Short final window; low reference motion", xy=(centres[-1], length[-1]),
                          xytext=(.05,.9), textcoords="axes fraction", fontsize=8,
                          arrowprops={"arrowstyle": "->", "color": "#555555"})
        fig.suptitle("5 s windows; last window is partial. These are GT-based diagnostics.", fontsize=11)
        for extension in ("png", "pdf"):
            fig.savefig(figures/f"scale_diagnostics.{extension}", dpi=180)
        plt.close(fig)
    (figures/"provenance.json").write_text(json.dumps({
        "plot_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "summary_sha256": hashlib.sha256((evaluation/"summary.json").read_bytes()).hexdigest(),
        "per_pose_sha256": hashlib.sha256((evaluation/"per_pose.csv").read_bytes()).hexdigest(),
        "packages": {name: importlib.metadata.version(name) for name in ("matplotlib", "numpy", "pillow")},
        "source_summary": "evaluation/summary.json", "source_errors": "evaluation/per_pose.csv",
        "trajectory": "largest-map optimized poses; primary online state OK/OK_KLT",
        "alignment": "SE3 and Sim3 fitted on identical matched pose set",
        "missing_data": "NaNs inserted across missing RGB input indices; no interpolation",
        "uncertainty": "single replay; no independent-replicate confidence intervals",
        "formats": ["png", "pdf"], "png_dpi": 180,
        "publisher_requirements": "none specified; internal research figures",
        "description": "Sim3-aligned XY/XZ path overlays, SE3/Sim3 position errors, all-frame states; separate exploratory scale diagnostics"
    }, indent=2)+"\n")
    print(f"Saved figures to {figures}")
