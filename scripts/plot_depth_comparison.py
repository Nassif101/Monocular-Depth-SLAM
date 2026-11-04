#!/usr/bin/env python3
"""Generate standalone figures from an existing depth comparison, without inference."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import numpy as np

from monocular_depth_slam.datasets.tum import load_config, read_rgb


ROOT = Path(__file__).resolve().parents[1]


def save_figure(figure, directory, name):
    figure.savefig(directory/f"{name}.png", dpi=180, bbox_inches="tight")
    figure.savefig(directory/f"{name}.pdf", bbox_inches="tight")
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=ROOT / "results/depth_comparison_fr1_xyz")
    args = parser.parse_args()
    summary = json.loads((args.results/"summary.json").read_text())
    rows = json.loads((args.results/"per_frame.json").read_text())
    histogram = np.load(args.results/"histograms.npz")
    figures = args.results/"figures"
    figures.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
                         "savefig.facecolor": "white", "font.family": "DejaVu Sans"})
    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5), constrained_layout=True)
    edges = histogram["gt_pred_edges"]
    image = axes[0, 0].pcolormesh(edges, edges, histogram["gt_pred_counts"].T,
                                  norm=colors.LogNorm(vmin=1), cmap="viridis", rasterized=True)
    axes[0, 0].plot([0, edges[-1]], [0, edges[-1]], "--", color="tomato", linewidth=1.2)
    axes[0, 0].set(xlabel="TUM sensor depth (m)", ylabel="DA3 fixed metric depth (m)",
                   title="Unfitted metric predictions, all valid sensor pixels")
    axes[0, 0].set_aspect("equal")
    fig.colorbar(image, ax=axes[0, 0], label="Pixel count", shrink=0.85)
    time = [row["time_seconds"] for row in rows]
    ratios = np.array([row["raw"]["median_pred_over_gt"] for row in rows])
    axes[0, 1].plot(time, ratios, linewidth=0.9, color="#14668f")
    axes[0, 1].axhline(1, color="black", linestyle="--", linewidth=1)
    split = summary["heldout_gt_anchored_scale_DIAGNOSTIC"]["calibration_frames"]
    axes[0, 1].axvspan(time[0], time[split-1], color="#e7b76a", alpha=0.25,
                       label="GT calibration prefix (diagnostic only)")
    axes[0, 1].set(xlabel="RGB time from first evaluated frame (s)",
                   ylabel="Frame median DA3 / sensor depth",
                   title="Scale bias varies over the sequence")
    axes[0, 1].legend(fontsize=8)
    # Sparse far bins remain in the full metrics/table, but must not visually
    # dominate the supported distance trend with only a handful of pixels.
    bins = [row for row in summary["by_sensor_depth"] if row["pixels"] >= 1000]
    sparse_pixels = sum(row["pixels"] for row in summary["by_sensor_depth"] if row["pixels"] < 1000)
    centres = [(row["lower_m"]+row["upper_m"])/2 for row in bins]
    axes[1, 0].plot(centres, [row["bias_m"] for row in bins], "o-", label="Signed mean error")
    axes[1, 0].plot(centres, [row["mae_m"] for row in bins], "s--", label="Mean absolute error")
    axes[1, 0].axhline(0, color="black", linewidth=0.7)
    axes[1, 0].set(xlabel="Sensor-depth bin centre (m)", ylabel="Depth error (m)",
                   title=f"Populated depth bins (≥1,000 pixels; {sparse_pixels} sparse pixels omitted)")
    axes[1, 0].legend(fontsize=8)
    rel_edges = histogram["signed_relative_edges"]
    counts = histogram["signed_relative_counts"]
    axes[1, 1].stairs(counts/counts.sum()*100, rel_edges, fill=True, color="#14668f", alpha=0.8)
    axes[1, 1].axvline(0, color="black", linestyle="--", linewidth=1)
    omitted = summary["primary_unfitted_metric"]["scored_pixels"] - int(counts.sum())
    axes[1, 1].set(xlabel="(DA3 − sensor) / sensor", ylabel="Fraction of plotted pixels (%)",
                   title=f"Signed relative errors (outside ±100%: {omitted:,} pixels)")
    fig.suptitle("DA3 Metric Large vs TUM freiburg1_xyz — fixed conversion, no GT scale fit", fontsize=13)
    save_figure(fig, figures, "depth_comparison_overview")

    sequence = load_config(ROOT/summary["protocol"]["dataset_config"], project_root=ROOT)
    by_timestamp = {frame.rgb.timestamp_text: frame for frame in sequence.frames}
    selected = summary["illustration_frames"]
    fig, axes = plt.subplots(len(selected), 4, figsize=(14, 2.35*len(selected)),
                             constrained_layout=True, squeeze=False)
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_bad("#cccccc")
    error_cmap = plt.get_cmap("RdBu_r").copy()
    error_cmap.set_bad("#cccccc")
    for i, row in enumerate(selected):
        stamp = row["rgb_timestamp"]
        example = np.load(args.results/f"example_{row['matched_frame_index']:03d}.npz")
        gt, pred, valid = example["reference_metres"], example["prediction_metres"], example["valid_reference"]
        axes[i, 0].imshow(read_rgb(by_timestamp[stamp].rgb, sequence.camera))
        depth_image = axes[i, 1].imshow(np.where(valid, gt, np.nan), cmap=cmap, vmin=0, vmax=4)
        axes[i, 2].imshow(pred, cmap=cmap, vmin=0, vmax=4)
        relative = np.full(gt.shape, np.nan, dtype=np.float32)
        relative[valid] = (pred[valid]-gt[valid])/gt[valid]
        error_image = axes[i, 3].imshow(relative, cmap=error_cmap, vmin=-0.5, vmax=0.5)
        axes[i, 0].set_ylabel(f"t = {row['time_seconds']:.2f} s\nAbsRel {row['raw']['abs_rel']*100:.1f}%")
        for ax in axes[i]:
            ax.set_xticks([])
            ax.set_yticks([])
    for ax, title in zip(axes[0], ("RGB", "Sensor depth (m)", "DA3 fixed metric depth (m)",
                                  "Signed relative error")):
        ax.set_title(title)
    fig.colorbar(depth_image, ax=axes[:, 1:3].ravel().tolist(), label="Depth (m), display clipped to 0–4", shrink=0.7)
    fig.colorbar(error_image, ax=axes[:, 3].ravel().tolist(), label="Relative error (display clipped to ±50%)", shrink=0.7)
    fig.suptitle("Preselected first / middle / last matched frames; grey = missing sensor depth", fontsize=13)
    save_figure(fig, figures, "depth_comparison_examples")
    print(f"Saved PNG and PDF figures to {figures}")
