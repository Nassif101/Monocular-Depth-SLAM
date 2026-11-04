# TUM loader and pinned DA3 checkpoint

Validated on 2026-11-04 on the Apple M4 Mac. The loader tests also pass in the
existing Ubuntu 24.04 ARM64 container. This step prepares real data and verifies
one pretrained depth inference; no SLAM trajectory or accuracy benchmark has run.

## Downloads and storage

The explicitly approved download consists of:

| File | Bytes | Decimal size |
| --- | ---: | ---: |
| TUM `rgbd_dataset_freiburg1_xyz.tgz` | 448,204,271 | 448.204 MB |
| DA3 Metric Large `model.safetensors` | 1,336,734,448 | 1.337 GB |
| Checkpoint `config.json` | 847 | Less than 1 KB |
| Total | 1,784,939,566 | 1.785 GB |

The extracted dataset adds 479.3 MB of file contents. Keeping the archive,
extracted dataset and checkpoint therefore uses approximately 2.26 GB, plus
filesystem overhead. Weights and data remain gitignored.

Preview the exact sizes without any network request:

```bash
.venv/bin/python scripts/download_baseline.py
```

To explicitly download the listed assets, verify them and extract the dataset:

```bash
.venv/bin/python scripts/download_baseline.py --download
```

`--only dataset` and `--only checkpoint` restrict the selection. Existing complete
files are verified and reused, partial transfers can resume, and existing
sequence directories are preserved. Downloaded files must have the configured
size and SHA-256 before they are accepted. The TUM hash was observed during the
first download; it is **not a publisher-provided checksum**. The checkpoint
weight hash matches Hugging Face's published LFS metadata.

Sources: [official TUM download page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download),
[DA3 Metric Large files](https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/4010e39f3634a45bc60553321fb49fb760bd594e).

## Loader behavior

The reader is in `src/monocular_depth_slam/datasets/tum.py`. It parses `rgb.txt`,
`depth.txt` and `groundtruth.txt`, accepts comments, preserves original timestamp
strings, and rejects malformed, non-finite, duplicate or unordered records.
Missing indexed image files raise explicit errors.

Association chooses globally closest pairs first, with a one-to-one constraint
and an inclusive configurable tolerance. Ties prefer earlier RGB timestamps,
then earlier target timestamps. Decimal comparisons preserve decimal tolerance
boundaries. RGB remains the full primary timeline: unmatched depth and poses
are represented by `None` and reported, rather than dropping RGB frames. Poses
are not interpolated. Ground-truth quaternions retain TUM's `qx qy qz qw` order.

The selected camera configuration is in
[`tum_rgbd.example.yaml`](../configs/datasets/tum_rgbd.example.yaml). It uses the
Freiburg 1 RGB calibration from the pinned upstream ORB-SLAM3 `TUM1.yaml`,
including distortion, 640×480 image size and calibration source. It is not a
generic calibration for Freiburg 2/3 or another image size.

RGB decoding returns uint8 RGB. Sensor depth decoding requires a 16-bit PNG,
divides values by 5000 to obtain z-depth in metres, and returns a separate valid
mask. Zero/no-data pixels become NaN. TUM already applied its Freiburg depth
correction factors, so those factors are not applied again. Images are not
resized, undistorted or cropped; registered RGB/depth pixels remain unchanged.
This reader does not unproject depth or implement a distortion-aware RGB-D
adapter.

Reference: [official TUM formats, camera calibration and depth scaling](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats).

Run a full image/index validation and export RGB/depth associations:

```bash
.venv/bin/python scripts/inspect_tum.py --all-images \
  --output results/tum_freiburg1_xyz/inspection.json \
  --write-associations results/tum_freiburg1_xyz/rgb_depth_associations.txt
.venv/bin/python -m pytest -q
```

The real sequence contains 798 RGB frames, 798 depth frames and 3000 poses.
With the declared 20 ms tolerances:

| Result | Count |
| --- | ---: |
| RGB/depth matches | 792 |
| RGB/pose matches | 796 |
| RGB frames with both | 790 |
| RGB frames preserved | 798 |

All 1596 PNGs decoded with the expected dimensions/formats. The valid sensor
depth fraction was approximately 75.69%, with values from 0.5368 m to 7.1748 m.
The inspection report includes index SHA-256 values and unmatched timestamps.
These are data checks, not reconstruction or trajectory accuracy results.

## Pinned pretrained checkpoint

[`configs/da3/metric_large.yaml`](../configs/da3/metric_large.yaml) selects
`depth-anything/DA3METRIC-LARGE` at revision
`4010e39f3634a45bc60553321fb49fb760bd594e`, using the already pinned DA3 source.
The 334,171,394-parameter checkpoint loaded and ran on native Metal.

Run the one-frame check with local files only:

```bash
bash scripts/setup_da3.sh
HF_HUB_OFFLINE=1 .venv-da3-macos/bin/python scripts/check_checkpoint.py --device mps
```

On the future configured NVIDIA environment, use `.venv-da3-cuda/bin/python`
and `--device cuda`; CUDA execution remains untested on this Mac.

The check used RGB timestamp `1305031102.175304`, preprocessing resolution 504
and `upper_bound_resize`, producing a finite `[1, 378, 504]` depth tensor. It
passed no sensor depth or ground-truth poses into the model. Local output and
provenance are stored in `results/checkpoint_smoke/`, including weight hashes,
source revision/local diff hash, input hash, timestamps, intrinsics and runtime.

The Metric Large raw output is canonical depth, **not directly metres**. Upstream
specifies:

```text
depth_metres = raw_canonical_depth * mean(fx_processed, fy_processed) / 300
```

The focal lengths must correspond to the processed image's resize/crop, rather
than the original camera resolution. The smoke check deliberately stores raw
canonical depth and records that no metre conversion or metric-scale validation
was performed. It does not claim RGB-D compatibility or depth accuracy.

Reference: [pinned upstream model notes and conversion formula](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/3d835ec1a5802d64a8b8b15f817a1ab54809bfe4/README.md#-model-cards).

The subsequent [full depth comparison](research/da3_depth_comparison.md) now
implements the prediction cache, processed-intrinsics metre conversion and
pixel-grid alignment, and evaluates all 792 matched frames. It reports 10.22%
AbsRel and 0.1569 m frame-averaged RMSE, with separate GT-scale diagnostics.
This section's original smoke outputs remain raw canonical depth.
