# Literature and novelty notes (exploratory)

Date: 2026-11-04. This is **not** a systematic literature review and does not establish novelty.

## Close comparisons

- **PRISM-SLAM** (Im et al., 2026): https://arxiv.org/abs/2605.19257 . Reported asynchronous Depth Anything 3 integration, Plücker ray factors, dynamic-scene uncertainty gating, and metric scale estimation. Examine the exact source equations and implementation availability before claiming its ray factors alone anchor scale. Its author-reported numbers are not our benchmark results.
- **SDMFusion** (Cen et al., 2026): https://doi.org/10.1038/s41598-026-41208-9 . Depth Anything V2 with ORB-SLAM3, depth-scale optimization, dynamic filtering and dense mapping. Closest architecture comparison.
- **ADEmono-SLAM** (Zhou et al., 2025): https://doi.org/10.3390/electronics14204126 . Depth-aided geometric monocular SLAM and dynamic filtering.
- **Depth Anything with Any Prior** (ICLR 2026): https://proceedings.iclr.cc/paper_files/paper/2026/hash/990a88cb9b87b9d8e9922d6a24aa41f7-Abstract-Conference.html . Sparse metric priors for dense depth; not metric scale from pure RGB alone.
- **MoGe2-SLAM** (2026): https://www.mdpi.com/1424-8220/26/20/6388 . Uses a camera-height/ground-plane metric prior; distinguish that external assumption from unanchored monocular recovery.

## Candidate hypothesis

At equal learned-depth inference budgets, a geometry/observability-driven keyframe selection policy may improve metric-scale accuracy, tracking robustness or accuracy-per-GPU-second compared with fixed-period or every-keyframe inference.

## Novelty risks

Keyframe selection, uncertainty gating, adaptive inference, scale/shift depth calibration, and bias priors are all established ideas in adjacent literature. The novelty must be defined by a specific method and a controlled result, not merely combining them.

## Needed audit

Search for adaptive depth inference in visual SLAM, information-gain-based keyframe scheduling, uncertainty-aware depth fusion, depth affine alignment, scale observability, and learned depth calibration. Maintain claim/source links and note what each comparison assumes about metric anchors.
