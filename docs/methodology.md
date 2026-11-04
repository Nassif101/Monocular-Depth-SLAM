# Evaluation methodology

## Comparisons

- Monocular ORB-SLAM3 (no learned depth)
- ORB-SLAM3 RGB-D-style adapter using DA3 depth (once implemented)
- Oracle depth calibration as a **diagnostic upper-bound** only
- Fixed-budget and adaptive depth scheduling (future)

## Metrics

- **SE(3)-aligned ATE:** metric-scale trajectory accuracy (no scale fitting)
- **Sim(3)-aligned ATE:** trajectory shape after scale fitting
- Relative pose error at declared frame/time intervals
- Scale drift, tracking loss/recovery, inference count, GPU time, wall time

## Controls

Use identical image subsets, camera calibration, association tolerance and evaluation timestamps. Report missing/failed trajectories, multiple seeds where stochastic, and both successes and failures. No tuning on the test sequence without disclosure.

## Depth conventions

Document whether predictions are z-depth, range, inverse depth, or disparity; include valid masks, unit conversion, resizing/cropping and pixel alignment. A learned prediction must not be labelled metric without verification.
