# Synthetic experiment evidence ledger

Date: 2026-11-04. Historical reports and CSV outputs are included in the accompanying research archive from the conversation. The figures below are **not** independent benchmark replications.

| Experiment | Model and trials | Principal observation |
| --- | --- | --- |
| Observability | 4 cameras, 10 landmarks, first camera fixed, rotations known | Reprojection and joint ray factors: rank 38/39; metric depth or independent metric ray origins: rank 39/39 |
| Scalar depth fusion | 80 synthetic depths, 160 randomized replications/scenario | WLS ~0.30% scale error with unbiased noise; ~14.98% with shared +15% bias; robust losses mitigate outliers |
| Joint bundle adjustment | 4 cameras, 18 landmarks, fixed rotations, 5 trials/scenario | Clean depth WLS median 0.378% scale error vs vision-only 59.156% (scale arbitrary); with 20% outliers WLS 1.584%, Huber 0.356%, Cauchy 0.138%; with 15% bias all depth methods ~14.5% |
| SE(3) + learned-depth bias | 4 cameras, 12 landmarks, optimized rotations/translations, partial visibility, 3 seeds/baseline | Unknown shared bias rank 54/55; free per-frame bias 57/58; calibrated bias and 70 cm baseline 0.45% median scale error vs uncalibrated 15.51% |
| Two-level scale/bias estimator | 35 time steps, 4 corrupted depth samples/trial, 30 trials | Median scale error: no bias model 16.09%; wrong calibration 14.95%; unanchored numerical convention 7.75% (**not identifiable**); correct calibration 2.09%; synthetic metric anchors 1.33% |

## Interpretation caveats

- The vision-only 59% figure reflects initialization under an unobservable gauge; it is **not** a meaningful general estimate of monocular SLAM error.
- Bias-observability rank depends on the specified gauge fixing, measurement model, and numerical threshold.
- The two-level estimator used **simulated relative-scale observations**, not measurements derived from real image feature tracks.
- Robust loss outcomes are scenario-specific and not universal performance rankings.
- The reports are exploratory; check and rerun all scripts before citing figures in a thesis or paper.

## Reproduction requirements

Use Python 3.10+, NumPy and SciPy. Capture exact versions and seeds; regenerate results from the archived scripts. Compare regenerated CSVs and report discrepancies. Do not silently replace the original experimental outputs.
