# Mathematical findings: scale, depth bias, and fusion

Date: 2026-11-04. Status: mathematical analysis and synthetic demonstrations, **not** a validated SLAM method.

## 1. Global monocular scale gauge

For calibrated monocular reprojection, transforming all landmarks and camera translations as `X_k' = s X_k`, `t_i' = s t_i` (with fixed camera rotations) preserves normalized image projections for positive `s`. Fixing the first camera pose removes the rigid SE(3) gauge but **not** global scale.

## 2. Plücker ray factor is not automatically a metric anchor

The analyzed PRISM-SLAM paper defines a ray-distance residual:

```text
e_ray = || d_i × X_k + m_i || / ||d_i||
m_i = t_i × d_i
```

Equivalently, `e_ray = ||d_i × (X_k - t_i)|| / ||d_i||`. If `t_i` and `X_k` are jointly optimized and scaled, a zero residual remains zero. At nonzero residual, scaling geometry scales the residual rather than identifying a physically correct metric scale. **This challenges the factor-alone scale claim, not the entire PRISM-SLAM system**, which also discusses depth priors and scale filtering. Independent metric ray origins or explicit metric depth can anchor scale.

## 3. Unknown depth bias preserves a gauge

For predicted log-depth `y_ik = log(z_ik) + beta_i + noise`, use

```text
r_ik = log(z_ik) + beta_i - y_ik
```

Under `z_ik' = s z_ik`, `beta_i' = beta_i - log(s)`, the residual is unchanged. This applies to a free shared bias and to free per-frame biases. Relative bias drift may be estimated; the common offset cannot be identified from monocular geometry and biased depth alone. A bias prior anchors the gauge *by assumption*, not by independent observation.

## 4. Robust depth factors

A joint synthetic bundle adjustment demonstrated that explicit metric depth factors can resolve scale if their units/calibration are trustworthy. Robust Huber/Cauchy depth factors suppressed large outliers compared with quadratic WLS. None corrected a shared 15% depth bias. Robustify depth factors separately rather than blindly applying the same robust loss to reprojection residuals.

## 5. Two-level estimator

Let `alpha_t = log(scale_t)` and `beta_t = log(depth_bias_t)`. A model using depth alignment, simulated relative geometric scale changes, and bias smoothness retains an exact common scale/bias null direction unless a calibration or independent metric anchor is supplied. Numerical minimum-norm solutions do not establish metric scale.

## 6. What we have *not* established

- No ORB-SLAM3 + DA3 sequence has been executed.
- No actual learned-depth checkpoint has been benchmarked.
- No verified real-time GPU latency or tracking improvement.
- No complete reproduction of PRISM-SLAM or SDMFusion.
- No established novelty claim for adaptive depth scheduling or bias adaptation.

## Practical design decision

Keep global metric anchoring distinct from relative depth-bias drift estimation. Evaluate a fixed-depth-call budget against an observability/information-aware depth inference scheduler. Report metric SE(3) ATE separately from scale-fitted Sim(3) ATE.
