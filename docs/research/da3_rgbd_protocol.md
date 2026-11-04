# DA3 / ORB-SLAM3 RGB-D feasibility protocol

Recorded on 2026-11-04 before collecting the first depth-assisted trajectory.
Question: does the fixed DA3 depth conversion yield a usable trajectory through
ORB-SLAM3's existing RGB-D interface, and how does that trajectory compare with
the preserved monocular baseline on this sequence?

This is one feasibility run per method on `freiburg1_xyz`, not an independently
replicated study. Frames and overlapping RPE pairs are correlated observations;
no significance test or confidence interval will be inferred from their count.
The existing monocular run is the historical control. Runtime comparisons are
descriptive because the runs are sequential and CPU scheduling is asynchronous.

- Submit the same 798 original RGB frames, timestamps, calibration, color order
  and ORB feature settings as the monocular run. Seed controls stay zero.
- Reuse the 792 verified single-image DA3 Metric Large predictions. Infer the
  six previously excluded RGB frames using the same checkpoint, processing
  settings and Mac environment; no sensor depth or poses enter inference.
- Convert canonical z-depth using processed mean focal length / 300, then
  bilinearly upsample to 640×480. Store float32 TIFF depth in nominal metres,
  with nonfinite/nonpositive values invalidated to zero. Do not use reference
  depth masks, range clipping, confidence filtering or any GT-fitted scale.
- Feed synchronized RGB and learned depth to `TrackRGBD`. Use the pinned
  upstream TUM1 RGB-D baseline parameter `Stereo.b=0.07732` and close/far
  multiplier `Stereo.ThDepth=40`; use `RGBD.DepthMapFactor=1` for float metres.
  These are RGB-D adapter assumptions, not an independently measured stereo
  baseline or a learned-depth uncertainty model. Do not tune them to the result.
- Preserve the first run, including failures. Export the optimized largest-map
  trajectory using the same upstream exporter/filter as the monocular run.
- Report all submitted/tracked/exported/reference-matched counts, initialization,
  losses, maps/keyframes, and SE(3)/Sim(3) ATE and 1 s ±20 ms RPE. Also refit and
  rescore both methods on the identical intersection of evaluable RGB timestamps,
  retaining missing-index gaps. Use fixed loader GT associations, no interpolation.
- SE(3) is the primary diagnostic of the depth-derived scale. Sim(3) explicitly
  fits a GT scale and measures trajectory agreement after that correction.
  Report the fitted factor even if results worsen. Avoid treating the unscaled
  monocular unit as an independently established metre.
- Report cached DA3 inference time separately from paced SLAM call/replay time.
  Offline cache replay does not demonstrate a real-time perception pipeline.

The earlier sensor comparison already revealed DA3 depth underestimation on
this sequence. This is an informed feasibility experiment, not a blinded test.
Any later GT-prefix calibration, oracle scaling, parameter tuning, repeated run
or sensor-depth control must be separately labeled and preserved.

The Experimental Design skill informed the controls and replication limits.
Its software acknowledgement and reference will be included in the result report.
