# Sensor-depth control and repeated replays

Recorded 2026-11-04 before collecting these trajectories. This follows the
initial 798-frame monocular/DA3 comparison and changes the input timeline to
the fixed 792 RGB frames with an associated sensor-depth image.

Question: how do monocular, fixed-conversion DA3 RGB-D and sensor-depth RGB-D
behave on identical inputs, and how much do repeated processes vary under the
same settings? This is a feasibility/engineering control, not a powered study.

- Select the 792 frames using the original loader's inclusive 20 ms one-to-one
  RGB/depth associations. Keep its original all-RGB GT pose associations;
  do not rematch GT after subsetting. Submit exactly these RGB images/timestamps
  to all three methods; preserve the earlier 798-frame results separately.
- Keep calibrated RGB intrinsics/distortion, original image grid, RGB ordering,
  ORB settings, seed zero, thread controls, paced replay, pinned libraries and
  largest-map export policy fixed. No parameter tuning or inference reruns.
- Reuse the verified prepared DA3 float32 nominal-metre TIFF files. Convert
  registered sensor depth from uint16 PNG by dividing by 5000. Preserve sensor
  holes as zero; no hole filling, interpolation, extra Freiburg correction,
  range filtering or fitted scale. Use the same RGB-D executable, float32
  format, depth factor 1, virtual baseline and close/far threshold for both.
- Sensor depth is an explicit sensor-measurement control, not a deployable
  learned-depth method or a noise-free oracle. Its native holes and RGB/depth
  timing offsets remain limitations of the comparison. GT world poses enter
  scoring only. No GT-based DA3 depth calibration is included here.
- Run three fresh processes per method, sequentially. Use a Latin-square order
  with seeded label/row permutations to place each method once in every
  within-block position, reducing systematic warm-up/order differences. Archive
  the exact nine-job schedule before replay. Seed controls remain identical;
  repeats measure asynchronous scheduling variability rather than new scenes.
- Preserve every run and failure; do not select the best run or silently replace
  a failure. Report exported coverage, initialization, loss events, map/keyframe
  counts and full-run scores separately for every process.
- For the primary comparison, intersect the evaluable RGB timestamps across
  all successful runs and refit each SE(3)/Sim(3) alignment on that same set.
  Report successful/attempted counts and common coverage if any run fails.
- Use 1 s ±20 ms RPE with identical pairs for every compared run. Indices refer
  to the 792 submitted frames: the six deliberately omitted source RGB frames
  are not tracking failures. Missing exports/GT within the submitted timeline
  still break RPE pairs. Record original dataset RGB indices separately.
- Report each result and median/minimum/maximum across three processes. No
  statistical significance or confidence interval is inferred: there is one
  independent sequence, and frames/RPE pairs are correlated observations.
  Three replays are a small operational stability check, not population evidence.
- SE(3) preserves native numerical scale; monocular units remain arbitrary.
  Sim(3) uses a GT-fitted scale correction and measures agreement after scaling.
  Offline cached depth and paced replay do not establish real-time throughput.

The Experimental Design and Scientific Visualization skills inform controls and
reporting; their software acknowledgement will appear in the results report.
