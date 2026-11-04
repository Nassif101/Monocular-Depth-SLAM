# Research handoff (2026-11-04)

This directory preserves the research conclusions and the implementation plan from the monocular SLAM investigation. **Synthetic results are not real-world SLAM benchmarks.**

## Read first

1. [Findings and mathematical constraints](findings.md)
2. [Experimental evidence ledger](experiments.md)
3. [Literature and novelty notes](literature.md)
4. [Codex implementation brief](../../docs/CODEX_HANDOFF.md)

## Key result

Pure monocular reprojection and camera/landmark-coupled Plücker ray-distance factors do not remove the global scale gauge. A free global or per-frame multiplicative learned-depth bias also preserves a scale/bias gauge. Fixing a bias or adding a prior can remove the numerical null direction, but the metric scale then depends on calibration assumptions. A genuinely independent metric anchor is needed to claim absolute metric recovery.

## Important provenance

Research experiments were developed and run as standalone SciPy scripts outside this repository. The research reports described here are **historical synthetic findings**. Do not claim they validate DA3 or ORB-SLAM3, or reproduce PRISM-SLAM. An archived bundle of the original standalone scripts, reports, and CSV outputs was also produced in the conversation for import; this GitHub handoff currently includes a small standalone gauge sanity-check and detailed experimental descriptions, not a port of every historical script.
