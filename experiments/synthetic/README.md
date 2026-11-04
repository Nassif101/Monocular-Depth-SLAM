# Synthetic research experiments

The 2026-11-04 research work included standalone NumPy/SciPy experiments for ray-factor scale observability, scalar robust depth fusion, joint bundle adjustment, joint SE(3) with latent learned-depth bias, and a two-level scale/bias estimator.

See `docs/research/experiments.md` for setup, numeric findings and limitations. The original full scripts, reports and CSVs are available as a separate research archive from the conversation and should be imported unchanged, then rerun with pinned dependencies before using any figures in publications.

A small executable gauge sanity check is included here as `scale_bias_gauge_demo.py`. It illustrates an exact invariance and is **not** a substitute for the historical full experiments.
