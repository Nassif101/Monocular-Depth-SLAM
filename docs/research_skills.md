# Research skills for this project

Installed on 2026-11-04 as user-level Codex skills, under `~/.codex/skills`
(or `$CODEX_HOME/skills` when configured). They become available on the next
conversation turn. The project stores provenance, not copies of the skill
collection, in [`configs/research_skills.json`](../configs/research_skills.json).

The installation includes 12 skills, 169 files, and 1,706,280 bytes of selected
file contents. Git transport also transfers repository metadata. Every installed
file was verified against its pinned upstream Git blob, and every `SKILL.md`
name/description was validated. Existing system skills were preserved.

## Installed skills and uses

Ten skills come from
[K-Dense's scientific-agent-skills](https://github.com/K-Dense-AI/scientific-agent-skills/tree/a25cf9ec00e2745d03f8704acf6f6919694ccd9a),
the current name of `claude-scientific-skills`:

| Skill | Use in monocular depth/SLAM research |
| --- | --- |
| `scientific-writing` | Draft methods, results, limitations and manuscript sections with evidence provenance. |
| `scientific-critical-thinking` | Audit claims, assumptions, confounders, leakage and whether conclusions follow from evidence. |
| `literature-review` | Search and synthesize SLAM/depth literature with explicit coverage and screening criteria. |
| `citation-management` | Verify paper metadata, DOI/arXiv references and BibTeX entries. |
| `peer-review` | Review our reports/manuscripts for methodological, reproducibility and reporting gaps. |
| `experimental-design` | Plan controls, ablations, replication and equal-inference-budget experiments before running them. |
| `statistical-analysis` | Choose analysis units, quantify effects and uncertainty, and handle paired or dependent observations. |
| `scientific-visualization` | Produce and audit scientific figures with clear units, masks, uncertainty and export settings. |
| `sympy` | Check exact algebra, symbolic derivatives, Jacobians, null directions and scale/bias invariances. |
| `uncertainty-and-units` | Check dimensional consistency, focal/depth conversions and measurement-uncertainty propagation. |

Two skills come from
[OpenAI's curated collection](https://github.com/openai/skills/tree/49f948faa9258a0c61caceaf225e179651397431/skills/.curated):

| Skill | Use |
| --- | --- |
| `pdf` | Read papers and produce visually checked PDF reports. |
| `jupyter-notebook` | Build reproducible exploratory or explanatory notebooks. |

## Suggested prompts

```text
$scientific-critical-thinking Audit docs/research/da3_depth_comparison.md.
Separate measured results, interpretations, assumptions and unsupported claims.
Check GT calibration, temporal dependence and generalization limitations.
```

```text
$sympy Check the scale/bias invariance of log(z) + beta - y under
z -> s*z and beta -> beta - log(s). State z > 0 and s > 0 explicitly,
use exact expressions, and distinguish an identity from a numerical test.
```

```text
$scientific-writing Draft a methods and results section from the depth
evaluation protocol and results/depth_comparison_fr1_xyz/summary.json.
Link quantitative claims to artifacts and retain all stated limitations.
```

```text
$experimental-design Plan the monocular versus depth-assisted SLAM ablation.
Keep image subsets, camera calibration and learned-depth inference budgets
controlled, and define the independent experimental units before analysis.
```

## Evidence rules for this research

Use these workflows to make reasoning and evidence inspectable:

- A mathematical claim needs a derivation with explicit assumptions. SymPy can
  check algebra, derivatives and symbolic examples; numerical agreement or a
  sampled Jacobian rank alone is not a general mathematical proof.
- An empirical claim needs a reproducible protocol, exact data/model revisions,
  recorded transformations and run artifacts. Keep synthetic experiments,
  sensor-depth comparisons and actual SLAM trajectory results distinct.
- A literature claim needs an opened primary source and an exact supporting
  proposition, equation or section. A correct citation does not by itself show
  that the cited paper supports the claim. Mark unresolved support explicitly.
- Pixels and consecutive frames are dependent observations. Sequence-level
  conclusions need suitable independent sequences/runs; 184 million scored
  pixels do not mean 184 million independent experimental replications.
- Preserve the distinction between unfitted metric results and GT-fitted
  diagnostics. GT calibration is an explicit metric anchor. It is not evidence
  that a free depth-bias state makes monocular scale observable.
- Manuscript drafts retain uncertainty, provenance and human verification
  status. Publication-facing claims and references require accountable review.

The existing project instructions and the user's task authorization remain in
force when a skill is used. Installing a skill does not authorize external
uploads, paid API calls, additional downloads or publication.

## Runtime dependencies

This installation added skill instructions, references, templates and helper
scripts. It did not add Python/system packages, connect accounts or configure
API keys. Core writing/peer-review CLIs use the Python standard library. Other
bundled tools have their own dependencies; inspect their `SKILL.md` before use.

Examples include SymPy for symbolic tools; `pint`/`uncertainties` for numerical
unit/uncertainty helpers; extra statistical packages for some analyses; and
Pandoc/XeLaTeX for an optional literature-review PDF pipeline. Several skills
also describe optional AI schematic services. Those are separate capabilities,
not prerequisites for ordinary claim review or prose drafting.

Use the project's existing plotting and PDF workflows where appropriate. Keep
any new research-tool dependency environment separate from the working DA3
environment, respect existing locks, and report sizes before further downloads.
The installed skills do not modify the SLAM or depth-evaluation pipeline.
