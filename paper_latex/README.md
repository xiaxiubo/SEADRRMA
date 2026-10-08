# TRACE Manuscript

The current manuscript entry point is `paper_TRACE_title_intro.tex`. The previous
DR-RMA draft is preserved as `paper_dr_rma.tex`. This directory includes both
manuscripts, their PDF snapshots, bibliography, all existing figure exports, editable
SVG/PPTX artwork, plotting scripts, and dated experiment/revision notes.

## Build the current manuscript

Install TeX Live or MiKTeX with IEEEtran, algorithm/algorithmic, subfig, booktabs,
multirow, amsmath, bm, and the packages listed in the manuscript preamble.

From the repository root:

```bash
python tools/validate_paper.py
python tools/build_paper.py
```

Or compile directly from this directory:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error paper_TRACE_title_intro.tex
```

To build the previous draft, run `python tools/build_paper.py --legacy` from the
repository root. PDF files are draft snapshots, not a statement of submission or
publication status. Dated planning documents retain their original status and may
not describe work completed later.

## Figure provenance and regeneration

All figure files referenced by both manuscripts are included, so compilation does
not require training data, checkpoints, or hardware access. PDF/PNG/SVG/TIFF exports
and `fig2_TRACE_v2.pptx` are retained for future editing.

`generate_simulation_figures.py` uses recorded simulation results. Full regeneration
requires restoring these private artifacts at the repository-relative paths below:

```text
algorithm_comparison/results/paper_zero_phase_final_20260716/logs_{drrma,rma,ppo,pd-dob,lqr}.npz
figures/attention_online/paper_zero_phase_0p5sin0p3hz_fc0p2_seed2/trace_mechanism.npz
figures/attention_online/final_dagger_raw_full_20260713_summary.json
checkpoints_attention_online/final_v1/fast_attn_v2.pth
```

The required model definition `attention_fast_v2.py` is included at the repository
root. The script also accepts `--comparison-dir` and `--output-dir`.

`hardware_experiment_plot.py` reads a supplied hardware CSV; its required columns
are documented in the script. The root `plot_hardware_four_algorithms.py` expects
the private CSV files named in its `FILES` mapping.

`generate_attention_figure.py` is a historical illustrative script that constructs
synthetic attention weights. Its generated attention details must not be described
as measured attention or used to substantiate model performance.

Python plotting requirements are in `requirements-figures.txt`. Compiling the
manuscript does not require installing those Python packages.

## Previous draft notes

This folder contains all LaTeX-related files for the DR-RMA paper.

## Structure

```
paper_latex/
├── paper_dr_rma.tex          # Main paper file
├── references.bib             # Bibliography
├── figures/                   # All figures
│   ├── figs/                  # System overview figures
│   │   ├── fig1png.png        # SEA system overview
│   │   └── fig2png.png        # DR-RMA architecture
│   ├── tracking_comparison.png
│   ├── baseline_comparison_test2_inertia_steps.png
│   ├── fig1_attention_heatmap.png
│   ├── fig2_inertia_estimation.png
│   └── ... (other result figures)
├── .latexmkrc                 # LaTeXmk configuration
└── README.md                  # This file
```

## Compilation

### Using LaTeX Workshop in VSCode

1. Open `paper_dr_rma.tex` in VSCode
2. Press `Ctrl+Alt+B` (or `Cmd+Option+B` on Mac) to build
3. Press `Ctrl+Alt+V` (or `Cmd+Option+V` on Mac) to view PDF

### Using Command Line

```bash
# Full compilation with bibliography
pdflatex paper_dr_rma.tex
bibtex paper_dr_rma
pdflatex paper_dr_rma.tex
pdflatex paper_dr_rma.tex

# Or use latexmk (recommended)
latexmk -pdf paper_dr_rma.tex

# Clean auxiliary files
latexmk -c
```

## Required Figures

The paper references the following figures (all included in `figures/`):

- `figs/fig1png.png` - SEA system overview
- `figs/fig2png.png` - DR-RMA architecture
- `fig1_attention_heatmap.png` - Attention weights heatmap
- `fig2_inertia_estimation.png` - Inertia estimation accuracy
- `baseline_comparison_test2_inertia_steps.png` - Five-algorithm comparison
- `attention_detail_combined.png` - Attention collapse details (needs creation)

## Notes

- Control frequency: 200 Hz
- Physics simulation: 1000 Hz
- Test scenario: 0.3→0.05→0.75 kg·m² inertia variations
- Algorithms compared: DR-RMA, PPO, RMA, PD-DOB, LQR
