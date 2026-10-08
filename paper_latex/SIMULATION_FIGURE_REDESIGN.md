# Simulation Figure Redesign

## Evidence logic

The simulation section should answer four reviewer questions in order:

1. Does TRACE improve closed-loop control under abrupt inertia changes?
2. What does the timescale-resolved fast branch do at each transition?
3. Does the estimator remain useful outside the headline trajectory?
4. Which design choices are necessary, and what is the runtime cost?

Fixed-inertia tracking is a sanity check, not the main evidence for an adaptive controller. It should be reported briefly or moved to supplementary material.

## Main-text figures

### Fig. 3 - Main closed-loop benchmark

File: `figures/fig_sim_main_comparison.pdf`

Full-width `figure*`, four vertically aligned full-episode panels plus two event zooms:

- (a) reference and load position for TRACE, RMA, PPO, PD+DOB, and LQR;
- (b) tracking error;
- (c) motor torque;
- (d) true and TRACE-estimated load inertia.
- (e) tracking error during the 0.5 s payload-release window;
- (f) tracking error during the 0.5 s payload-attachment window.

Use $0.5\sin(2\pi\,0.3t)$ with fixed zero phase so the initial position error is zero. Mark the 3 s payload-release and 7 s payload-attachment events in every panel. This is the decisive performance figure.

### Fig. 4 - Adaptation outputs

File: `figures/fig_sim_adaptation_estimates.pdf`

Single-column figure with two aligned panels:

- (a) true inertia, short-head estimate, long-head estimate, and fused estimate;
- (b) learned jump gate and confidence.

This figure must use the exact TRACE rollout shown in Fig. 3. In particular, the fused estimate in Fig. 4(a) and Fig. 3(d) must be numerically identical, not merely generated with nominally similar settings.

### Fig. 5 - Event attention

File: `figures/fig_sim_attention_windows.pdf`

Single-column figure with release and attachment windows stacked vertically. Use a white-to-bright-red scale so low weights remain visually quiet. The figure is an empirical routing diagnostic, not a formal change detector or identifiability proof.

### Figs. 6 and 7 - Cross-condition generalization

Files: `figures/fig_sim_generalization_inertia.pdf` and `figures/fig_sim_generalization_tracking.pdf`

Two independent single-column heatmaps:

- stable-segment inertia RMSE across six scenario families and six Coulomb-friction levels;
- closed-loop tracking RMSE for the same grid.

The 108-condition suite contains six trajectory/inertia patterns, six load Coulomb-friction values, and three load viscous-damping values. Each cell averages over the three damping values. The former boxplot and scatter plot were removed because they repeated information already visible in the heatmaps.

## Main-text tables

### Main benchmark table

Report metrics that match the claim of rapid and smooth adaptation:

- overall tracking RMSE;
- 0.5 s post-release RMSE;
- 0.5 s post-attachment RMSE;
- RMS torque increment.

Do not lead with maximum error when it is dominated by startup initialization. Saturation can be stated in text when all methods have zero saturation.

The pre-/post-DAgger and confidence-hold comparison is reported in prose rather than as a separate table. It supports model selection but is not a complete architectural ablation.

### Runtime table

Report workstation and Jetson Orin results separately. The Orin requires at least two CPU threads to achieve zero 5 ms deadline misses in the measured run.

## Required strict ablation before submission

Use a table rather than another large figure. Every row must be evaluated under the same zero-phase benchmark and retrained where removal changes the policy input distribution:

- full TRACE;
- nominal fixed inertia, without the Fast Branch;
- Fast-only, without the Slow TCN Branch;
- short head only;
- long head only;
- fixed 0.5 fusion instead of the learned jump gate;
- pre-DAgger versus final DAgger model.

Recommended columns: overall RMSE, post-release RMSE, post-attachment RMSE, stable inertia RMSE, torque-increment RMS, and latency.

RMA and PPO remain algorithmic baselines. They should not be presented as strict component ablations because their policies and latent representations differ from TRACE.

## Supplementary figures

- fixed-inertia tracking at low, nominal, and high inertia;
- error and torque zooms around both inertia events;
- the worst three cases from the 108-condition suite;
- random-phase cold-start robustness;
- sensor noise, delay, and parameter-mismatch stress tests;
- per-seed distributions once repeated-seed evaluations are available.

## Reproduction

Regenerate the main and single-column figures from frozen NPZ/JSON artifacts with:

```powershell
& 'C:\Users\94490\.conda\envs\torch_env\python.exe' paper_latex\generate_simulation_figures.py
```
