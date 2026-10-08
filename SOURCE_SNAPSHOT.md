# Source Snapshot

- Collection date: 2026-10-08 (Asia/Shanghai)
- Source controller: NVIDIA Orin
- Source workspace: `~/workspace/SEA_BOX/ETHERCAT_SEABOX`
- Source branch: `main`
- Source base commit: `92a001380bb75daf1119bdf9209b2a2d06cbfab6`
- Source worktree state: dirty, with 118 tracked/untracked entries

The hardware-test source on the Orin contained experiment-specific uncommitted
changes. This publication intentionally captures the current files used by the
joint tests rather than claiming they are identical to the source base commit.

Excluded from the publication:

- raw CSV/log/status files
- historical `.bak` and `.pre_*` copies
- Python caches
- whole-arm demonstration code
- training datasets and checkpoints not required at runtime
- all model weights and ONNX external-data files (distributed separately)
- temporary manuscript build products and preview caches

Publication-specific changes:

- the local `paper_latex` workspace was added on 2026-10-08 with manuscript sources,
  bibliography, exported figures, editable artwork, PDF snapshots, and plotting scripts

- nonzero torque output is disabled by default in the main experiment entries
- automatic relay operation is disabled by default in fixed-inertia scripts
- DRRMA and TRACE package paths are repository-relative
- repository documentation, dependency files, validation, and offline tests were
  added
