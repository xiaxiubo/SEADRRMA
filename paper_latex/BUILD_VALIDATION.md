# Manuscript Build Validation

Validated on 2026-10-08 using TeX Live 2026 from the published source checkout.

| Entry | Result | Pages |
| --- | --- | --- |
| `paper_TRACE_title_intro.tex` | PDF compilation succeeded | 12 |
| `paper_dr_rma.tex` | PDF compilation succeeded | 14 |

`python tools/validate_paper.py` checked both entries, four TeX sources,
11 referenced figures and 31 citation keys. No missing dependencies were found.

The existing bibliography produces 22 BibTeX warnings about empty traditional
`journal` or `year` fields. Some entries use `journaltitle` or `date` instead.
These warnings do not prevent compilation; bibliography formatting still needs
review before submission. The legacy draft also has font substitutions and
underfull-box warnings. Source text, numerical results and bibliography entries
were preserved during packaging.

The published PDFs are draft snapshots, not a claim of journal acceptance or
hardware validation. Compiling the manuscripts uses the included figures;
regenerating experimental plots requires the private input data described in
`README.md`.
