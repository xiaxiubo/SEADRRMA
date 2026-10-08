"""Build a manuscript from its own directory with latexmk."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy", action="store_true", help="Build the previous DR-RMA draft.")
    args = parser.parse_args()
    executable = shutil.which("latexmk")
    if executable is None:
        parser.error("latexmk is not on PATH; install TeX Live or MiKTeX first")
    directory = Path(__file__).resolve().parents[1] / "paper_latex"
    manuscript = "paper_dr_rma.tex" if args.legacy else "paper_TRACE_title_intro.tex"
    result = subprocess.run(
        [executable, "-pdf", "-interaction=nonstopmode", "-halt-on-error", manuscript],
        cwd=directory,
        check=False,
    )
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
