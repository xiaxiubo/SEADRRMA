"""Check manuscript figure, input, and bibliography dependencies without compiling."""

from __future__ import annotations

import ast
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PAPER = ROOT / "paper_latex"
ENTRIES = ("paper_TRACE_title_intro.tex", "paper_dr_rma.tex")


def source(path: Path) -> str:
    return re.sub(r"(?<!\\)%.*", "", path.read_text(encoding="utf-8"))


def main() -> int:
    visited: set[Path] = set()
    figures: set[str] = set()
    missing: list[str] = []
    citations: set[str] = set()
    pending = [PAPER / name for name in ENTRIES]
    while pending:
        path = pending.pop()
        if path in visited:
            continue
        if not path.is_file():
            missing.append(str(path.relative_to(ROOT)))
            continue
        visited.add(path)
        content = source(path)
        for name in re.findall(r"\\(?:input|include)\{([^}]+)\}", content):
            pending.append(PAPER / (name if Path(name).suffix else name + ".tex"))
        for name in re.findall(r"\\includegraphics\s*(?:\[[^\]]*\])?\s*\{([^}]+)\}", content):
            extensions = ("",) if Path(name).suffix else (".pdf", ".png", ".jpg", ".jpeg")
            candidates = [base / (name + ext) for base in (PAPER, PAPER / "figures")
                          for ext in extensions]
            figures.add(name)
            if not any(candidate.is_file() for candidate in candidates):
                missing.append("figure: " + name)
        for group in re.findall(r"\\cite\w*\s*(?:\[[^\]]*\])?\{([^}]+)\}", content):
            citations.update(key.strip() for key in group.split(","))
        for group in re.findall(r"\\bibliography\{([^}]+)\}", content):
            for name in group.split(","):
                if not (PAPER / (name.strip() + ".bib")).is_file():
                    missing.append("bibliography: " + name)
    bibliography = (PAPER / "references.bib").read_text(encoding="utf-8")
    keys = set(re.findall(r"@\w+\s*\{\s*([^,\s]+)\s*,", bibliography))
    missing.extend("citation: " + key for key in sorted(citations - keys))
    for path in [*PAPER.glob("*.py"), ROOT / "plot_hardware_four_algorithms.py",
                 ROOT / "attention_fast_v2.py"]:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print(json.dumps({"entries": ENTRIES, "tex_sources": len(visited),
                      "figures": len(figures), "citation_keys": len(citations),
                      "missing_dependencies": missing}, indent=2))
    return 1 if missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
