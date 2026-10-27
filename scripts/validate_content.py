#!/usr/bin/env python3
"""Fast source checks; browser rendering still requires the manual checklist."""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
FILES = sorted((ROOT / "docs" / "content").glob("chapter-*.md"))
errors = []

for path in FILES:
    text = path.read_text(encoding="utf-8")
    if text.count("```") % 2:
        errors.append(f"{path}: unpaired code fence")
    if text.count(r"\[") != text.count(r"\]"):
        errors.append(f"{path}: unpaired display-math delimiter")
    for bad in (",qquad", "交通避让", "冲突解除", "回航逻辑"):
        if bad in text:
            errors.append(f"{path}: legacy or malformed token {bad!r}")

ch2 = (ROOT / "docs/content/chapter-02.md").read_text(encoding="utf-8")
if "C^A_B" in ch2:
    errors.append("chapter-02.md: legacy coordinate-transform notation")

ch9 = (ROOT / "docs/content/chapter-09.md").read_text(encoding="utf-8")
if "#define A_MAX_MPS2 294.1995" not in ch9:
    errors.append("chapter-09.md: code constant no longer matches 30g table")

if errors:
    print("\n".join(errors))
    sys.exit(1)
print(f"PASS: {len(FILES)} chapters; delimiters, legacy tokens and canonical constants checked")
