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

ch3 = (ROOT / "docs/content/chapter-03.md").read_text(encoding="utf-8")
ch8 = (ROOT / "docs/content/chapter-08.md").read_text(encoding="utf-8")
example3d = ROOT / "examples/guidance_3d.c"
for token in (r"\boldsymbol\omega_{LOS}", r"\mathbf a_c", "三维"):
    if token not in ch3:
        errors.append(f"chapter-03.md: missing 3-D PN token {token!r}")
for token in (r"\mathbf{ZEM}_\perp", r"\mathbf a_{IAC}", r"\mathbf a_{TZC}"):
    if token not in ch8:
        errors.append(f"chapter-08.md: missing 3-D terminal-guidance token {token!r}")
if not example3d.exists() or "Vec3 pn,apn,zem,iac,tzc" not in example3d.read_text(encoding="utf-8"):
    errors.append("examples/guidance_3d.c: missing unified 3-D guidance outputs")
for name, token in {
    "full_guidance_3d.c": "HANDOFF",
    "estimator_3d.c": "AxisKF",
    "mpc_3d.c": "project",
}.items():
    path = ROOT / "examples" / name
    if not path.exists() or token not in path.read_text(encoding="utf-8"):
        errors.append(f"examples/{name}: missing required 3-D reference implementation")

angle_figure = ROOT / "docs/assets/guidance-angles-overview.svg"
if not angle_figure.exists():
    errors.append("docs/assets/guidance-angles-overview.svg: missing angle convention figure")
else:
    figure_text = angle_figure.read_text(encoding="utf-8")
    for token in ("三维弹目几何", "LOS 方位角", "航迹倾角", "终端方向误差"):
        if token not in figure_text:
            errors.append(f"guidance-angles-overview.svg: missing figure label {token!r}")

if errors:
    print("\n".join(errors))
    sys.exit(1)
print(f"PASS: {len(FILES)} chapters; delimiters, legacy tokens and canonical constants checked")
