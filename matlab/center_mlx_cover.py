"""Center the cover-page paragraphs of project_report.mlx.

openAndSave (.m -> .mlx) cannot carry text alignment, so the Live Script
cover renders left-ragged. This injects <w:jc w:val="center"/> into the
title-section paragraphs (title, course/institution lines, team heading,
roster - all five roster lines are the same width so per-line centering
keeps the columns; the BASE PAPERS references stay left-aligned, standard
for citations). Run AFTER every openAndSave regeneration:

    mmenv python center_mlx_cover.py
"""
import re
import shutil
import sys
import zipfile
from pathlib import Path

ROSTER = {
    "B Sainath            CB.SC.U4AIE24309",
    "Manohar Paturi       CB.SC.U4AIE24339",
    "K Pushpak            CB.SC.U4AIE24328",
    "Sai Krishna          CB.SC.U4AIE24308",
    "Vishal               CB.SC.U4AIE24363",
}


def center(path: Path) -> int:
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        parts = {n: z.read(n) for n in names}
    doc = parts["matlab/document.xml"].decode("utf-8")

    paras = list(re.finditer(r"<w:p>.*?</w:p>", doc, re.S))
    done = 0
    out = []
    last = 0
    seen_heading = False
    for m in paras:
        p = m.group(0)
        style = re.search(r'pStyle w:val="([^"]+)"', p)
        style = style.group(1) if style else ""
        if style == "heading":
            seen_heading = True
        text = "".join(re.findall(r"<w:t[^>]*>([^<]*)</w:t>", p))
        cover = (not seen_heading and (
            style in ("title", "text") or text in ROSTER))
        if cover and "<w:jc" not in p:
            p = p.replace("</w:pPr>", '<w:jc w:val="center"/></w:pPr>', 1)
            done += 1
        out.append(doc[last:m.start()])
        out.append(p)
        last = m.end()
    out.append(doc[last:])
    parts["matlab/document.xml"] = "".join(out).encode("utf-8")

    tmp = path.with_suffix(".tmp.mlx")
    with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
        for n in names:
            z.writestr(n, parts[n])
    shutil.move(tmp, path)
    return done


if __name__ == "__main__":
    n = center(Path(sys.argv[1] if len(sys.argv) > 1 else "project_report.mlx"))
    print(f"centered {n} cover paragraphs")
