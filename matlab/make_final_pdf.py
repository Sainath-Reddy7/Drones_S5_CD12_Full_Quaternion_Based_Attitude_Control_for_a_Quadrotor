"""Build the final report PDF.

Pipeline: MATLAB publishes project_report.m to HTML (images render
inline under their own headings; the PDF pipeline floats and strands
them), this script post-processes the HTML (fit images to page, keep
each catalogue figure with its heading, single-line centred title,
hidden flush-tag output), prints it with headless Edge, injects TOC
page numbers (second pass), stamps footer page numbers, and prepends
the reportlab cover page (navy Amrita banner -> title -> course ->
team, Sainath first).

Run AFTER publishing (format must be 'html'):
    matlab: publish('project_report.m', struct('format','html', ...
                'outputDir', <published>, 'showCode', false))
    mmenv python make_final_pdf.py
"""
import io
import re
import subprocess
from pathlib import Path

import matplotlib
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas as rl_canvas
from reportlab.platypus import (Image, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)
from pypdf import PdfReader, PdfWriter

FONTS = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
pdfmetrics.registerFont(TTFont("DVS", str(FONTS / "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DVSB", str(FONTS / "DejaVuSans-Bold.ttf")))
NAVY = colors.HexColor("#0b3d91")
INK = colors.HexColor("#1a1f2e")

EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
here = Path(__file__).parent
PUB = here / "published"
COVER = PUB / "cover_page.pdf"
BODY = PUB / "project_report_body.pdf"
FINAL = PUB / "project_report_final.pdf"

TITLE = ("Full Quaternion Attitude Control and Differential-Flatness "
         "Trajectory Tracking for a Quadrotor")

# ---------------------------------------------------------------- cover --
doc = SimpleDocTemplate(
    str(COVER), pagesize=A4,
    topMargin=2.2 * cm, bottomMargin=2 * cm,
    leftMargin=1.7 * cm, rightMargin=1.7 * cm,
    title="Cover", author="Group 12")

avail = A4[0] - doc.leftMargin - doc.rightMargin - 14   # frame padding
tfs = 16.0
while pdfmetrics.stringWidth(TITLE, "DVSB", tfs) > avail:
    tfs -= 0.1

title_s = ParagraphStyle("t", fontName="DVSB", fontSize=tfs,
                         leading=tfs + 7, textColor=INK, alignment=1,
                         spaceAfter=14)
sub_s = ParagraphStyle("s", fontName="DVS", fontSize=11.5, leading=16,
                       textColor=colors.HexColor("#41485c"), alignment=1,
                       spaceAfter=26)
head_s = ParagraphStyle("h", fontName="DVSB", fontSize=11, leading=15,
                        textColor=NAVY, alignment=1, spaceBefore=18,
                        spaceAfter=8)
mem_s = ParagraphStyle("m", fontName="DVS", fontSize=11, leading=20,
                       textColor=INK, alignment=1)
name_s = ParagraphStyle("n", fontName="DVS", fontSize=11, leading=14,
                        textColor=INK)
roll_s = ParagraphStyle("r", fontName="DVS", fontSize=11, leading=14,
                        textColor=INK, alignment=2)

members = [
    ("B Sainath", "CB.SC.U4AIE24309"),
    ("Manohar Paturi", "CB.SC.U4AIE24339"),
    ("K Pushpak", "CB.SC.U4AIE24328"),
    ("Sai Krishna", "CB.SC.U4AIE24308"),
    ("Vishal", "CB.SC.U4AIE24363"),
]
team_tbl = Table(
    [[Paragraph(n, name_s), Paragraph(r, roll_s)] for n, r in members],
    colWidths=[8.6 * cm, 5 * cm], hAlign="CENTER")
team_tbl.setStyle(TableStyle([
    ("LEFTPADDING", (0, 0), (-1, -1), 0),
    ("RIGHTPADDING", (0, 0), (-1, -1), 0),
    ("TOPPADDING", (0, 0), (-1, -1), 3.5),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
    ("LINEBELOW", (0, 0), (-1, -2), 0.4, colors.HexColor("#c9cfdd")),
]))

story = [
    Spacer(1, 10),
    Image(str(here / "amrita_logo.png"), width=11.8 * cm, height=3.7 * cm),
    Spacer(1, 34),
    Paragraph("A Project Report on", sub_s),
    Paragraph(TITLE, title_s),
    Spacer(1, 20),
    Paragraph("Bachelor of Technology - Artificial Intelligence Engineering",
              head_s),
    Paragraph("School of Artificial Intelligence<br/>"
              "Amrita School of Engineering, Coimbatore<br/>"
              "Amrita Vishwa Vidyapeetham", mem_s),
    Paragraph("Project Group 12 - Team Members", head_s),
    team_tbl,
    Paragraph("Base Papers", head_s),
    Paragraph("[1] Fresk &amp; Nikolakopoulos, \"Full Quaternion Based "
              "Attitude Control for a Quadrotor\", ECC 2013.<br/>"
              "[2] Choutri, Lagha, Dala &amp; Lipatov, \"Quadrotors Trajectory "
              "Tracking using a Differential Flatness-Quaternion based "
              "Approach\", IEEE 2017.", mem_s),
]
doc.build(story)

# ------------------------------------------------------- HTML -> body PDF --
CSS = """
<style type="text/css">
@page { size: 612pt 792pt; margin: 64pt 58pt 58pt 58pt; }
img { max-width: 100% !important; height: auto; }
h1 { white-space: nowrap; font-size: 13pt; text-align: center;
     color: #1a1f2e; }
h2 { break-after: avoid; page-break-after: avoid; }
.figblock { break-inside: avoid; page-break-inside: avoid; }
li:has(> .pg) { display: flex; align-items: baseline; }
li:has(> .pg) > a { flex: 0 1 auto; }
li > .dots { flex: 1 0 auto; min-width: 8pt;
             border-bottom: 1px dotted #999; margin: 0 3pt 3.5pt 3pt; }
li > .pg { flex: 0 0 auto; color: #444; }
</style>
"""


def build_print_html(html: str, pagemap=None) -> str:
    # 1. drop the flush-tag output blocks (publish mechanics, not content)
    html = re.sub(r'<pre class="codeoutput">Figure [MGE]\d+ embedded\.\s*'
                  r'</pre>\s*', "", html)
    # 2. keep each catalogue figure heading + caption + image together
    html = re.sub(
        r'(<h2 id="\d+">(?:15\.\d+ Figure [^\n]*?)</h2>\s*'
        r'(<p>[^<]*?</p>\s*)?<img[^>]*>)',
        r'<div class="figblock">\1</div>', html)
    # 3. TOC: link + optional dot leader + page number
    if pagemap:
        def repl(m):
            anchor, text = m.group(1), m.group(2)
            pg = pagemap.get(text)
            if pg is None:
                return m.group(0)
            return (f'<a href="#{anchor}">{text}</a>'
                    f'<span class="dots"></span><span class="pg">{pg}</span>')
        html = re.sub(r'<a href="#(\d+)">([^<]+)</a>', repl, html)
    # 4. style overrides at the end of head so they win the cascade
    html = html.replace("</head>", CSS + "</head>", 1)
    return html


def edge_print(src: Path, dst: Path):
    subprocess.run(
        [EDGE, "--headless", "--disable-gpu", "--no-pdf-header-footer",
         f"--print-to-pdf={dst}", src.as_uri()], check=True,
        capture_output=True, timeout=240)


def heading_pagemap(pdf: Path) -> dict:
    """Map section-heading text -> 1-based page in the printed body PDF."""
    heads = {}
    r = PdfReader(str(pdf))
    for i, page in enumerate(r.pages):
        t = page.extract_text() or ""
        for line in t.splitlines():
            line = line.strip()
            if re.match(r"^(\d+\.\s|Appendix [AB]\s-|15\.\d+\s)", line) \
                    and 4 < len(line) < 90 and line not in heads:
                heads[line] = i + 1
    return heads


html = (PUB / "project_report.html").read_text(encoding="utf-8", errors="replace")

# pass 1 (no page numbers yet)
p1 = PUB / "print_pass1.html"
p1.write_text(build_print_html(html), encoding="utf-8")
edge_print(p1, BODY)
pagemap = heading_pagemap(BODY)

# pass 2 (with TOC page numbers)
p2 = PUB / "print_pass2.html"
p2.write_text(build_print_html(html, pagemap), encoding="utf-8")
edge_print(p2, BODY)

# verify pass-2 pagination did not shift; if it did, redo with new map
if heading_pagemap(BODY) != pagemap:
    pagemap = heading_pagemap(BODY)
    p2.write_text(build_print_html(html, pagemap), encoding="utf-8")
    edge_print(p2, BODY)

# ------------------------------------------------- footer page numbers --
body = PdfReader(str(BODY))
w = PdfWriter()
for i, page in enumerate(body.pages):
    pw, ph = float(page.mediabox.width), float(page.mediabox.height)
    buf = io.BytesIO()
    c = rl_canvas.Canvas(buf, pagesize=(pw, ph))
    c.setFont("DVS", 8.5)
    c.setFillColor(colors.HexColor("#41485c"))
    c.drawCentredString(pw / 2, 24, str(i + 1))
    c.showPage()
    c.save()
    page.merge_page(PdfReader(io.BytesIO(buf.getvalue())).pages[0])
    w.add_page(page)

# ----------------------------------------------------------- assemble --
final = PdfWriter()
final.append(str(COVER))
final.append(w)
final.write(str(FINAL))
COVER.unlink()
for tmp in (p1, p2, BODY):
    tmp.unlink(missing_ok=True)
(PUB / "project_report.pdf").unlink(missing_ok=True)  # superseded PDF build
print("final PDF:", FINAL, "-", len(PdfReader(str(FINAL)).pages), "pages")
print("body pages:", len(body.pages), "| TOC entries mapped:", len(pagemap))
