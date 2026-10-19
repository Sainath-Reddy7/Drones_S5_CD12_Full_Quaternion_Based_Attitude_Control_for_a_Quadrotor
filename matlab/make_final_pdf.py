"""Build the final report PDF: a dedicated reportlab cover page (navy
Amrita logo banner -> title -> course -> team, Sainath first) prepended
to the MATLAB publish output (TOC + body with hidden code echoes).
Run AFTER publishing:  mmenv python make_final_pdf.py"""
from pathlib import Path

import matplotlib
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)
from pypdf import PdfReader, PdfWriter

FONTS = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
pdfmetrics.registerFont(TTFont("DVS", str(FONTS / "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DVSB", str(FONTS / "DejaVuSans-Bold.ttf")))
NAVY = colors.HexColor("#0b3d91")
INK = colors.HexColor("#1a1f2e")

here = Path(__file__).parent
COVER = here / "cover_page.pdf"
FINAL = here / "published" / "project_report_final.pdf"

doc = SimpleDocTemplate(
    str(COVER), pagesize=A4,
    topMargin=2.2 * cm, bottomMargin=2 * cm,
    leftMargin=2.4 * cm, rightMargin=2.4 * cm,
    title="Cover", author="Group 12")

title_s = ParagraphStyle("t", fontName="DVSB", fontSize=17, leading=23,
                         textColor=INK, alignment=1, spaceAfter=14)
sub_s = ParagraphStyle("s", fontName="DVS", fontSize=11.5, leading=16,
                       textColor=colors.HexColor("#41485c"), alignment=1,
                       spaceAfter=26)
head_s = ParagraphStyle("h", fontName="DVSB", fontSize=11, leading=15,
                        textColor=NAVY, spaceBefore=18, spaceAfter=8)
mem_s = ParagraphStyle("m", fontName="DVS", fontSize=11, leading=20,
                       textColor=INK, leftIndent=2.2 * cm)
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
    [["", Paragraph(n, name_s), Paragraph(r, roll_s)] for n, r in members],
    colWidths=[2.2 * cm, 7.4 * cm, 4.2 * cm], hAlign="LEFT")
team_tbl.setStyle(TableStyle([
    ("LEFTPADDING", (1, 0), (-1, -1), 0),
    ("RIGHTPADDING", (1, 0), (-1, -1), 0),
    ("TOPPADDING", (0, 0), (-1, -1), 3.5),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
    ("LINEBELOW", (1, 0), (-1, -2), 0.4, colors.HexColor("#c9cfdd")),
]))

story = [
    Spacer(1, 10),
    Image(str(here / "amrita_logo.png"), width=11.8 * cm, height=3.7 * cm),
    Spacer(1, 34),
    Paragraph("A Project Report on", sub_s),
    Paragraph("Full Quaternion Attitude Control and "
              "Differential-Flatness Trajectory Tracking for a Quadrotor",
              title_s),
    Spacer(1, 20),
    Paragraph("Bachelor of Technology - Artificial Intelligence Engineering",
              head_s),
    Paragraph("School of Artificial Intelligence<br/>"
              "Amrita School of Engineering, Bengaluru<br/>"
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

w = PdfWriter()
w.append(str(COVER))
w.append(str(here / "published" / "project_report.pdf"))
w.write(str(FINAL))
COVER.unlink()
print("final PDF:", FINAL, "-", len(PdfReader(str(FINAL)).pages), "pages")
