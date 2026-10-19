"""Shared reportlab builder for the project's study/proposal PDFs.

Typography: DejaVu family (bundled with matplotlib) for full unicode math
glyphs (q̇, ω, ⊗, σ, ½, ‖, ψ ...), A4, numbered footer, section rules,
shaded equation blocks and insight boxes.
"""
from __future__ import annotations

import matplotlib
from pathlib import Path
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import cm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate, Frame, KeepTogether, NextPageTemplate, PageBreak,
    PageTemplate, Paragraph, Spacer, Table, TableStyle,
)

_FONTS = Path(matplotlib.get_data_path()) / "fonts" / "ttf"
pdfmetrics.registerFont(TTFont("DVS", str(_FONTS / "DejaVuSans.ttf")))
pdfmetrics.registerFont(TTFont("DVSB", str(_FONTS / "DejaVuSans-Bold.ttf")))
pdfmetrics.registerFont(TTFont("DVSO", str(_FONTS / "DejaVuSans-Oblique.ttf")))
pdfmetrics.registerFont(TTFont("DVSM", str(_FONTS / "DejaVuSansMono.ttf")))
pdfmetrics.registerFont(TTFont("DVSMB", str(_FONTS / "DejaVuSansMono-Bold.ttf")))

INK = colors.HexColor("#1a1f2e")
ACCENT = colors.HexColor("#0b3d91")
ACCENT2 = colors.HexColor("#8a4b08")
EQBG = colors.HexColor("#f2f4f8")
NOTEBG = colors.HexColor("#fdf6e3")
NOTEFRAME = colors.HexColor("#d8b64a")
RULE = colors.HexColor("#c6ccd8")

S = {
    "title": ParagraphStyle("title", fontName="DVSB", fontSize=21, leading=26,
                            textColor=ACCENT, spaceAfter=4),
    "subtitle": ParagraphStyle("subtitle", fontName="DVSO", fontSize=11.5,
                               leading=15, textColor=colors.HexColor("#41485c"),
                               spaceAfter=10),
    "h1": ParagraphStyle("h1", fontName="DVSB", fontSize=15, leading=19,
                         textColor=ACCENT, spaceBefore=16, spaceAfter=6,
                         keepWithNext=1),
    "h2": ParagraphStyle("h2", fontName="DVSB", fontSize=11.5, leading=15,
                         textColor=INK, spaceBefore=11, spaceAfter=4,
                         keepWithNext=1),
    "body": ParagraphStyle("body", fontName="DVS", fontSize=9.5, leading=13.8,
                           textColor=INK, spaceAfter=5),
    "eq": ParagraphStyle("eq", fontName="DVSM", fontSize=9.3, leading=13.5,
                         textColor=INK, alignment=1),
    "eqcap": ParagraphStyle("eqcap", fontName="DVSO", fontSize=8.3, leading=11,
                            textColor=colors.HexColor("#5a6275"), alignment=1,
                            spaceBefore=2, spaceAfter=6),
    "note": ParagraphStyle("note", fontName="DVS", fontSize=9.0, leading=13.2,
                           textColor=INK),
    "mono": ParagraphStyle("mono", fontName="DVSM", fontSize=8.6, leading=12.4,
                           textColor=INK),
    "small": ParagraphStyle("small", fontName="DVS", fontSize=8.2, leading=11.5,
                            textColor=colors.HexColor("#4a5164")),
    "tablehead": ParagraphStyle("th", fontName="DVSB", fontSize=8.6, leading=11.5,
                                textColor=colors.white),
    "tablecell": ParagraphStyle("tc", fontName="DVS", fontSize=8.6, leading=11.8,
                                textColor=INK),
}


def _footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(2 * cm, 1.55 * cm, A4[0] - 2 * cm, 1.55 * cm)
    canvas.setFont("DVS", 7.6)
    canvas.setFillColor(colors.HexColor("#6a7186"))
    canvas.drawString(2 * cm, 1.15 * cm, doc._doc_title)
    canvas.drawRightString(A4[0] - 2 * cm, 1.15 * cm, f"page {doc.page}")
    canvas.restoreState()


class Doc:
    """Fluent builder producing a PDF via platypus."""

    def __init__(self, path: str, title: str, subtitle: str):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.story = []
        self._doc_title = title
        self.title(title)
        self.story.append(Paragraph(subtitle, S["subtitle"]))
        self._doc = BaseDocTemplate(
            str(self.path), pagesize=A4,
            leftMargin=2 * cm, rightMargin=2 * cm,
            topMargin=1.8 * cm, bottomMargin=2 * cm,
            title=title, author="Group 12 - quadrotor-project")
        self._doc._doc_title = title
        self._doc.addPageTemplates([
            PageTemplate(id="main",
                         frames=[Frame(2 * cm, 1.9 * cm, A4[0] - 4 * cm,
                                       A4[1] - 3.7 * cm, id="f")],
                         onPage=_footer),
        ])

    def title(self, text):
        self.story.append(Paragraph(text, S["title"]))
        return self

    def h1(self, text):
        self.story.append(Paragraph(text, S["h1"]))
        return self

    def h2(self, text):
        self.story.append(Paragraph(text, S["h2"]))
        return self

    def p(self, text):
        self.story.append(Paragraph(text, S["body"]))
        return self

    def eq(self, lines, caption=None):
        """Equation block: mono, centered, shaded; caption underneath.
        Box and caption are atomic (KeepTogether) so a page break can
        never strand the caption from its equation."""
        if isinstance(lines, str):
            lines = [lines]
        rows = [[Paragraph(ln.replace(" ", "&nbsp;"), S["eq"])] for ln in lines]
        t = Table(rows, colWidths=[15.4 * cm])
        t.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, -1), EQBG),
            ("BOX", (0, 0), (-1, -1), 0.6, RULE),
            ("TOPPADDING", (0, 0), (-1, -1), 3),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
            ("LEFTPADDING", (0, 0), (-1, -1), 10),
            ("RIGHTPADDING", (0, 0), (-1, -1), 10),
        ]))
        parts = [Spacer(1, 3), t]
        if caption:
            parts.append(Paragraph(caption, S["eqcap"]))
        else:
            parts.append(Spacer(1, 5))
        self.story.append(KeepTogether(parts))
        return self

    def note(self, title, text):
        """Insight/remark box."""
        body = [
            [Paragraph(f"<b>{title}</b>", ParagraphStyle(
                "nt", parent=S["note"], fontName="DVSB",
                textColor=ACCENT2))],
            [Paragraph(text, S["note"])],
        ]
        t = Table(body, colWidths=[15.4 * cm])
        t.setStyle(TableStyle([
            ("BACKGROUND", (0, 0), (-1, -1), NOTEBG),
            ("BOX", (0, 0), (-1, -1), 0.9, NOTEFRAME),
            ("TOPPADDING", (0, 0), (-1, 0), 5),
            ("BOTTOMPADDING", (0, -1), (-1, -1), 6),
            ("LEFTPADDING", (0, 0), (-1, -1), 9),
            ("RIGHTPADDING", (0, 0), (-1, -1), 9),
        ]))
        self.story.append(Spacer(1, 4))
        self.story.append(t)
        self.story.append(Spacer(1, 6))
        return self

    def bullets(self, items, bold_lead=True):
        for it in items:
            if bold_lead and " - " in it:
                head, rest = it.split(" - ", 1)
                txt = f"<b>{head}</b> - {rest}"
            else:
                txt = it
            self.story.append(Paragraph(
                f"&bull;&nbsp;&nbsp;{txt}",
                ParagraphStyle("bl", parent=S["body"], leftIndent=11,
                               firstLineIndent=-11, spaceAfter=2.5)))
        self.story.append(Spacer(1, 2))
        return self

    def table(self, header, rows, widths=None, align_right=()):
        data = [[Paragraph(h, S["tablehead"]) for h in header]]
        for r in rows:
            data.append([Paragraph(str(c), S["tablecell"]) for c in r])
        if widths is None:
            widths = [15.4 * cm / len(header)] * len(header)
        t = Table(data, colWidths=widths, repeatRows=1)
        style = [
            ("BACKGROUND", (0, 0), (-1, 0), ACCENT),
            ("GRID", (0, 0), (-1, -1), 0.45, RULE),
            ("TOPPADDING", (0, 0), (-1, -1), 3),
            ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
            ("LEFTPADDING", (0, 0), (-1, -1), 5),
            ("RIGHTPADDING", (0, 0), (-1, -1), 5),
            ("ROWBACKGROUNDS", (0, 1), (-1, -1),
             [colors.white, colors.HexColor("#f5f7fa")]),
        ]
        for c in align_right:
            style.append(("ALIGN", (c, 1), (c, -1), "RIGHT"))
        t.setStyle(TableStyle(style))
        self.story.append(Spacer(1, 3))
        self.story.append(t)
        self.story.append(Spacer(1, 6))
        return self

    def mono(self, lines):
        if isinstance(lines, str):
            lines = [lines]
        for ln in lines:
            self.story.append(Paragraph(ln.replace(" ", "&nbsp;"), S["mono"]))
        self.story.append(Spacer(1, 4))
        return self

    def pagebreak(self):
        self.story.append(PageBreak())
        return self

    def spacer(self, h=6):
        self.story.append(Spacer(1, h))
        return self

    def save(self):
        # Bind section headings to the following atomic (KeepTogether) unit:
        # reportlab's keepWithNext does not chain into KeepTogether successors,
        # so an orphaned heading must be prevented by merging it into the unit.
        merged = []
        i = 0
        story = self.story
        while i < len(story):
            item = story[i]
            if (isinstance(item, Paragraph) and item.style.name in ("h1", "h2")
                    and i + 1 < len(story) and isinstance(story[i + 1], KeepTogether)):
                merged.append(KeepTogether([item] + list(story[i + 1]._content)))
                i += 2
                continue
            merged.append(item)
            i += 1
        self._doc.build(merged)
        return str(self.path)
