#!/usr/bin/env python3
"""Render docs/OPEN_BUGS.md to docs/OPEN_BUGS.pdf (landscape).

The register used to be a hand-maintained PDF with no source, so it drifted from
the code it described. The markdown is now the single source; this regenerates
the PDF from it. Handles only the subset of markdown the register uses:
headings, pipe tables, paragraphs, bullets, blockquotes and horizontal rules.

Run:  python3 tools/render_open_bugs.py [source.md] [out.pdf]
"""
from __future__ import annotations

import html
import os
import re
import sys

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (
    HRFlowable, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle,
)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "docs", "OPEN_BUGS.md")
OUT = os.path.join(REPO, "docs", "OPEN_BUGS.pdf")

# Status colouring: the register is read at a glance, so fixed/open must differ
# without reading the text.
GREEN = colors.HexColor("#1b7f3b")
RED = colors.HexColor("#a41f1f")
AMBER = colors.HexColor("#8a6100")
GREY = colors.HexColor("#f2f2f2")
HEAD_BG = colors.HexColor("#2b2b2b")


def _inline(text: str) -> str:
    """Markdown inline → reportlab mini-HTML. Escape first, then re-add tags."""
    t = html.escape(text)
    t = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", t)
    t = re.sub(r"(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)", r"<i>\1</i>", t)
    t = re.sub(r"~~(.+?)~~", r"<strike>\1</strike>", t)
    t = re.sub(r"`(.+?)`", r'<font face="Courier" size="6.5">\1</font>', t)
    return t


def _split_row(line: str) -> list[str]:
    return [c.strip() for c in line.strip().strip("|").split("|")]


def build(src: str = SRC, out: str = OUT) -> str:
    lines = open(src, encoding="utf-8").read().splitlines()

    ss = getSampleStyleSheet()
    body = ParagraphStyle("body", parent=ss["BodyText"], fontSize=7.4, leading=9.2,
                          alignment=TA_LEFT, spaceAfter=3)
    cell = ParagraphStyle("cell", parent=body, fontSize=6.6, leading=8.0, spaceAfter=0)
    cell_b = ParagraphStyle("cellb", parent=cell, textColor=colors.white,
                            fontName="Helvetica-Bold")
    h1 = ParagraphStyle("h1", parent=ss["Title"], fontSize=15, leading=18, spaceAfter=4)
    h2 = ParagraphStyle("h2", parent=ss["Heading2"], fontSize=10.5, leading=12,
                        spaceBefore=9, spaceAfter=4,
                        textColor=colors.HexColor("#1a1a1a"))
    h3 = ParagraphStyle("h3", parent=ss["Heading3"], fontSize=8.6, leading=10.5,
                        spaceBefore=6, spaceAfter=3)
    quote = ParagraphStyle("quote", parent=body, leftIndent=8, fontSize=7.0,
                           leading=8.8, textColor=colors.HexColor("#444444"),
                           borderPadding=2)
    bullet = ParagraphStyle("bullet", parent=body, leftIndent=10, bulletIndent=2)

    story: list = []
    i = 0
    while i < len(lines):
        ln = lines[i]
        s = ln.strip()

        if not s:
            i += 1
            continue

        if s.startswith("|") and i + 1 < len(lines) and re.match(
                r"^\|[\s:|-]+\|$", lines[i + 1].strip()):
            header = _split_row(s)
            i += 2
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                rows.append(_split_row(lines[i]))
                i += 1

            ncol = len(header)
            # The middle columns carry the prose; give them the space.
            if ncol == 6:      # A / B tables: #, Bug, Location, Evidence, Sev, Status
                widths = [16, 78, 52, 78, 16, 52]
            elif ncol == 5:    # C table: #, Item, Evidence, Sev, Status
                widths = [18, 60, 110, 18, 60]
            else:
                widths = [266 / ncol] * ncol
            widths = [w * mm * (266 / sum(widths)) / mm for w in widths]
            widths = [w / sum(widths) * 266 * mm for w in widths]

            data = [[Paragraph(_inline(c), cell_b) for c in header]]
            for r in rows:
                r = (r + [""] * ncol)[:ncol]
                data.append([Paragraph(_inline(c), cell) for c in r])

            t = Table(data, colWidths=widths, repeatRows=1)
            style = [
                ("BACKGROUND", (0, 0), (-1, 0), HEAD_BG),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("GRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#bbbbbb")),
                ("LEFTPADDING", (0, 0), (-1, -1), 3),
                ("RIGHTPADDING", (0, 0), (-1, -1), 3),
                ("TOPPADDING", (0, 0), (-1, -1), 2.5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5),
            ]
            for ri, r in enumerate(rows, start=1):
                joined = " ".join(r)
                if ri % 2 == 0:
                    style.append(("BACKGROUND", (0, ri), (-1, ri), GREY))
                if "✅" in joined:
                    style.append(("TEXTCOLOR", (0, ri), (0, ri), GREEN))
                elif "☐" in joined and ("HIGH" in joined):
                    style.append(("TEXTCOLOR", (0, ri), (0, ri), RED))
                elif "☐" in joined:
                    style.append(("TEXTCOLOR", (0, ri), (0, ri), AMBER))
            t.setStyle(TableStyle(style))
            story.append(t)
            story.append(Spacer(1, 5))
            continue

        if s.startswith("# "):
            story.append(Paragraph(_inline(s[2:]), h1))
        elif s.startswith("## "):
            story.append(Paragraph(_inline(s[3:]), h2))
        elif s.startswith("### "):
            story.append(Paragraph(_inline(s[4:]), h3))
        elif s.startswith("---"):
            story.append(Spacer(1, 3))
            story.append(HRFlowable(width="100%", thickness=0.5,
                                    color=colors.HexColor("#999999")))
            story.append(Spacer(1, 3))
        elif s.startswith(">"):
            story.append(Paragraph(_inline(s.lstrip("> ").rstrip()), quote))
        elif re.match(r"^[-*] ", s):
            story.append(Paragraph(_inline(s[2:]), bullet, bulletText="•"))
        elif re.match(r"^\d+\. ", s):
            n, rest = s.split(". ", 1)
            story.append(Paragraph(_inline(rest), bullet, bulletText=f"{n}."))
        else:
            story.append(Paragraph(_inline(s), body))
        i += 1

    doc = SimpleDocTemplate(
        out, pagesize=landscape(A4),
        leftMargin=12 * mm, rightMargin=12 * mm,
        topMargin=10 * mm, bottomMargin=10 * mm,
        title="PX4_DXP — Open Bug Register", author="PX4_DXP",
    )

    def _footer(canvas, _doc):
        canvas.saveState()
        canvas.setFont("Helvetica", 6.5)
        canvas.setFillColor(colors.HexColor("#777777"))
        canvas.drawString(12 * mm, 5 * mm,
                          "Generated from docs/OPEN_BUGS.md by tools/render_open_bugs.py "
                          "— edit the markdown, not this PDF.")
        canvas.drawRightString(landscape(A4)[0] - 12 * mm, 5 * mm, f"page {_doc.page}")
        canvas.restoreState()

    doc.build(story, onFirstPage=_footer, onLaterPages=_footer)
    return out


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else SRC
    out = sys.argv[2] if len(sys.argv) > 2 else OUT
    print(f"wrote {build(src, out)}")
