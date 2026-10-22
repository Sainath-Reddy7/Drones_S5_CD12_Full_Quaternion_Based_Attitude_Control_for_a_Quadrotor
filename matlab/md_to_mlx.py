"""Convert quadrotor_math_derivations_github.md to a MATLAB Live Script.

Markdown -> publish-markup .m -> openAndSave -> .mlx.

Structural mapping:
  #/## headings        -> %% section headings (math $..$ stripped in headings)
  paragraphs/bullets   -> % text / % * bullets
  fenced ascii blocks  -> indented monospace text (%   ...)
  tables               -> indented monospace blocks
  $$..$$ display math  -> single-line $$..$$ (joined)
  inline $..$          -> kept

TeX is normalised to the subset proven in project_report.m:
  \\tfrac/\\dfrac -> \\frac, \\text -> \\mathrm, \\operatorname -> \\mathrm,
  \\mathcal X / \\mathbb{R} -> plain letters, \\lVert/\\rVert -> \\Vert,
  \\boxed/\\big/\\!/\\; stripped, braceless \\frac args braced,
  unicode math -> TeX.

Usage:  mmenv python md_to_mlx.py <in.md> <out.m>
"""
import re
import sys
from pathlib import Path

REPL = [
    (r"\\tfrac", r"\\frac"), (r"\\dfrac", r"\\frac"),
    (r"\\text\b", r"\\mathrm"), (r"\\operatorname", r"\\mathrm"),
    (r"\\mathcal X", "{X}"), (r"\\mathbb\{R\}", "{R}"), (r"\\mathbb\{H\}", "{H}"),
    (r"\\lVert", "||"), (r"\\rVert", "||"),
    (r"\\boxed\{([^{}]*)\}", r"\1"),
    (r"\\bigg?\b", ""), (r"\\Big\b", ""), (r"\\!", ""), (r"\\;", "\\;"),
    (r"\\succeq", r"\\geq"), (r"\\succ", ">"),
    (r"×", r"\\times"), (r"≈", r"\\approx"), (r"±", r"\\pm"),
    (r"−", "-"), (r"°", r"^\\circ"), (r"≥", r"\\geq"), (r"µ", r"\\mu"),
    (r"—", "-"), (r"–", "-"), (r"→", "->"), (r"\\dot\{X\}", r"\\dot{X}"),
    # MATLAB's equation importer supports no \begin{} environments and no
    # \\ row breaks (its TeX table = the classic text-TeX subset), so:
    (r"\\qquad", r"\\;\\;\\;\\;"), (r"\\quad", r"\\;\\;"),
    (r"\\ ", r"\\;"), (r"\\,", r"\\;"),
    (r"\\left", ""), (r"\\right", ""),
    (r"\\to(?![a-zA-Z])", r"\\rightarrow"), (r"\\ge(?![a-zA-Z])", r"\\geq"),
    (r"\\Rightarrow", "=>"), (r"\\Leftrightarrow", "<=>"),
    (r"\\arcsin", r"\\mathrm{arcsin}"), (r"\\arctan", r"\\mathrm{arctan}"),
    (r"\\arccos", r"\\mathrm{arccos}"), (r"\\tan(?![a-zA-Z])", r"\\mathrm{tan}"),
    (r"\\lim(?![a-zA-Z])", r"\\mathrm{lim}"), (r"\\det(?![a-zA-Z])", r"\\mathrm{det}"),
    (r"\\mathbf 0", r"\\mathbf{0}"),
    (r"\\mathcal\{X\}", "{X}"),
]

MATRIX_RE = re.compile(r"\\begin\{bmatrix\}(.*?)\\end\{bmatrix\}")
ALIGNED_RE = re.compile(r"\\begin\{aligned\}(.*?)\\end\{aligned\}")


def tex_normalize(s: str) -> str:
    # matrices -> bracket notation first (this removes their & and \\ too)
    def mat(m):
        inner = m.group(1).replace("&", ", ").replace("\\\\", "; ")
        inner = re.sub(r"\[\d+pt\]", "", inner)  # \\[2pt] row-space hacks
        return "[" + inner.strip() + "]"
    s = MATRIX_RE.sub(mat, s)
    for a, b in REPL:
        s = s.replace(a, b) if not a.startswith("\\") and len(a) > 2 else \
            re.sub(a, b, s)
    # braceless \frac args: \frac x2 -> \frac{x}{2}
    s = re.sub(r"\\frac (\w) (\w)", r"\\frac{\1}{\2}", s)
    s = re.sub(r"\\frac (\w) 2", r"\\frac{\1}{2}", s)
    s = re.sub(r"\\frac\{([^{}]*)\} (\w)", r"\\frac{\1}{\2}", s)
    s = re.sub(r"\\frac (\w)\{", r"\\frac{\1}{", s)
    # \frac x2 handled; also \frac \alpha2 style
    s = re.sub(r"\\frac (\\[a-zA-Z]+) 2", r"\\frac{\1}{2}", s)
    s = re.sub(r"\\bar ([a-zA-Z])", r"\\bar{\1}", s)
    s = re.sub(r"\\bar\{q\}", r"\\bar{q}", s)
    # nested braces produced by the \mathcal X -> {X} rule: \dot{{X}} -> \dot{X}
    s = re.sub(r"\{(\{[A-Za-z]+\})\}", r"\1", s)
    return s


def strip_inline_math(s: str) -> str:
    return re.sub(r"\$([^$]*)\$", r"\1", s)


def normalize_line(s: str) -> str:
    """Strip markdown bold/italic emphasis FIRST (spans may wrap math and
    break MATLAB's inline-equation pairing), then TeX-normalise ONLY
    inside $...$ spans; prose unicode (dashes, arrows, x) untouched."""
    s = re.sub(r"\*\*(.+?)\*\*", r"\1", s)          # bold (may wrap math)
    s = re.sub(r"(?<!\*)\*([A-Za-z(][^*]*?)\*(?!\*)", r"\1", s)  # italics
    # a double-quote hugging $ breaks MATLAB's inline-equation pairing
    s = s.replace('"$', '" $').replace('$"', '$ "')
    # punctuation-only math spans (+/-) do not convert: emit as plain text
    s = re.sub(r"\$([+=<>-]+)\$", r"\1", s)
    out = []
    for seg in re.split(r"(\$[^$]+\$)", s):
        if seg.startswith("$") and seg.endswith("$") and len(seg) > 2:
            out.append("$" + tex_normalize(seg[1:-1]) + "$")
        else:
            out.append(seg)
    return "".join(out)


def split_aligned(eq: str) -> list:
    """Split one RAW (pre-tex_normalize) display TeX string into rows.
    aligned environments become separate single-line equations (MATLAB has
    no multi-line equations); every alignment '&' is dropped; a trailing
    \\tag{N} or \\qquad\\text{(N)} after \\end{aligned} is kept on the last
    row. tex_normalize must run AFTER this: its '\\ ' rule corrupts the
    '\\\\' row separators and its group capture used to drop the tag."""
    m = ALIGNED_RE.search(eq)
    if not m:
        return [eq]
    tail = eq[m.end():]
    tm = (re.search(r"\\tag\{([^{}]*)\}", tail) or
          re.search(r"\\(?:qquad|quad|;)\\text\{([^{}]*)\}", tail))
    tag = tm.group(1) if tm else None
    if tm:
        tail = (tail[:tm.start()] + tail[tm.end():]).strip()
    rows = [re.sub(r"\s*&\s*", " ", r).strip()
            for r in re.split(r"\\\\\s*", m.group(1))]
    rows = [re.sub(r"\[\d+pt\]", "", r).strip() for r in rows]  # \\[2pt] hacks
    rows = [r for r in rows if r]
    if rows and tag:
        rows[-1] = rows[-1] + " \\qquad\\text{" + tag + "}"
    if rows and tail:  # any residual content after \end{aligned}
        rows[-1] = rows[-1] + " " + tail
    return rows


def convert(md: str) -> str:
    out = []
    in_fence = False
    fence_buf = []
    in_disp = False
    disp_buf = []
    i = 0
    lines = md.split("\n")
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("```"):
            if in_fence:
                out.append("%")
                for f in fence_buf:
                    out.append("%   " + f)
                out.append("%")
                fence_buf = []
            in_fence = not in_fence
            i += 1
            continue
        if in_fence:
            fence_buf.append(ln)
            i += 1
            continue
        if ln.strip() == "$$":
            if in_disp:
                eq = " ".join(x.strip() for x in disp_buf if x.strip())
                if eq:
                    for row in split_aligned(eq):
                        out.append("% $$" + tex_normalize(row) + "$$")
                        out.append("%")
                disp_buf = []
            in_disp = not in_disp
            i += 1
            continue
        if in_disp:
            disp_buf.append(ln)
            i += 1
            continue
        s = ln.rstrip()
        if not s.strip():
            out.append("%")
        elif s.startswith("|"):
            # markdown table row -> monospace block member
            if re.match(r"^\|[\s:|-]+\|$", s):
                i += 1
                continue  # separator row
            if not (out and out[-1].startswith("%   ") or
                    (out and out[-1] == "%" and
                     any(o.startswith("%   ") for o in out[-3:]))):
                out.append("%")
            cells = [c.strip() for c in s.strip("|").split("|")]
            out.append("%   " + "  ".join(
                re.sub(r"\*\*(.+?)\*\*", r"\1",
                       strip_inline_math(tex_normalize(c)) if "$" in c else c)
                for c in cells))
        elif s.startswith("# ") or s.startswith("## "):
            head = s.lstrip("#").strip()
            out.append("")
            out.append("%% " + strip_inline_math(head))
        elif s.startswith("### "):
            head = s.lstrip("#").strip()
            out.append("%% " + strip_inline_math(head))
        elif s.lstrip().startswith("- "):
            item = s.lstrip()[2:]
            out.append("% * " + normalize_line(item))
        elif re.match(r"^\d+\. ", s):
            item = re.sub(r"^\d+\. ", "", s)
            out.append("% " + s.split(".")[0] + ". " + normalize_line(item))
            out.append("%")  # keep numbered items as separate paragraphs
        elif s.startswith("> "):
            out.append("% NOTE: " + normalize_line(s[2:]))
        elif s.strip() == "---":
            pass
        else:
            body = normalize_line(s)
            out.append("% " + body if body else "%")
        i += 1

    text = "\n".join(out)
    # collapse 3+ consecutive %-blank separator lines
    text = re.sub(r"%\n%\n(%\n)+", "%\n%\n", text)
    return text.strip("\n") + "\n"


if __name__ == "__main__":
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2])
    dst.write_text(convert(src.read_text(encoding="utf-8")),
                   encoding="utf-8", newline="\n")
    print("wrote", dst)
