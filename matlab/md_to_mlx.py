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
    (r"\\lVert", r"\\Vert"), (r"\\rVert", r"\\Vert"),
    (r"\\boxed\{([^{}]*)\}", r"\1"),
    (r"\\bigg?\b", ""), (r"\\Big\b", ""), (r"\\!", ""), (r"\\;", " "),
    (r"\\succeq", r"\\geq"), (r"\\succ", ">"),
    (r"×", r"\\times"), (r"≈", r"\\approx"), (r"±", r"\\pm"),
    (r"−", "-"), (r"°", r"^\\circ"), (r"≥", r"\\geq"), (r"µ", r"\\mu"),
    (r"—", "-"), (r"–", "-"), (r"→", "->"), (r"\\dot\{X\}", r"\\dot{X}"),
]


def tex_normalize(s: str) -> str:
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
    return s


def strip_inline_math(s: str) -> str:
    return re.sub(r"\$([^$]*)\$", r"\1", s)


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
                out.append("% $$" + tex_normalize(eq) + "$$")
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
            item = re.sub(r"\*\*(.+?)\*\*", r"\1", item)
            out.append("% * " + item if "$" not in item else
                       "% * " + tex_normalize(item))
        elif re.match(r"^\d+\. ", s):
            item = re.sub(r"^\d+\. ", "", s)
            item = re.sub(r"\*\*(.+?)\*\*", r"\1", item)
            out.append("% " + s.split(".")[0] + ". " + item)
        elif s.startswith("> "):
            body = re.sub(r"\*\*(.+?)\*\*", r"\1", s[2:])
            out.append("% NOTE: " + body)
        elif s.strip() == "---":
            pass
        else:
            body = re.sub(r"\*\*(.+?)\*\*", r"\1", s)
            if "$" in body:
                body = tex_normalize(body)
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
