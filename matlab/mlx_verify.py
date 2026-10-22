"""Verify quadrotor_math_derivations.mlx against the source markdown.

Parses the MLX (OPC zip -> WordprocessingML-ish document.xml) and the
markdown into comparable block streams, then diffs them:

  H  section heading          P  paragraph (text + inline eq TeX)
  E  display equation (TeX)   M  monospace block (fences, tables)

The markdown TeX is normalised with the same rule set the converter
uses, so any difference reported is a genuine content difference.

Usage:  mmenv python mlx_verify.py <file.mlx> <file.md>
"""
import re
import sys
import zipfile

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from md_to_mlx import tex_normalize, split_aligned  # noqa: E402


def md_blocks(md: str):
    """-> list of (kind, payload). kind in H P E M."""
    blocks = []
    lines = md.split("\n")
    i = 0
    fence = []
    in_fence = False
    while i < len(lines):
        ln = lines[i]
        s = ln.strip()
        if ln.startswith("```"):
            if in_fence:
                blocks.append(("M", tuple(fence)))
                fence = []
            in_fence = not in_fence
            i += 1
            continue
        if in_fence:
            fence.append(s)
            i += 1
            continue
        if s == "$$":
            eq = []
            i += 1
            while i < len(lines) and lines[i].strip() != "$$":
                eq.append(lines[i].strip())
                i += 1
            i += 1  # closer
            for row in split_aligned(
                    tex_normalize(" ".join(x for x in eq if x))):
                blocks.append(("E", row))
            continue
        if s.startswith("|"):
            if re.match(r"^\|[\s:|-]+\|$", s):
                i += 1
                continue
            cells = [c.strip() for c in s.strip("|").split("|")]
            cells = [strip_bold(c) for c in cells]
            row = "  ".join(
                tex_normalize(strip_dollar_plain(c)) if "$" in c else c
                for c in cells)
            if blocks and blocks[-1][0] == "T":
                blocks[-1][1].append(row)
            else:
                blocks.append(("T", [row]))
            i += 1
            continue
        if blocks and blocks[-1][0] == "T" and not s.startswith("|"):
            blocks.append(("M", tuple(blocks.pop()[1])))
        if not s or s == "---":
            i += 1
            continue
        if s.startswith("#"):
            head = s.lstrip("#").strip()
            head = strip_dollar_plain(strip_bold(head))
            blocks.append(("H", re.sub(r"\s+", " ", head)))
        elif s.startswith("> "):
            blocks.append(("P", "NOTE: " + strip_dollar_plain(
                strip_bold(s[2:]))))
        elif s.lstrip().startswith("- ") or re.match(r"^\d+\. ", s):
            blocks.append(("P", strip_dollar_plain(strip_bold(s))))
        else:
            # plain paragraph line: join with following continuation lines
            j = i + 1
            buf = [s]
            while j < len(lines):
                nxt = lines[j].strip()
                if (not nxt or nxt == "---" or nxt.startswith(("#", "|", "> ",
                        "- ", "$$", "```")) or re.match(r"^\d+\. ", nxt)):
                    break
                buf.append(nxt)
                j += 1
            blocks.append(("P", strip_dollar_plain(
                strip_bold(" ".join(buf)))))
            i = j
            continue
        i += 1
    if blocks and blocks[-1][0] == "T":
        blocks.append(("M", tuple(blocks.pop()[1])))
    return blocks


def strip_bold(s: str) -> str:
    return re.sub(r"\*\*(.+?)\*\*", r"\1", s)


def strip_dollar(s: str) -> str:
    return re.sub(r"\$([^$]*)\$",
                  lambda m: "@" + tex_normalize(m.group(1)) + "@", s)


def strip_dollar_plain(s: str) -> str:
    return re.sub(r"\$([^$]*)\$", lambda m: tex_normalize(m.group(1)), s)


def mlx_blocks(xml: str):
    """-> list of (kind, payload) from document.xml paragraph stream."""
    import xml.etree.ElementTree as ET
    root = ET.fromstring(xml)
    W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"
    blocks = []
    for p in root.iter(W + "p"):
        style = ""
        ppr = p.find(W + "pPr")
        if ppr is not None:
            ps = ppr.find(W + "pStyle")
            if ps is not None:
                style = ps.get(W + "val", "")
        parts = []

        def walk(node):
            for ch in list(node):
                if ch.tag == W + "customXml" and \
                        ch.get(W + "element") == "equation":
                    tex = "".join(t.text or "" for t in ch.iter(W + "t"))
                    parts.append("@" + tex.strip() + "@")
                else:
                    if ch.tag == W + "t":
                        parts.append(ch.text or "")
                    walk(ch)

        walk(p)
        text = "".join(parts)
        text_sq = re.sub(r"\s+", " ", text).strip()
        if not text_sq:
            continue
        if style in ("title", "heading"):
            blocks.append(("H", text_sq))
        elif style == "CodeExampleLine":
            blocks.append(("M-line", text.strip()))
        elif re.fullmatch(r"@[^@]+@", text_sq):
            blocks.append(("E", text_sq[1:-1].strip()))
        else:
            blocks.append(("P", text_sq))
    merged = []
    for k, v in blocks:
        if k == "M-line":
            if merged and merged[-1][0] == "M":
                merged[-1][1].append(v)
            else:
                merged.append(("M", [v]))
        else:
            merged.append((k, v))
    return merged


def _is_mono(p: str) -> bool:
    return 'w:val="CodeExampleLine"' in p


def squash(s: str) -> str:
    """Content-level comparison key: ignore whitespace, equation-element
    markers and markdown emphasis stars; keep words and TeX verbatim."""
    s = s.replace("@", "")
    s = re.sub(r"(?<![\w*])\*|\*(?![\w*])", "", s)  # stray emphasis stars
    return re.sub(r"\s+", "", s)


def compare(md_path, mlx_path):
    md = open(md_path, encoding="utf-8").read()
    xml = zipfile.ZipFile(mlx_path).read("matlab/document.xml").decode("utf-8")
    A = md_blocks(md)          # source of truth
    B = mlx_blocks(xml)
    # drop the mlx title paragraph if md's first heading equals it
    if B and B[0][0] == "H" and A and B[0][1] == A[0][1]:
        pass
    # merge adjacent monospace blocks on both sides (a blank line between
    # a fence and a table is a formatting separator, not content)
    def merge_m(seq):
        out = []
        for k, v in seq:
            if k == "M" and out and out[-1][0] == "M":
                out[-1] = ("M", list(out[-1][1]) + list(v))
            else:
                out.append((k, v))
        return out

    A = merge_m(A)
    B = merge_m(B)
    print(f"md blocks: {len(A)}   mlx blocks: {len(B)}")
    n_mis = 0
    for idx in range(max(len(A), len(B))):
        a = A[idx] if idx < len(A) else ("<missing>", "")
        b = B[idx] if idx < len(B) else ("<missing>", "")
        ok, why = False, ""
        if a[0] == b[0]:
            if a[0] in ("H", "E", "P"):
                ok = squash(str(a[1])) == squash(str(b[1]))
                why = "text/TeX differs"
            elif a[0] == "M":
                am = [squash(x) for x in a[1]]
                bm = [squash(x) for x in b[1]]
                ok = am == bm
                why = "monospace block differs"
        if not ok and a[0] == "P" and b[0] == "E":
            # a bullet whose whole payload is one math span becomes a
            # standalone equation paragraph in the mlx
            core = re.sub(r"^([-*#>.]|\d+\.)\s*", "", str(a[1]))
            am = squash(strip_dollar_plain(core).replace("@", ""))
            bm = squash(str(b[1]).replace("@", ""))
            ok = am == bm
            why = "pure-math bullet vs equation"
        if not ok and a[0] == "P" and b[0] == "P":
            am = squash(re.sub(r"^([-*#>.]|\d+\.)\s*", "", str(a[1])))
            bm = squash(str(b[1]))
            ok = am == bm
            why = "text differs (ignoring list marker)"
        if not ok:
            n_mis += 1
            print(f"--- block {idx}: md={a[0]} mlx={b[0]} ({why})")
            sa = str(a[1])
            sb = str(b[1])
            print(f"    md : {sa[:120]}")
            print(f"    mlx: {sb[:120]}")
            if n_mis > 30:
                print("... (stopping after 30 mismatches)")
                break
    print(f"total mismatched blocks: {n_mis}")
    return n_mis


if __name__ == "__main__":
    sys.exit(1 if compare(sys.argv[2], sys.argv[1]) else 0)
