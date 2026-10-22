"""Structural stats + junk scan for quadrotor_math_derivations.mlx."""
import re
import sys
import zipfile

PROVEN = set("""alpha angle approx bar cdot circ cos ddot delta dot frac geq
hat in infty int lambda ldots leq mathbf mathrm min neq omega Omega otimes
partial phi pi pm psi rho rightarrow Sigma sigma sin sqrt sum tau theta
times Xi zeta circ""".split())

xml = zipfile.ZipFile(sys.argv[1]).read("matlab/document.xml").decode("utf-8")

eqs = re.findall(r'w:element="equation"[^>]*>(.*?)</w:customXml>', xml, re.S)
tex_all = []
for e in eqs:
    parts = re.findall(r"<!\[CDATA\[(.*?)\]\]>", e, re.S)
    if not parts:
        parts = [re.sub(r"<[^>]+>", "", e)]
    tex = " ".join(p.strip() for p in parts if p.strip())
    tex_all.append(tex)
# display equations = paragraphs that contain ONLY an equation element
paras = re.findall(r"<w:p [^>]*>(.*?)</w:p>", xml, re.S)
display = sum(1 for p in paras
              if 'w:element="equation"' in p and "<w:t" not in p
              and p.count('w:element="equation"') == 1)

cmds = set()
for t in tex_all:
    cmds.update(re.findall(r"\\([a-zA-Z]+)", t))

styles = re.findall(r'<w:pStyle w:val="([^"]+)"', xml)
from collections import Counter
c = Counter(styles)

joined = " ".join(tex_all)
junk = {
    "begin-env": len(re.findall(r"\\begin", joined)),
    "row-break": len(re.findall(r"\\\\", joined)),
    "ampersand": joined.count("&"),
    "pt-hack": len(re.findall(r"\[\d+pt\]", joined)),
    "thin-comma": joined.count("\\,"),
    "nested-brace": len(re.findall(r"\{\{[A-Za-z]\}\}", joined)),
}
outside = sorted(cmds - PROVEN)

print(f"equation elements : {len(eqs)}  (display-style: {display})")
print(f"headings          : {c.get('heading', 0)}  title: {c.get('title', 0)}")
print(f"monospace lines   : {c.get('CodeExampleLine', 0)}")
print(f"code cells        : {len(re.findall(r'<w:pStyle w:val="CodeSection"', xml))}")
print(f"unique TeX cmds   : {len(cmds)}  outside proven set: {outside or 'NONE'}")
print(f"junk scan         : {junk}")
