"""Report lines of derivations_report.m with TeX commands OUTSIDE $..$ spans."""
import re

bad = 0
for n, ln in enumerate(open("derivations_report.m", encoding="utf-8"), 1):
    if not ln.startswith("%"):
        continue
    txt = re.sub(r"\$\$.*\$\$", "", ln)          # display equations
    txt = re.sub(r"\$[^$]*\$", "", txt)          # inline math spans
    hits = re.findall(r"\\[a-zA-Z]+", txt)
    if hits:
        bad += 1
        if bad <= 10:
            print(n, hits[:4], "|", ln.strip()[:90])
print("lines with TeX outside math spans:", bad)
