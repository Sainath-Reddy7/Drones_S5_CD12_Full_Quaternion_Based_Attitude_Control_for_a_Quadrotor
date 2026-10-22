"""Scan the rendered PDF text layer for literal TeX leakage."""
import re
import subprocess

txt = subprocess.run(
    ["C:/Users/bsain/AppData/Local/Programs/MiKTeX/miktex/bin/x64/pdftotext.exe",
     "pub_probe/derivations.pdf", "-"],
    capture_output=True, text=True).stdout

tex_cmds = re.findall(r"\\[a-zA-Z]+", txt)
dollars = txt.count("$")
norm_sym = txt.count("\u2208")  # ∈
print("literal TeX commands in rendered text:", len(tex_cmds), sorted(set(tex_cmds))[:10])
print("literal $ signs:", dollars)
print("rendered \u2208 symbols:", norm_sym)
