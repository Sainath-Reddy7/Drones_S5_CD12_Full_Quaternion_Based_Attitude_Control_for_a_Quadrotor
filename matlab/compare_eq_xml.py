"""Compare equation-element XML structure: working project_report.mlx vs derivations."""
import re
import sys
import zipfile


def probe(path, label):
    xml = zipfile.ZipFile(path).read("matlab/document.xml").decode("utf-8")
    print(f"===== {label} =====")
    # all equation open-tag variants
    tags = [t for t in re.findall(r"<w:customXml[^>]*>", xml) if "equation" in t]
    from collections import Counter
    for t, n in Counter(tags).most_common(3):
        print(f"open-tag x{n}: {t}")
    # first equation element, full XML
    m = re.search(r"<w:customXml[^>]*equation[^>]*>.*?</w:customXml>", xml, re.S)
    if m:
        s = m.group(0)
        print(f"first equation element ({len(s)} chars):")
        print(s[:600].replace("\n", "\\n"))
    # does TeX live in CDATA or w:t?
    inner = m.group(0) if m else ""
    print("CDATA present:", "<![CDATA[" in inner)
    wt = re.findall(r"<w:t[^>]*>([^<]*)</w:t>", inner)
    print("w:t children:", wt[:3])
    # paragraph-level: what wraps the element (pStyle of containing paragraph)
    ctx = xml[max(0, m.start() - 400):m.start()] if m else ""
    st = re.findall(r'<w:pStyle w:val="([^"]+)"', ctx)
    print("containing paragraph styles near first eq:", st[-1] if st else "?")
    print()


probe(sys.argv[1], "WORKING: project_report.mlx")
probe(sys.argv[2], "DERIVATIONS")
