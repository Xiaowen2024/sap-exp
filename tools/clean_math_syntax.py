import re

with open("docs/mechanism-eval.html", "r", encoding="utf-8") as f:
    content = f.read()

# 1. Add KaTeX CDN to head if not present
katex_head = """  <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">
  <script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>
  <script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js" onload="renderMathInElement(document.body, {delimiters: [{left: '$$', right: '$$', display: true}, {left: '$', right: '$', display: false}]});"></script>
"""
if "katex.min.js" not in content:
    content = content.replace("  <link rel=\"stylesheet\" href=\"styles.css\">\n</head>",
                              "  <link rel=\"stylesheet\" href=\"styles.css\">\n" + katex_head + "</head>")

# 2. String literal replacements
str_replacements = [
    ("$B \\le 5$", "<i>B</i> &le; 5"),
    ("$B=1$", "<i>B</i> = 1"),
    ("$B = 1$", "<i>B</i> = 1"),
    ("$B \\ge 100$", "<i>B</i> &ge; 100"),
    ("$B \\approx 100$", "<i>B</i> &approx; 100"),
    ("$B = 80\\text{--}96$", "<i>B</i> = 80&ndash;96"),
    ("$B = 1\\text{--}40$", "<i>B</i> = 1&ndash;40"),
    ("$B = 2$", "<i>B</i> = 2"),
    ("$B = 4$", "<i>B</i> = 4"),
    ("$B \\le 2$", "<i>B</i> &le; 2"),
    ("$B$", "<i>B</i>"),
    ("$\\ge 95\\%$", "&ge; 95%"),
    ("$\\pm 2\\times$", "&plusmn;2&times;"),
    ("$|\\rho_s|$", "|<i>&rho;<sub>s</sub></i>|"),
    ("$|\\rho_s| \\ge 0.83$", "|<i>&rho;<sub>s</sub></i>| &ge; 0.83"),
    ("$|\\rho_s| \\ge 0.88$", "|<i>&rho;<sub>s</sub></i>| &ge; 0.88"),
    ("$|\\rho_s|=1.00$", "|<i>&rho;<sub>s</sub></i>| = 1.00"),
    ("$(\\tau_n, k_n)$", "(<i>&tau;<sub>n</sub></i>, <i>k<sub>n</sub></i>)"),
    ("$(k_n, \\tau_n)$", "(<i>k<sub>n</sub></i>, <i>&tau;<sub>n</sub></i>)"),
    ("$(k_e, \\tau)$", "(<i>k<sub>e</sub></i>, <i>&tau;</i>)"),
    ("$180\\text{ Hz}$", "180 Hz"),
    ("$\\tau_n \\le 5\\text{ ms}$", "<i>&tau;<sub>n</sub></i> &le; 5 ms"),
    ("$\\tau \\le 0.68 < 1.0$", "<i>e</i> &le; 0.68 &lt; 1.0"),
    ("$e \\le 0.68 < 1.0$", "<i>e</i> &le; 0.68 &lt; 1.0"),
    ("$e \\le 0.678 < 1.0$", "<i>e</i> &le; 0.678 &lt; 1.0"),
    ("$e > 1.0$", "<i>e</i> &gt; 1.0"),
    ("$e=0.73$", "<i>e</i> = 0.73"),
    ("$e_{\\text{real}} \\approx 0.727$", "<i>e</i><sub>real</sub> &approx; 0.727"),
    ("$e_{\\text{real}}=0.73$", "<i>e</i><sub>real</sub> = 0.73"),
    ("$e_{\\text{sim}} = 0.00$", "<i>e</i><sub>sim</sub> = 0.00"),
    ("$e_{\\text{sim}} = 0.729$", "<i>e</i><sub>sim</sub> = 0.729"),
    ("$e_{\\text{sim}} = 0.678$", "<i>e</i><sub>sim</sub> = 0.678"),
    ("$R_t = \\sigma w_i$", "<i>R<sub>t</sub></i> = <i>&sigma; w<sub>i</sub></i>"),
    ("$mg$", "<i>mg</i>"),
    ("$\\Delta t = 5.56\\text{ ms}$", "&Delta;<i>t</i> = 5.56 ms"),
    ("$\\sqrt{2}\\mu$", "&radic;2<i>&mu;</i>"),
    ("$\\mu_k \\to 0.230$", "<i>&mu;<sub>k</sub></i> &rarr; 0.230"),
    ("$v_{\\text{rel}} \\sim 1\\text{ m/s}$", "<i>v</i><sub>rel</sub> &sim; 1 m/s"),
    ("$d_{\\text{ref}} \\downarrow \\implies e_{\\text{sim}} \\uparrow$", "<i>d</i><sub>ref</sub> &darr; &rArr; <i>e</i><sub>sim</sub> &uarr;"),
    ("$\\tau \\downarrow \\implies e_{\\text{sim}} \\uparrow$", "<i>&tau;</i> &darr; &rArr; <i>e</i><sub>sim</sub> &uarr;"),
    ("$0.2 \\to 1.0$", "0.2 &rarr; 1.0"),
    ("$\\tau$", "<i>&tau;</i>"),
    ("$0 \\to 10\\text{ ms}$", "0 &rarr; 10 ms"),
    ("$k_e$", "<i>k<sub>e</sub></i>"),
    ("(\\text{N/m})", "(N/m)"),
]

for src, dst in str_replacements:
    content = content.replace(src, dst)

with open("docs/mechanism-eval.html", "w", encoding="utf-8") as f:
    f.write(content)

print("Replacement complete. Checking remaining '$':")
with open("docs/mechanism-eval.html", "r", encoding="utf-8") as f:
    matches = re.findall(r"\$([^$]+)\$", f.read())
    print(f"Remaining inline math count: {len(matches)}")
    for m in matches:
        print("  rem:", m)
