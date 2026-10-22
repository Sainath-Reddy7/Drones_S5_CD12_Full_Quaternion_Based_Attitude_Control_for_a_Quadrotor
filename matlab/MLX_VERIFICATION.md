# Verification Report — quadrotor_math_derivations.mlx

Source of truth: `quadrotor_math_derivations_github.md` (desktop copy).
Artifact: `matlab/quadrotor_math_derivations.mlx` (regenerable via
`md_to_mlx.py` + `openAndSave`). Verifier: `matlab/mlx_verify.py` — parses
the MLX's internal document XML and the markdown into aligned block streams
(headings / paragraphs / display equations / monospace tables) and diffs
them after applying the documented TeX normalisation, so any reported
difference is a genuine content difference.

## A. Errors identified and corrected

Source-document errors (fixed in the .md, content-preserving):
1. Unbalanced trailing `$$` at end of file (1094-char math block swallowed
   the references section on GitHub; broke conversion).
2. `<https://doi.org/...>` autolink markup (renders literally in MATLAB).

Conversion errors found by the two-pass diff and fixed in `md_to_mlx.py`:
3. Display equations were being stored with inline style — now all 52 are
   display-style equation elements.
4. Numbered-list items merged into single paragraphs (5-item controller
   structure list became one blob) — items now separate paragraphs.
5. Markdown bold/italic emphasis wrapping math broke MATLAB's inline
   equation pairing, leaving literal `$...$`/`*...*` in 20+ paragraphs
   (e.g. "*Row 1: $\dot p$.*", "*(Answer: ...)*", `**13 states**`).
6. Double-quote hugging a `$` breaks pairing ("$R$" in the quoted
   paper sentence) — space inserted (verified by MATLAB probe).
7. Punctuation-only math spans `$+$`/`$-$` do not convert — emitted as
   plain text.
8. TeX outside the MATLAB subset normalised to equivalents that render:
   `\tfrac`/`\dfrac`→`\frac`, `\text`/`\operatorname`→`\mathrm`,
   `\mathcal X`/`\mathbb{R}`→`{X}`/`{R}` (brace-wrapped, so `\in\mathbb{H}`
   does not collapse into the invalid `\inH`), `\lVert`→`\Vert`,
   `\boxed`/`\big`/`\!` stripped, braceless `\frac` arguments braced.
9. Unicode→TeX conversion previously applied to prose (literal `\times`,
   `->` in text); now applied only inside math spans — prose keeps its
   original characters.
10. **Rendering vocabulary (visual-error round)**: the MLX equations
    contained 33 `\begin{bmatrix/aligned}` environments, 59 `\\` row
    breaks, and `\qquad`/`\left`/`\right`/`\arcsin`/`\lim`/`\Rightarrow`
    — none of which MATLAB's equation importer supports (its TeX table
    is the classic text-TeX subset, confirmed against the 230 visually
    proven equations of project_report.mlx). All rewritten to supported
    equivalents: matrices to bracket notation `[a, b; c, d]`, aligned
    blocks split into separate display equations, `\qquad`→`\;\;\;\;`,
    `\left`/`\right` dropped, function names to `\mathrm{...}`,
    `\Rightarrow`→`=>`, `\to`→`\rightarrow`, `\ge`→`\geq`. Final
    inventory: 37 unique commands, every one inside MATLAB's documented
    TeX table; zero environments, zero row breaks.
11. **Aligned-row defects (screenshot round)**: the two `aligned` blocks
    (Eqs. 7b, 14) were split into rows *after* TeX normalisation, which
    (a) left the alignment `&` in mid-row position (`\phi&=`) so MATLAB
    refused to render those five equations at all (they fell back to
    literal source text), (b) let the `\ `-rule corrupt the `\\` row
    separators into stray leading `;` on rows 2-3, and (c) dropped the
    `\qquad\text{(7b)}`/`(14)` tags that sit after `\end{aligned}`.
    Fixed by splitting rows on the raw TeX first, dropping every `&`,
    and re-attaching the tag to the last row. The same order bug existed
    in `mlx_verify.py`'s markdown parser (it validated the broken
    output against itself); fixed identically.
12. **Residual LaTeX hacks**: literal `[2pt]`/`[6pt]` row-spacing hacks
    (`\\[2pt]`) survived inside Eq. (8)'s matrix; thin spaces `\,` and
    nested braces from the `\mathcal` rule (`\dot{{X}}`, `\ddot{{X}}` in
    Eqs. 14/15a/15b) are not in MATLAB's TeX table. All normalised
    (`[Npt]` stripped, `\,`→`\;`, `{{X}}`→`{X}`).

## B. Verification performed

Content (automated, two independent passes):
- Passes after the aligned-row/tag round: **350 markdown blocks ↔
  350 MLX blocks, aligned 1:1, 0 mismatches** (earlier passes: 347/347,
  350/350).
- Structural stats: 482 equation elements (55 display + 427 inline),
  41 headings, 82 monospace table/fence lines, **0** code cells,
  **0** leftover markdown (`**`, `$$`, `|---`) and **0** TeX constructs
  outside MATLAB's documented subset (37 unique commands, all in the
  classic text-TeX table; `\beta` additionally confirmed against
  MATLAB's documented Greek-letter table).
- Junk scan of the equation payload itself (this round): **0** `&`,
  **0** `\\` row breaks, **0** `\begin{}`, **0** `[Npt]` hacks,
  **0** `\,`, **0** nested `{{X}}` braces; Eqs. (7b)/(14) tags present.

MATLAB:
- `.m → .mlx` conversion and `.mlx → .mlx` re-open via
  `matlab.internal.liveeditor.openAndSave` both succeed (R2024b) — the
  file parses without errors.
- Document XML is well-formed (ElementTree validation).

## C. Limitations (not claimed as verified)

- Equation **visual rendering** inside the Live Editor GUI cannot be
  verified headlessly; equations are stored as native Live Editor
  equation elements (raw TeX), which is the same mechanism the editor's
  own equation editor uses. Final visual check requires one manual look
  in MATLAB.
- The document is pure theory (derivations): it contains **no executable
  MATLAB code cells by design**, so there is nothing to run — no
  execution claims are made.
- Tables are aligned monospace text blocks, not native MLX tables: the
  `.m`-to-`.mlx` pipeline has no table markup, and hand-authoring native
  table XML risks file corruption. All table content is present and
  verified (see A/pass results).
- Markdown's two heading levels map to one MLX heading level (no
  sub-heading markup exists in script form); hierarchy is preserved by
  the existing "Eq. (N) —" numbering.
