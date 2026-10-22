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

## B. Verification performed

Content (automated, two independent passes):
- Pass 1 and pass 2 both: **347 markdown blocks ↔ 347 MLX blocks, aligned
  1:1, 0 mismatches** — every heading, paragraph, derivation step,
  equation, table row, note and reference is present, in order, with
  identical wording and TeX (modulo the documented normalisations).
- Structural stats: 479 equation elements (52 display + 427 inline),
  41 headings, 82 monospace table/fence lines, **0** code cells,
  **0** leftover markdown (`**`, `$$`, `|---`) or out-of-subset TeX
  (`\tfrac`, `\text{`, `\boxed`, `\operatorname`, `\mathbb`, `\lVert`).

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
