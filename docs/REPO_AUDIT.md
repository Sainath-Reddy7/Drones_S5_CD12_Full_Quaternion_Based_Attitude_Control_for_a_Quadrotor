# Repository Audit Log

Whole-repo audit rounds applied to this repository. Each entry lists what was
changed and how it was verified. Run `pytest` after every round: the suite
must stay green (83 passed, 2 skipped — WSL-only backends — in `.venv310`).

## Round 3 — full-repository audit (this round)

Static analysis over every tracked file (byte-compile of all Python modules,
content-hash duplicate scan, machine-specific-path scan, doc-link check,
imports-vs-requirements check), plus a manual math review of the paper-1 core.

Corrections:

- **Removed a root-level duplicate HTML file.**
  `fresk_nikolakopoulos_precise_attitude_simulator_xyz_labels(1).html` was
  byte-identical to `deploy/simulator.html` (the GitHub Pages artifact). The
  README link now points at `deploy/simulator.html` directly (the old link
  used URL-encoded parentheses, `%281%29`, which broke local viewers).
- **Removed a stray session log** (`sim/interactive/logs/session_214755.csv`,
  referenced nowhere; the directory is already in `.gitignore`).
- **`requirements.txt`** now documents the optional simulation backends
  (mujoco, pybullet, gym-pybullet-drones, keyboard) and where the
  ArduPilot/PDF build extras live, instead of leaving them undeclared.
- **README test counts corrected**: 85 collected (table rows for
  `test_sim_adapters.py` 5→6 and `test_sim_interactive.py` 14→15; the
  structure-table claim 70→85).

Verified intentional duplicates (kept): `deploy/figures/` mirrors result
figures for the Vercel bundle; `results/sim_*/guid_gated_archive/` is a
historical snapshot (partially identical to current results — archived runs);
`results/flatness/` and `results/flatness/scenarios/` are separate benchmark
runs (12 of 13 files differ).

Verification: byte-compile clean; zero machine-specific paths in tracked
files; zero broken doc links; full suite re-run green; fresh-clone check —
core math tests pass from a clean checkout.

## Round 2 — figure de-duplication

Removed 7 redundant copies in `matlab/figures/` (5 byte-identical + 2 stale
variants) — the MATLAB report references the `matlab/`-root copies by bare
filename; nothing referenced the `figures/` copies.

## Round 1 — derivations Live Script

`docs/quadrotor_math_derivations.md` superseded by
`matlab/quadrotor_math_derivations.mlx` (converted from the markdown source,
verified block-by-block; see `matlab/MLX_VERIFICATION.md`).
