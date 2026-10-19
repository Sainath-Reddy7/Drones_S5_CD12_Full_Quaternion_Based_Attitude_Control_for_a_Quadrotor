# MATLAB Report — Completeness Audit

Report: `matlab/project_report.m` (source), `matlab/project_report.mlx`
(Live Script), `matlab/published/project_report.pdf` (33 pages, MATLAB
publish, JPEG figure format). Audited against the repository.

## MATLAB implementation (baseline preserved)

- All validated code preserved unchanged: quaternion ops, paper-1 plant +
  law (run_paper1/integ1/ref1), Riccati (riccati_ham), mixer
  (mix_feasible), flatness map (flatness_reference), trajectory
  generators (make_circle/figure8/orbit/waypoints, traj_eval, ramp,
  minjerk), closed loop (run_paper2/integ2). The ONLY additions:
  show_img helper + capture-flush cells (publish needs a later code cell
  to flush a figure capture - measured), `close all` guards before each
  recording frame, and the cover/catal­ogue text. Numbers re-verified
  identical (step 16.3, sine 17.4, flip 44.4 deg; circle 2.7 cm radial;
  fig8 4.7; orbit 3.9; min-snap 6.1 vs min-jerk 8.1 cm).

## MuJoCo materials — INCLUDED

- Adapter + conventions documented (Section 6 intro, Section 16).
- In-engine renders documented M1-M6 with frames embedded (Figures M1,
  M4 = pages 18-19 of the PDF); path traces M7-M10 with Figure M9
  (page 20); three-plant paper-2 results (Section 10).
- All 20 repository GIFs inventoried (13 MuJoCo-derived, 6 ideal-model,
  plus gym_step legacy = 20 files in results/recordings/), each
  catalogue entry names its file, frame count and metrics.

## GymPyBullet materials — INCLUDED (documented separately)

- Adapter documented (custom colour-coded URDF, rotor mapping, 250 Hz
  altitude controller) in Sections 14/16; recordings G1-G3 with Figure
  G1 (page 21); step/sine/flip results in Section 16. Explicitly noted
  as a DIFFERENT engine with separate implementation; compared with
  MuJoCo only where evidence supports it (0.03 deg step agreement;
  flip knife-edge divergence).

## Team + university — INCLUDED

- Both members with roll numbers (B Sainath CB.SC.U4AIE24309, Vishal
  CB.SC.U4AIE24363), School of Artificial Intelligence, Amrita School
  of Engineering Coimbatore, Amrita Vishwa Vidyapeetham.
- Logo: the repository README asset is white-on-transparent; rendered as
  a navy banner on the cover (page 3, pixel-verified).

## Verification chain

- 7 equation groups re-verified in-script (algebra, law+saturate,
  inertia, model rows, mixer, flatness map, LQR Hurwitz).
- PDF: 7 distinct marked images embedded (hash-checked, no duplicates),
  logo navy banner pixel-verified on page 3, no raw $ / HTML / error
  dumps (text-extraction checked), page numbers sequential.
- Full visual judge pass: 33/33 pages, only cosmetic note = baked-in
  caption on one GIF frame (padded).

## Outstanding (explicit, not invented)

- Faculty-guide details: absent from all project materials — not stated.
- E3 flatness feed-forward (omega_d/tau_ff): specified in the extension
  proposal, NOT yet run.
- PyBullet port of the paper-2 stack; wind-robustness runs for the
  planner.
- ArduPilot/Gazebo runners live in the Python repo (WSL2 required);
  their numbers are quoted from results/comparison.md, not re-run in
  MATLAB.
