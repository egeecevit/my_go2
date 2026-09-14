# MPC reports: what to trust

This folder holds the measurement history of `MdlConvexMPC` and its interaction
with `MdlTrot`'s gait. Everything dated before 2026-09-13 predates a
correction to `models/unitree_go2/go2.xml`'s joint damping, which changed the
plant enough to invalidate most of the quantitative findings measured against
it. This file exists so a reader can tell, in the time it takes to read a
table, which documents still describe the robot as it is.

## Status table

| file | status | why | what survives |
|---|---|---|---|
| `feedforward_horizon_repair.md` | still current | Changelog of changes present in the code today. Its acceptance-run numbers were taken at joint damping 2.0, so treat those specific figures as pre-correction, but the changes themselves are live. | The changes described (QP contact compaction, integer solve cadence, ground-truth bypass, Go2 leg-dynamics model, stance-sweep anchoring) — all present in the tree. |
| `archive/mit_cheetah_mpc_plan.md` | COMPLETED, SUPERSEDED AS GUIDANCE | The convex MPC it specifies was built and works; kept as the design record. Superseded as forward-looking guidance by `WBIC_PLAN.md`. | The implementation itself. Its prohibition on whole-body control in `MdlTrot` no longer applies — WBIC is the intended next step. |
| `archive/horizon_sweep.html` ("Shorter Than One Stance") | CONCLUSION FALSIFIED | Concluded longer lookahead is ~5x worse; at corrected damping, 0.75 s and 1.5 s lookahead both ran 40 s without falling. | Correctly identified that raising `dt` to lengthen lookahead also coarsens the contact-schedule grid — the real mechanism behind the later-measured `dt` effect, even though the report drew the wrong conclusion from it. |
| `archive/horizon_adjudication.html` ("Real Until Touchdown") | CENTRAL CONCLUSION VOID | Its "model stops predicting past ~0.1 s" finding rests on an unsound crossover metric (see below) and was measured at damping 2.0. | `test_discretization_consistency` — QP prediction verified against independent RK4 to 1e-7; still in the tree and passing. Its estimator-in-loop yaw-drift section is unreviewed, not void. |
| `archive/stage_a_envelope.md` | QUANTITATIVELY VOID | Measured at damping 2.0: concluded the gait falls above 0.55 m/s and at any tested yaw rate. At corrected damping the robot trots at 0.95 m/s with zero torque clipping, peak torque 54-73% of limit. | Its methodology: falls must be detected kinematically (`MdlSimDriver_qpos[:,2] < 0.20 m`), not from the console — a robot lying on the ground for two minutes still prints "autorun complete" with no error. |
| `archive/b3_term_magnitudes.md` | QUANTITATIVELY VOID | Term magnitudes measured at damping 2.0, on motion that no longer occurs. | Verification by hypothesis test that `MdlSimDriver_qvel[0:3]` is WORLD-frame linear velocity and `qvel[3:6]` is BODY-LOCAL angular velocity — a permanent fact about the log format. |
| `archive/swing_gain_audit.md` | SIGNS INVERTED, QUANTITATIVELY VOID | Declares "negative = nose down"; the code's `rpyFromQuat` convention makes positive pitch nose-down, so every pitch sign is suspect. Also measured at damping 2.0. | The resonance argument: the swing trajectory's own fundamental is `2π/T_sw`, and `swing_natural_frequency` near it puts the swing loop on resonance with its own reference. Adopted — `trot.toml` now ships `swing_natural_frequency = 50`, up from the 28 found sitting at 0.90 of the swing fundamental. |

## The two corrections that invalidated the rest

1. **Joint damping.** `models/unitree_go2/go2.xml` had `damping="2"` N·m·s/rad
   on every joint — a mujoco_menagerie house default, 20x Unitree's own URDF
   value of 0.1. It dissipated 153.5 W against 179.2 W of positive motor work
   (86%), gave a mechanical cost of transport of 1.45, and an equivalent drag
   of 190 N. It is now 0.2. This changed the plant enough to halve the
   velocity shortfall and flip the sign of the pitch bias. Every quantitative
   result measured at damping 2.0 describes a different robot.
2. **The "model validity" metric.** Used by `horizon_adjudication.html` and
   referenced by `swing_gain_audit.md`, it reported the horizon step where the
   QP's prediction error exceeds the error of believing the reference. That
   crossover tracks the *configured horizon* (step 14 at H15, 19 at H20, 27 at
   H30, 37 at H40, all at `dt=0.01`), not the robot — the QP minimises
   deviation from the reference, so its plan converges toward that reference
   across the horizon, and the two error curves meet asymptotically. At
   `dt=0.05` it reports a crossover at step 0 for configurations that then
   walk 40 s.

## Facts that survived the triage

These are worth having without opening an archived report:

- `MdlSimDriver_qvel[0:3]` is WORLD-frame linear velocity; `qvel[3:6]` is
  BODY-LOCAL angular velocity. Verified by hypothesis test, not assumed.
- Falls must be detected kinematically (trunk height, e.g.
  `MdlSimDriver_qpos[:,2] < 0.20 m`), never from the console: a robot that has
  been lying on the ground for two minutes still prints "autorun complete"
  with no error.
- The swing loop's resonance mechanism is real and independent of damping:
  the swing trajectory's own fundamental is `2π/T_sw`, and putting
  `swing_natural_frequency` near it resonates the loop with its own
  reference. `trot.toml` now ships `swing_natural_frequency = 50` (was 28,
  which sat at 0.90 of the swing fundamental).
- The QP's condensed prediction was verified against an independent RK4
  integrator to 1e-7 across a mid-horizon contact change
  (`test_discretization_consistency`), closing off "implementation bug" as an
  explanation for anything downstream.
- The `dt`/contact-quantisation confound: `MdlTrot` builds the contact
  schedule at `tk = elapsed + k·dt`, so raising `dt` to buy lookahead also
  coarsens the grid touchdown is quantised to. `dt` governs tracking; the
  horizon governs only solve cost.

## Where the current truth lives

- `WBIC_PLAN.md` at the repo root — the current plan and status.
- The findings report: https://claude.ai/code/artifact/42f9f256-6a1b-46d9-aa69-b55e68f0cf13
- Tooling: `figs/mpc/score_run.py` (scorecard), `figs/mpc/run_cell.sh`
  (serialised headless runs), `figs/mpc/mpcdump.py` (per-solve prediction
  dump parser).

`reports/state_estimator/` has not been triaged in this pass.
