# Handoff: AMB inside the WBIC + MPC trot

Written 2026-11-26, at the end of the session that put the trot on whole-body impulse control
(Kim et al. 2019). Nothing of AMB is in the trot yet. This file is the starting point for that work.

## The research question (Ege, 2026-11-26)

AMB keeps the robot balanced on a diagonal pair of feet. Two questions follow:

1. Can that be used for **slow trotting**, meaning a trot whose two-foot (diagonal) support phases are long?
   Ege's idea: with AMB the two-foot phase can be prolonged, and that is useful.
2. Does AMB help at **fast trotting**?

**Terminology (Ege confirmed, 2026-11-26): "double support" is the two-foot diagonal phase.** Slow trotting
means long two-foot phases. In `MdlTrot` the timing is

- two-foot (diagonal) phase = `(1 − duty) · period` = the swing time of the other pair,
- four-foot phase = `(duty − 0.5) · period`,
- below duty 0.5 there is no four-foot phase but a flight phase of `(0.5 − duty) · period`, and the two-foot
  phase is `duty · period`.

At a fixed period a higher duty factor shortens the two-foot phase and adds four-foot overlap, so a longer
two-foot phase needs a longer swing time, not only a higher duty factor. Parametrise the gait by the two
durations, not by period and duty: `period = 2 (t_two + t_four)`, `duty = 1 − t_two / period`.

## Where things are

- Repo `/home/ege/legged/iros/project`, branch `amb`. The whole-body trot is commit `99655d6` ("wbic+mpc
  implementation done", not pushed when this was written). On top of it, **uncommitted**: the flight-phase fix of
  2026-11-27 (`QuadrupedWBC.{hh,cc}`, `MdlTrot.{hh,cc}`, `tests/test_wbc.cc`, `tests/test_trot_wbc.cc`,
  `config/default/trot.toml`, `figs/trot_wbc/`, `reports/trot_wbc/`). Ege commits; Claude only when asked.
- `[trot] wbc = true` selects the whole-body trot. Default is `false`: key T runs the old per-leg trot.
- Results: `reports/trot_wbc/REPORT.pdf` (10 pages), rebuilt by
  `/home/ege/legged/iros/.venv/bin/python figs/trot_wbc/run_experiments.py` (189 runs, 20 min; `--only`, `--out`).
- Tests: `cd build && ctest` gives 21/21.
- Plan of this session: `/home/ege/.claude/plans/i-want-to-investigate-hazy-hammock.md`.
  Earlier handoff (still valid for the AMB side): `/home/ege/legged/iros/docs/HANDOFF_trot_wbc_amb.md`.
- Thesis as markdown: `/home/ege/legged/quadcontrol-amb/paper/*.md` (Ch. 6 is `06-wbc-integration.md`).

## What the whole-body trot is

| Piece | Where | Notes |
|---|---|---|
| Task assembly + solve | `MdlTrot::wbcSolve` (static, testable) | contacts > body orientation > base position > one task per swing foot; gains 100/10 |
| Per-tick glue | `MdlTrot::_wbcTick` | state from the two estimator modules, MPC first-step forces as `f_des`, joint command τ + PD 30/3 on every leg |
| Seam in the old code | `MdlTrot::_trotDuring`: `if (_wbc && _wbcTick()) return;` | gait clock, footstep law, swing quintic, body reference, MPC call are shared with the old path |
| Solver, model | `QuadrupedWBC`, `QuadrupedDynamics` | `MAX_TASKS = 6`: two body tasks and up to four swing feet; a problem with no contact (flight) is solved, the base then falls freely |
| Touchdown height | `_startSwing`: `wbc_touchdown_depth` below the foot's lift-off height | old path keeps the nominal footprint height |
| Log | `MdlTrot_wbc` (47 doubles, layout at `_logWbc` in `MdlTrot.hh`) | desired and solved forces, QP status, task errors, swing errors, tick time, the MPC's roll and pitch reference |
| Attitude integral | `_trotDuring`, before the body reference is integrated | `_attitudeComp` enters `_solveMPC` as the reference roll and pitch |

Decisions Ege took on 2026-11-26:

- **Torque is capped, never an error.** The 20-tick saturation abort was removed from both trot paths and
  `torque_saturation_limit` no longer exists.
- **Pitch offset**: first documented, not fixed (2026-11-26); on 2026-11-27 Ege had the **attitude integral**
  added (`wbc_attitude_integral_gain = 0.5`, whole-body path only, on by default). Once per gait cycle half of
  that cycle's mean roll and pitch is moved into the MPC's roll/pitch reference, as Kim's released code does
  continuously (`rpy_int`, `rpy_comp`). It holds the mean tilt within about 0.01° at every speed, cadence and
  duty tested. `wbc_attitude_integral_input` (0.01 rad) is its anti-wind-up for pushes. Report Sec. 3.2.
  For any study of the uncompensated controller set the gain to 0 (the harness' `KI_OFF`).
- **Touchdown fixed** for the whole-body path (feet land 1–5 ms early at ≤ 0.14 m/s instead of 21–26 ms early).
- **Flight phases are handled by the whole-body path** (Ege, 2026-11-27). Before, any duty below 0.5 ended in a
  module ERROR at the first flight phase. Report Sec. 3.7 has the duty sweep (harness experiment `duty`, 0.30 to
  0.70 at the 0.3 s period): all runs complete; with flight the whole-body trot pitches nose-up (−1.2° at duty
  0.30, 0.35 m/s; positive pitch is nose down), rides up to 12 mm low and is airborne at most about half the
  scheduled time. Cause found for part of it: the legs (8.3 of 15.2 kg, all knees one way) carry pitch angular
  momentum at push-off that the rigid-body MPC does not see; in flight it moves to the trunk. Sec. 3.7 has the
  numbers and what is still unexplained. The attitude integral now removes the mean; the moments behind it
  remain and show as the MPC's pitch reference (about +2° at duty 0.30).

## Rules for the next session

- **Judge a controller on full ground truth.** `orientationestimator.use_ground_truth` is `true` by default but
  `posvelestimator.use_ground_truth` is `false`. The harness sets both to `true` on every run and keeps two
  filter-in-the-loop runs as a separate comparison. Say which state input every number used.
- One `simulation` process at a time. Headless runs need `< /dev/null`. Always pass
  `supervisor.log.file_name` (`bin/localconf.toml` enables logging into `bin/deneme.mat`).
- Compare logs aligned on the control tick; the log thread starts and stops on wall-clock-dependent ticks.
- A log line holds 127 doubles: `MdlTrot_wbc` does not fit next to the full default variable list.
- **AMB and the attitude integral both act on roll.** The integral moves the MPC's roll reference once per
  cycle on the cycle-mean roll; an AMB term would act within the two-foot phase. Decide early whether the
  integral stays on in AMB runs (it removes a mean, not the rocking) and say so with every result.
- The viewer's red marker and trail are the shadow filter's estimate, drawn on purpose even in ground-truth
  mode; the controller gets truth (checked).

## Measured baselines without AMB

All on ground-truth state, in place unless stated. Report Sec. 4 has the tables.

**How long a two-foot phase survives** (15 s of trot, `wbc_touchdown_depth = 0.008`, one run each, scratch
runs of 2026-11-26, not in `results.json`):

| Two-foot phase | Four-foot phase | Whole-body trot | Old path |
|---|---|---|---|
| 0.50 s | 0 | no fall, roll and pitch within 1.1° | no fall, within 0.7° |
| 0.75 s | 0 | falls 3.0 s after trot entry | falls after 7.9 s |
| 1.0 s | 0 | falls after 7.2 s | falls after 5.7 s |
| 1.5 s | 0 | falls after 12.7 s | falls after 6.3 s |
| 2.0 s | 0 | falls after 14.9 s | falls after 6.3 s |
| 0.50 s | 0.2 s | no fall, peak roll 5°, pitch 4° | not run |
| 1.0 s | 0.2 s | no fall, peak roll 21°, pitch 19° | not run |
| 1.5 s | 0.2 s | no fall, peak roll 17°, pitch 15° | not run |

So without AMB the limit lies between 0.50 and 0.75 s of two-foot support, and a 0.2 s four-foot phase rescues
longer ones only by catching a body that has already tipped 15–20°. This is the regime where AMB can show an
effect. Reproduce with the harness' `overrides()` plus `trot.duty`, `trot.period`, `trot.wbc_touchdown_depth`.

**Other baselines** (report Sec. 4):

- Momentum L about the stance diagonal, in place at the 0.3 s gait: peak 0.011–0.042 kg m²/s, CoM within 3 mm
  of the line. At 0.70 m/s: peak 1.82–1.87 kg m²/s every stance and the CoM sweeps 53–58 mm across the line.
  That large value belongs to the gait: the body moves forward over a diagonal that is oblique to the motion.
- dL/dt = M g c_h holds on two-foot stance (correlation 0.99; slope 0.90–0.96 about the line frozen at stance start).
- Lateral 0.1 s push, in place, 0.3 s gait: recovers from 180–200 N (whole-body) and 140–160 N (old path). The
  boundary is not clean: one case fails at 170 N between successes at 160 and 180–185 N.
- Static diagonal balance (earlier work, centreline posture): without AMB the robot falls 1.2 s after lift-off;
  with AMB it balances and recovers 9–12 N pushes.
- **Touchdown depth depends on swing time.** 3 mm suits the 0.3 s gait; at a 1.0 s period roll rms is 1.40° with
  3 mm and 0.22° with 8 mm. Fix this parameter before comparing anything at slow cadence.

## Three facts that shape the design

1. **No free rows.** A trot stance uses all 18 degrees of freedom (contacts, body orientation, body position,
   swing feet). A new lowest-priority task does nothing. AMB has to take over or add to an existing row, or act
   outside the hierarchy (MPC forces, foothold).
2. **L about the stance line is not an acceleration-level task.** With two point feet dL/dt = M g c_h whatever
   the joint accelerations are, and the QP's floating-base rows enforce that. AMB works one level up, through
   where the CoM goes relative to the line.
3. **Time scale.** With the thesis gains the AMB law's dominant time constant is about 0.28 s (my calculation).
   A 0.15 s stance is shorter than that; a 0.75 s stance is not.

## The seam: the swing foot's tangential direction

Thesis Ch. 6 splits each swing foot's task into three directions measured from its hip, relative to the stance
line: normal (along the line), radial (hip to foot, the leg length), tangential (perpendicular to both: the way
the foot moves when the leg swings about an axis parallel to the line). AMB commands only the tangential one,
as a foot acceleration `ρ θ̈ e_t`, and passes the measured tangential position and velocity through as
references so no PD fights it; normal and radial stay under ordinary PD. Code: `MdlAMB::principalFrame`,
`tangentialArc`, `swingReference`, `tangentialFeedforward` (public statics, not tied to a leg pair).

In the trot the swing feet already have their two rows, so AMB needs no new task: it changes what the swing
tasks ask for along the tangential direction. The conflict is the stride: with the leg hanging down the
tangential direction is horizontal and perpendicular to the diagonal, which lies about 37° from the forward
axis, so about 60 % of a forward step is tangential. In place there is no stride and AMB can own that direction
for most of the swing; moving forward it can only be an addition to the swing trajectory.

## What `MdlAMB` assumes that a trot breaks

- Fixed pair: FL and RR stance, FR and RL swing (`geometry`, `project`, `robotMomentum` use legs {0, 3}).
- Stance feet on the body centreline, reached through `MdlDiagonalStance`. Gains (950, 360, 41) and template
  masses were tuned there, where the tangential direction is pure hip abduction with 4–7 mm of CoM authority.
  On the trot's nominal footprint the geometry and the authority are different and unmeasured.
- Body orientation gains 400/40 were needed there (100/10 let the symmetric swing mode run away); the trot uses 100/10.
- Base state from the two planted feet; the trot uses the estimator modules instead, which is what should stay.
- The law has state (third order in L). L changes sign when the stance diagonal changes, because the line's
  direction does. What the state does at a pair switch is undecided.

## Agreed order (Ege confirmed it on 2026-11-26)

Ege's summary: first test whether AMB holds a trot diagonal stance, without centring the legs; then assess
whether it improves the trot gait at different speeds, trot in place included.

0. Housekeeping before any comparison: add the two-foot-phase sweep to `figs/trot_wbc/run_experiments.py` with
   the gait parametrised by `t_two`, `t_four`, and settle the touchdown depth for slow gaits.
1. **Static check on the trot's own footprint**: from a normal stand lift one diagonal pair and balance with AMB
   on the other, both pairs, no centreline stepping. If AMB cannot hold this, it cannot help a trot.
2. **Trot in place** with long two-foot phases and a short four-foot phase: AMB on the tangential direction
   during the two-foot phase, fading out before touchdown so the foot lands on its target; the four-foot phase
   is where the law's state is handed over or reset. Compare against the table above: longest two-foot phase
   without a fall, peak roll and pitch at 1.0 and 1.5 s, push limit.
3. **Forward at several speeds** (the report's 0.35, 0.70, 1.00 m/s are the baseline): AMB as an addition to the
   swing trajectory, regulating the deviation of L from the gait's own profile, not L itself. Include the
   normal 0.3 s gait, where little effect is expected but the measurement is cheap.
4. Every comparison is AMB on against AMB off on the same gait, on ground-truth state, with the filter runs
   as a separate row.

Comparisons worth keeping in view, neither of which is the thesis' AMB: acting through the MPC's horizontal
forces, and through the foothold.

## Open items carried over

- Why a 9–11 mm position lag remains on ground truth at speed (it drives the pitch offset the attitude integral
  now covers).
- Uncapped torque requests reach hundreds of times the limit in recovered pushes near the boundary (roll above
  27°); the cap makes it harmless, the cause is not examined.
- After a module ERROR the Supervisor's sit can roll the robot over.
- `test_trot_wbc` does not check the body tasks' error signs.
- With both filters in the loop the whole-body trot loses most of its advantage over the old path
  (roll rms 0.49° against 0.50°); the attitude filter drifts 1.5° in yaw over 12 s.
