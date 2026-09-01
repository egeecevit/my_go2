# Stage A: envelope and horizon measurement with the torque tripwire disabled

> **Superseded in part.** [`reports/mpc/swing_gain_audit.md`](swing_gain_audit.md)
> disproves this report's causal attribution of the systematic longitudinal
> swing lag (Section "Swing-lag prediction vs. measurement") to the disabled
> `swing_feedforward_scale`: the closed-loop error dynamics with the actual
> forcing predict +6 mm against the +46 mm measured here, while correctly
> predicting y and z. The lag is a swing-loop bandwidth problem
> (`swing_natural_frequency = 28` runs near the swing's own 25.1 rad/s
> fundamental), not a missing-feedforward problem. The measurements below are
> unaffected; only that causal claim is withdrawn.

## What was run

Two grids, both headless MuJoCo, both with ground truth attitude and
position/velocity (`orientationestimator.use_ground_truth = true`,
`posvelestimator.use_ground_truth = true`), both with
`trot.torque_saturation_limit = 100000000` (the 20-consecutive-cycle abort
disabled so saturation shows up as a measurement instead of a hard stop), both
requesting `supervisor.autorun_trot_duration = 120` and logging at 500 Hz
(`supervisor.log.period = 2`). No source or config file was modified; every
run used the shipped `trot.toml` gait parameters (period 0.5 s, duty 0.5,
`swing_natural_frequency = 28 rad/s` all axes, `swing_feedforward_scale =
0.0`) and the default `mpc.toml` (`dt = 0.01`, `horizon = 10` except where
swept).

- **Grid 1 (envelope)**: `trot.forward_velocity` in {0.30, 0.55, 1.00, 1.50}
  m/s × `trot.yaw_rate` in {0.0, 0.5} rad/s. 8 cells, named
  `env_vx<V>_yaw<W>`.
- **Grid 2 (horizon)**: `mpc.horizon` in {5, 10, 15, 25} at 0.55 m/s, 0 rad/s.
  4 cells, named `hor_H<N>`.

All 12 runs produced a valid log (`Supervisor: Logging enabled to
<name>.mat with 9 variables` confirmed in every `.log`) and were run strictly
sequentially. Raw `.mat`/`.log` pairs and the analysis scripts
(`analyze.py`, `report_gen.py`, `swing_lag_axis.py`) are in
`/home/arch-ege/metuatlas/quadcontrol/runs/stageA/`; the prior batch's
shipped-tripwire baseline (`torque_saturation_limit = 20`, the "already
known" numbers in the task brief) is archived unchanged in
`runs/stageA_shipped/`.

**Housekeeping note**: `runs/stageA_shipped/` originally also held
`hor_H05.{mat,log}` and `hor_H10.mat` from an earlier horizon sweep run with
the shipped tripwire *active* (`torque_saturation_limit = 20`) — a
legitimate as-shipped-baseline measurement, consistent with the rest of that
archive, not an error. It was superseded for this report's purposes because
Grid 2 here specifically needs the tripwire disabled (the point of Grid 2 is
to see whether the gait survives at each horizon once saturation is no
longer an artificial hard stop), so it was removed rather than kept
alongside the re-measured `hor_H05`/`hor_H10` in `runs/stageA/` to avoid two
files sharing a name under different configs. Both horizon cells were
re-measured under the disabled-tripwire config for Grid 2; no recovery of
the original files is needed. Every number in this report comes from a run
whose console log was checked to contain the intended overrides.

A fall is called from the raw trunk-height trace (`MdlSimDriver_qpos[:,2] <
0.20 m`, restricted to `t < 120 s` so the deliberate end-of-run sit is not
mistaken for a fall), checked for whether it is a **sustained** collapse
(height stays down, one contiguous excursion or many) rather than a
momentary dip that recovers. This check was done independently of, and in
addition to, `analyze.py`'s own window logic, because the console output
alone is actively misleading for these runs (see below). For the five cells
that complete the full run (`env_vx0.30_yaw0.0`, `env_vx0.55_yaw0.0`,
`hor_H10`, `hor_H15`, `hor_H25`), trunk height only drops below 0.20 m once,
at t ≈ 122.4 s — that is the deliberate sit-down after the trot stops, not a
fall, and the `t < 120 s` cutoff excludes it from the fall check by
construction. In all five of those cells `MdlTrot_torquescale` is exactly
1.000 on every logged stance sample and the saturated fraction is exactly
0.000 — none of them ever clips a torque command. None of these five is
described as falling anywhere in this report.

## Envelope results

| cmd (vx, yaw) | fell? | fall time | mean h after fall | achieved vx (mean) | achieved yaw | height | max\|roll\|/\|pitch\| | worst joint at failure |
|---|---|---|---|---|---|---|---|---|
| 0.30, 0.0 | **no** | — | — | 0.278 m/s (92.8%) | 0.0000 rad/s | 0.292 m | 1.9° / 2.4° | never saturates |
| 0.55, 0.0 | **no** | — | — | 0.518 m/s (94.3%) | −0.0000 rad/s | 0.291 m | 3.2° / 3.7° | never saturates |
| 1.00, 0.0 | **yes** | 4.67 s | 0.100 m | n/a (falls inside 10 s warmup) | n/a | n/a | n/a | FR hip, 100.4% of 23.7 N·m, stance |
| 1.50, 0.0 | **yes** | 3.35 s | 0.105 m | n/a | n/a | n/a | n/a | RL abduction, 100.4% of 23.7 N·m, stance |
| 0.30, 0.5 | **yes** | 7.32 s | 0.128 m | n/a | n/a | n/a | n/a | RR abduction, 100.4%, stance |
| 0.55, 0.5 | **yes** | 10.58 s | 0.119 m | n/a | n/a | n/a | n/a | RL abduction, 100.3%, stance |
| 1.00, 0.5 | **yes** | 4.07 s | 0.106 m | n/a | n/a | n/a | n/a | RL abduction, 100.4%, stance |
| 1.50, 0.5 | **yes** | 3.31 s | 0.102 m | n/a | n/a | n/a | n/a | RR abduction, 100.5%, stance |

"Never saturates" means `MdlTrot_torquescale` was exactly 1.0 on every logged
stance sample for the whole run — zero clipped cycles, not just a low
fraction.

Only the two lowest-speed, zero-yaw cells survive. Every cell with either
`vx ≥ 1.0` or `yaw = 0.5` collapses, and collapses *permanently*: trunk
height drops below 0.20 m once and never recovers to standing height again
for the rest of the run (mean post-fall height 0.10–0.13 m vs. 0.29 m
standing). At the two lower-yaw cells (0.30/0.55 m/s, yaw 0.5) the height
trace does cross back above 0.20 m several times (11–13 separate excursions)
before settling into the low-crawl regime — a few bounces, not a recovery to
walking.

**Every single failure has the same signature**: a joint at essentially
exactly 100.3–100.5% of its rated torque limit, always the abduction or hip
joint (the 23.7 N·m joints, never the 45.43 N·m knee), always in stance, in
the cycles immediately preceding the collapse. This is not a marginal,
tripwire-only overshoot — it is the leg physically failing to produce the
torque the controller is asking for.

### Pre-fall statistics for the six failing cells

`analyze.py`'s steady-state window requires 10 s past trot start, so all six
failing cells (which fall in 3.3–10.6 s) report `nan` for the standard
steady-state columns above. The table below instead covers the window from
trot start to the fall itself (computed in `swing_lag_axis.py`,
`pre_fall_stats`), so the run is not simply blank in the report:

| cmd (vx, yaw) | window (s) | n samples | mean vx (achieved) | max \|roll\| | worst-leg saturated fraction (stance) | worst joint |
|---|---|---|---|---|---|---|
| 1.00, 0.0 | 0.80 → 4.67 | 1935 | 0.306 m/s (cmd 1.0) | 77.9° | FL 17.3% (min scale 0.465) | FR hip, 100.4% |
| 1.50, 0.0 | 0.80 → 3.35 | 1274 | 0.416 m/s (cmd 1.5) | 139.0° | FL 17.6% (min scale 0.364) | RL abduction, 100.4% |
| 0.30, 0.5 | 0.80 → 7.32 | 3259 | 0.206 m/s (cmd 0.3) | 92.5° | RR 3.5% (min scale 0.418) | RR abduction, 100.4% |
| 0.55, 0.5 | 0.80 → 10.58 | 4890 | 0.428 m/s (cmd 0.55) | 93.0° | RL 1.1% (min scale 0.387) | RL abduction, 100.4% |
| 1.00, 0.5 | 0.80 → 4.07 | 1634 | 0.360 m/s (cmd 1.0) | 124.5° | RL 17.2% (min scale 0.272) | RR abduction, 100.8% |
| 1.50, 0.5 | 0.80 → 3.31 | 1252 | 0.430 m/s (cmd 1.5) | 145.4° | RL 19.2% (min scale 0.287) | RL abduction, 100.8% |

The achieved speed never gets close to the command in any failing cell
(0.21–0.43 m/s achieved against 1.0–1.5 m/s commanded) — the controller is
already fighting saturation and losing ground before it collapses, not
tripping while cruising at speed. The `max|roll|` values are large (78–145°)
because the window runs up to and including the trunk-height crossing at
0.20 m, i.e. it captures the physical tumble itself, not just the run-up to
it; saturation is already present in multiple legs well before that point in
every case.

## The tripwire question

**The shipped tripwire is not pre-empting a gait that would otherwise
continue.** With the abort disabled, none of the six cells that previously
tripped the 20-cycle torque limiter recovers into a stable walk. In four of
the six, the trunk drops below 0.20 m *before* the old tripwire would even
have fired: `env_vx1.00_yaw0.0` and `env_vx1.50_yaw0.0` (shipped abort at
8.0 s and 7.7 s respectively) are already collapsed by 4.67 s and 3.35 s
under ground truth; `env_vx1.00_yaw0.5` and `env_vx1.50_yaw0.5` (shipped
abort 8.0 s / 7.5 s) collapse at 4.07 s / 3.31 s. The two yaw-0.5 cells at
lower speed live a bit longer than their old abort times (7.32 s vs. 11.2 s
shipped, 10.58 s vs. 15.3 s shipped) but still fall well inside the 120 s
window and never recover.

**The console log is actively misleading once the tripwire is gone.**
`MdlPosVelEstimator`'s periodic print (`trust=...`, `foot pos err=...`) keeps
printing plausible-looking numbers and the run reaches `autorun complete`
at ~120–126 s for every one of the six failing cells, with no `torque
limited` warning anywhere in the log for most of them (a `Leg N force
command rejected` or `Leg N command failed` warning appears only once, right
near the very end, when the supervisor finally tries — and fails — to
execute the sit). Reading the console alone, `env_vx1.50_yaw0.0` looks like
a clean, uneventful 122 s trot; the trunk-height trace shows it has been
lying on the ground below 0.10–0.15 m since 3.35 s. **The kinematic (height)
check is required — the log's own "autorun complete" / "no torque limited"
text is not sufficient evidence of success**, exactly as the task brief
anticipated but more starkly than expected: it is not that the log
under-reports a struggling-but-upright robot, it is silent about a robot
that has been on the ground for the entire run.

So the answer to "does the gait survive above 0.55 m/s, or does it genuinely
fall": **it genuinely falls**, and falls fast — inside the first
gait cycle or two once actuator saturation becomes sustained rather than
transient. The 6–19% torque overshoot that triggers the shipped tripwire is
a symptom of a real loss of control authority, not a conservative false
alarm.

## Swing-lag prediction vs. measurement

The correct test of `lag = a_ref/ω²` is a **within-swing** regression:
pair each sample's instantaneous tracking error with that same sample's
instantaneous reference acceleration (`MdlTrot_footref[9i+6 .. 9i+8]`), not
a regression across the handful of surviving commanded speeds. Regressing
across speeds (as an earlier version of this report did) folds the
per-cell intercept into an apparent slope and is the wrong estimator here —
it is kept below only as a note on why it should not be used.

**Within-swing regression** (t ∈ (20, 110) s, swing samples only,
`env_vx0.30_yaw0.0` and `env_vx0.55_yaw0.0`, the only two cells that survive
long enough to fill this window):

| cell | fitted slope | fitted intercept | predicted slope (1/ω²) |
|---|---|---|---|
| env_vx0.30_yaw0.0 | 1.60 mm per (m/s²) | 8.0 mm | 1.28 mm per (m/s²) |
| env_vx0.55_yaw0.0 | 1.38 mm per (m/s²) | 20.9 mm | 1.28 mm per (m/s²) |

**This confirms the slope law to within 8–25%** — a materially different
conclusion from the across-speed fit's apparent 4.7× miss. The
apparent-mass cancellation `kp = ω²·Λ` is doing what it is supposed to do:
the *acceleration-proportional* part of the swing error scales at close to
the predicted 1/ω² rate. What the across-speed fit was actually measuring
was mostly the intercept (a roughly fixed per-cell offset, see below)
changing between cells, misread as a speed-dependent slope.

**The `a_peak` table this report used originally was wrong.** The formula
`a_peak = 5.7735·v·T_st/T_sw²` is the horizontal quintic component only; the
swing trajectory also carries a raised-cosine vertical clearance term whose
acceleration is roughly 15 m/s² and does **not** scale with speed. Measured
mean |a_ref| during swing (`swing_lag_axis.py`, same t ∈ (20,110) s window,
all four legs pooled) is:

| vx | old (horizontal-only) a_peak | measured mean \|a_ref\| |
|---|---|---|
| 0.30 | 6.9 m/s² | **15.67 m/s²** |
| 0.55 | 12.7 m/s² | **21.34 m/s²** |
| 1.00 | 23.1 m/s² | — (fell before window) |
| 1.50 | 34.6 m/s² | — (fell before window) |

At 0.30 m/s the vertical clearance term is the *majority* of the measured
acceleration (15.7 of 15.67 m/s² measured is consistent with a ~15 m/s²
speed-independent floor plus a small horizontal contribution), which is why
the intercept at low speed is not negligible even though the horizontal
`a_peak` there is small. This makes the true reference acceleration larger
than this report's original table assumed at every speed, which pushes the
*predicted* lag up, not down — the original table was an underestimate of
the mechanism's own scale, independent of whether the fitted slope matches.

### Error decomposition by axis (the real headline)

Longitudinal (x), lateral (y), and vertical (z) components of the swing
tracking error, body frame, signed as `reference − measured`, t ∈ (20,110) s,
swing samples, per leg (`swing_lag_axis.py`):

| cell | leg | mean x | mean y | mean z | rms x | rms y | rms z | x / stride |
|---|---|---|---|---|---|---|---|---|
| vx=0.30 | FL | +26.7 mm | +9.0 mm | +3.1 mm | 30.5 | 13.5 | 17.8 | 35.7% |
| vx=0.30 | FR | +26.7 mm | −9.0 mm | +3.1 mm | 30.5 | 13.5 | 17.8 | 35.7% |
| vx=0.30 | RL | +27.2 mm | +8.9 mm | +6.1 mm | 30.9 | 10.5 | 16.4 | 36.3% |
| vx=0.30 | RR | +27.2 mm | −8.9 mm | +6.1 mm | 30.9 | 10.5 | 16.4 | 36.3% |
| vx=0.55 | FL | +48.8 mm | +10.0 mm | +2.8 mm | 56.6 | 14.9 | 19.5 | 35.5% |
| vx=0.55 | FR | +48.8 mm | −10.0 mm | +2.8 mm | 56.6 | 14.9 | 19.5 | 35.5% |
| vx=0.55 | RL | +44.0 mm | +7.5 mm | +6.7 mm | 51.9 | 9.1 | 16.9 | 32.0% |
| vx=0.55 | RR | +44.0 mm | −7.5 mm | +6.7 mm | 51.9 | 9.1 | 16.9 | 32.0% |

("Stride" here is `vx · T_sw` — the distance the body travels during one
0.25 s swing — 75 mm at 0.30 m/s, 137.5 mm at 0.55 m/s.)

The error is **almost entirely longitudinal**: x is 3–5× the y or z
component in both mean and rms, at every leg, at both speeds. It is
**systematic and one-directional** — every leg's mean x error is positive
(the reference always leads the measured foot; the foot always trails) with
essentially no leg-to-leg or speed-to-speed sign variation, not a zero-mean
tracking noise. The y component is not really an error at all: it is a
fixed, leg-position-dependent offset (+ for the left legs FL/RL, − for the
right legs FR/RR, same magnitude on each side) — an abduction-plane bias
rather than a lag along the direction of motion. And **the size is
consistent as a fraction of stride, not of absolute distance**: 32–36% of
the stride length at both speeds, all four legs. At 0.55 m/s that is a
foot landing roughly **one third of a stride short of its commanded
touchdown point**, at the nominal, non-failing operating point — this is
present in the two cells that otherwise look completely healthy (zero
torque clipping, sub-4° tilt, 92–94% of commanded body speed).

### Verdict

The prediction holds where it is actually testable: the acceleration-
proportional part of the swing error tracks close to the predicted 1/ω²
slope (within 8–25%) once measured the right way (within-swing, not
across-speed). What does not hold is the magnitude of the reference
acceleration this report first assumed (the vertical clearance term was
omitted, understating `a_ref` at every speed) and, more importantly, the
practical picture: swing tracking error is dominated by a large, systematic,
purely-longitudinal, one-third-of-a-stride shortfall that is present even
when nothing is saturating and the gait is otherwise stable. Only two
speeds could be measured; `vx = 1.00` and `1.50` fall before 10 s of trot
elapses, so neither the within-swing slope nor the axis decomposition could
be checked in the range where the effect (and any consequence for stability)
would be largest.

## Horizon sweep (vx = 0.55, yaw = 0.0)

Solve-time statistics recomputed over t ∈ (20,100) s (steady window,
excludes ramp-up/ramp-down and, for H05, the post-fall dragging phase):

| horizon | lookahead | fell? | fall time | achieved vx | height | solve time median / p95 / max (ms) | % > 1 ms | n > 10 ms | qpOASES status |
|---|---|---|---|---|---|---|---|---|---|
| 5 | 0.05 s | **yes** | 4.92 s | n/a | n/a | 0.079 / 0.232 / 1.042 | 0.01% | 0 | 0 everywhere |
| 10 (default) | 0.10 s | no | — | 0.518 m/s | 0.291 m | 0.061 / 0.174 / 0.507 | 0.00% | 0 | 0 everywhere |
| 15 | 0.15 s | no | — | 0.522 m/s | 0.291 m | 0.130 / 0.319 / 10.589 | 0.10% | 5 | 0 everywhere |
| 25 | 0.25 s | no | — | 0.527 m/s | 0.291 m | 0.379 / 2.643 / 8.403 | 21.61% | 0 | 0 everywhere |

Horizon 25's lookahead (0.25 s) equals exactly one full stance duration —
the point the repo's own `mpc.toml` documents as where the single rigid-body
model stops predicting anything useful, because the next touchdown's contact
transient isn't modeled.

**This is the key evidence for the solver-time-vs-model-error question, but
it needs both budgets stated, not one.** `MdlTrot::_solveMPC()` runs
**synchronously inside the 1 kHz module loop** — a slow solve does not
queue or run async, it directly delays that cycle's control output — and
this headless simulator does **not** enforce real time, so none of the
numbers above by themselves say what would happen on hardware.

- Against the **10 ms MPC replan budget** (`mpc.dt = 0.01`): every horizon
  is comfortably inside it. 5 solves out of several tens of thousands
  exceed 10 ms at H15 (a rare outlier, not a pattern) and H25 has zero
  samples over 10 ms at all. By this budget alone, H25 "completes the full
  120 s run cleanly and solver time is not what limits it" — which is what
  an earlier version of this report concluded, and which is still true as
  far as it goes.
- Against the **1 ms actuator/control-loop cycle**: H25 spends **21.6% of
  its solves above 1 ms**, i.e. roughly one cycle in five would blow the
  1 kHz budget outright on real hardware, where there is no headless
  simulator to silently absorb a late solve. H15 crosses 1 ms far less
  often (0.10%) but still not never. H05 and H10 are effectively free by
  either budget (≤0.08 ms median, ≤1.04 ms max).

So: **H25 is not model-limited in this measurement (it completes, and the
10 ms replan deadline is never missed), but it is compute-limited against
the 1 kHz control cycle that this synchronous call sits inside.** "Long
horizons are fine now" is true only against the replan-period budget; it is
not a green light for horizon 25 on hardware without also checking whether
occasionally running a cycle over 1 ms is tolerable there. qpOASES status
was 0 (nominal) on every single solve, at every horizon — no infeasibility,
timeout, or degradation, at any horizon tested.

The one cell that genuinely fails, H5, fails for a reason that has nothing
to do with solve time (its solves are the cheapest of the four, all under
1.05 ms even including the post-fall segment): its 0.05 s lookahead is a
fifth of the 0.25 s stance duration, so the model simply cannot see far
enough ahead to plan a stance. This is a prediction-horizon/model-adequacy
failure, not a computational one — exactly the distinction the horizon
question is asking about, and the data separates the two cleanly: **short
horizon fails on model grounds while being computationally the cheapest of
all four; long horizon (25) does not fail against the replan-period budget,
but is measurably expensive against the tighter 1 kHz control-cycle
budget it runs inside.**

## What the data says

1. **The tripwire is not the problem.** Every cell it aborts is genuinely
   falling, usually before the tripwire would even have fired. Disabling it
   does not recover a working gait anywhere in the tested envelope; it only
   delays the console's acknowledgment of a fall that has already happened,
   for up to two minutes, with a plausible-looking log the entire time.
2. **The envelope boundary is sharp and repeatable**: 0.30 and 0.55 m/s
   straight-line walking survive cleanly (92–94% of commanded speed,
   sub-4° tilt, zero torque clipping ever); 1.00 m/s and 1.50 m/s straight,
   and *any* tested yaw rate (0.5 rad/s) at *any* speed, collapse within
   3–11 s via the same signature: an abduction or hip joint pinned at
   ~100% of its 23.7 N·m limit in stance, with achieved speed already well
   below command before the collapse.
3. **The swing-lag model holds where it can be tested.** Measured the
   correct way (within-swing, error vs. instantaneous reference
   acceleration), the fitted slope is within 8–25% of the predicted 1/ω².
   The real finding is not that the model is wrong but that swing tracking
   at both surviving speeds carries a large, systematic, longitudinal
   shortfall — about one third of a stride, present even with zero
   saturation — that the acceleration-proportional term alone does not
   capture and that this study does not trace to a specific cause. Neither
   this regression nor the axis decomposition could be checked above
   0.55 m/s because the gait does not survive that long.
4. **Long horizons are not model-limited, but H25 is compute-limited.** No
   horizon up to 25 threatens the 10 ms MPC replan deadline (qpOASES status
   0 everywhere, effectively zero samples over 10 ms). But H25 exceeds the
   tighter 1 ms control-cycle budget on about one solve in five, and that
   call runs synchronously inside the real-time loop with no async queue
   and no real-time enforcement in this headless simulator to hide a late
   cycle — so this measurement does not clear H25 for hardware use, only for
   the replan-period accounting. H5's failure is unambiguously a lookahead/
   model problem: it is the cheapest horizon to solve and the one that
   falls.
