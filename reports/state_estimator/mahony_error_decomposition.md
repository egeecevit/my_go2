# Mahony orientation estimator: why the per-axis errors fall where they do

Date: 2026-08-19  
Estimator revision: `fec8222a990a9b4ee0d3ef03f26338d8aa36c990`

Companion to [`mahony_accel_filter_comparison.md`](mahony_accel_filter_comparison.md).
That report measured four gain settings; this one accounts for every number in
its two orientation tables from the filter's own equations. No new simulations
were run. All values are in **mrad** unless marked otherwise, and all quoted
measurements are the steady-trot (`t >= 9 s`) window of the four logs described
there.

## The four observations this report explains

1. Case A has a large constant roll offset but pitch close to truth.
2. Case B has the *same* roll as A, yet pitch develops a constant offset. Adding
   the accelerometer low-pass was supposed to help; pitch got worse.
3. Case C has good roll but a large pitch offset.
4. Case D is good on both.
5. The angular-velocity plots look equally good in all four cases.

None of these is a coincidence, a tuning accident, or a bug. Each follows from a
two-term error model with one term switched by `mahony_ki` and the other by
`accel_filter_tau`, and the four cases are the 2x2 factorial of those switches.

| case | `mahony_kp` | `mahony_ki` | `accel_filter_tau` | bias term | reference term |
| --- | ---: | ---: | ---: | --- | --- |
| A | 0.1 | 0 | 0.0 s | present | present |
| B | 0.1 | 0 | 0.5 s | present | suppressed |
| C | 0.6 | 0.09 | 0.0 s | removed | present |
| D | 0.6 | 0.09 | 0.5 s | removed | suppressed |

## The error model

Notation, all per horizontal body axis and all small-angle:

| symbol | meaning |
| --- | --- |
| `θ` | attitude error of the estimate, truth to estimate |
| `b` | true gyroscope bias on that axis, as realized in the run |
| `b̂` | `_gyroBias`, the integrator state |
| `g̃` | `_gain`, the magnitude gate, in [0, 1] |
| `ρ` | **attitude error implied by the gravity reference itself** — the attitude a body would be at if its measured up really were the direction of `_accFilt` |

`ρ` is the key definition and it is worth stating precisely, because the sign
depends on it. It is not "the angle by which the reference must be rotated"; it
is the attitude *the reference asserts*, minus the truth, read in exactly the
same sense as `θ`. With that reading, and `MdlOrientationEstimator.cc:135-136`

```cpp
const Eigen::Vector3d upBody = _q.conjugate() * Eigen::Vector3d::UnitZ();
_wcorr = (_accFilt / anorm).cross(upBody);
```

the correction is, to first order,

```
wcorr ≈ ρ − θ
```

which reads exactly as it should: *how far the estimate sits from where the
reference says it should be*. This was checked numerically against the code's
own expressions over a grid of `θ` and `ρ` up to 23 mrad; the first-order form is
exact to better than 2.3e-4 rad, and the entire residual is a z component. That
z residual is itself the cross-coupling documented on `getGyroBias()`: `wcorr` is
perpendicular to the *measured* gravity direction, not to body z, so a tilted
estimate leaks a little correction onto the yaw axis.

Feeding that into the two lines that make up the loop,

```cpp
_gyroBias -= _params.mahony_ki * _gain * _wcorr * dt;                       // :154
const Eigen::Vector3d w = gyro - _gyroBias + _params.mahony_kp * _gain * _wcorr;  // :168
```

and using `gyro = ω_true + b`, the error dynamics per horizontal axis are

```
θ̇ = (b − b̂) + kp·g̃·(ρ − θ)
b̂̇ = ki·g̃·(θ − ρ)
```

(The full vector form carries an extra `θ × ω` term from differentiating the
attitude error through a rotating body. It averages to zero at DC for a gait
that is symmetric fore and aft, and it is one of the second-order effects
discussed under *Where the closure is not exact* below.)

## The two equilibria

**With `ki = 0`.** The `ki` line is the *only* write to `_gyroBias` in the whole
module apart from `reset()` and the clamp, so `b̂ ≡ 0` for the entire run — not
"small", not "slow to converge", identically zero. Averaging `θ̇ = 0` over a gait
cycle:

```
θ_ss = ρ̄ + b / (kp·⟨g̃⟩),      ρ̄ ≡ ⟨g̃ρ⟩ / ⟨g̃⟩
```

**With `ki > 0`.** The integrator can only stop when `⟨g̃(θ − ρ)⟩ = 0`, which
forces

```
θ_ss = ρ̄        and then, from θ̇ = 0,        b̂ = b
```

So the steady attitude error is the sum of exactly two terms:

```
θ_ss = ρ̄  +  [ki == 0] · b / (kp·⟨g̃⟩)
        │                  │
        │                  └── the BIAS term: present iff ki = 0
        └───────────────────── the REFERENCE term: large iff tau = 0
```

Two terms, two independent switches, four cases. Everything below is bookkeeping
on these two lines.

## Why `ρ̄` is large at `tau = 0`, and why it lands on pitch

`ρ` oscillates through the stride whatever `tau` is; what matters is whether it
has a **mean**. A zero-mean oscillation cannot move the equilibrium. Two
nonlinearities rectify it into one:

1. **The gate is computed from the instantaneous magnitude.** `_gain` is a
   function of `|_accFilt|`, and at `tau = 0` that is the raw specific force,
   whose magnitude is pulled off `g` by footfall impacts and trunk bounce at
   fixed points in the stride. The correction is therefore *phase-selectively
   sampled*: a periodic `g̃` multiplying a periodic `ρ` at the same frequency has
   a nonzero product mean, `⟨g̃ρ⟩ ≠ 0`, even though `⟨ρ⟩ = 0`. This is precisely
   the failure the code comment at `MdlOrientationEstimator.cc:120-126` warns
   about, and it is why gating cannot substitute for filtering.
2. **`wcorr` is bilinear.** `â × upBody` is a product of two signals that are
   both phase-locked to the gait — the apparent gravity direction and the
   estimate's own oscillating attitude. A product of two sinusoids at the same
   frequency has a DC component set by their relative phase.

Both mechanisms are *quadratic* in the oscillation amplitude, which is the
lever the low-pass pulls. At `tau = 0.5 s` the first-order filter attenuates the
2 Hz surge by `1/sqrt(1 + (2π·2·0.5)²) = 1/6.36`, so a quadratic rectification
should fall by roughly 40x. Measured, the steady mean pitch error falls from
−23.47 mrad in case C to −1.33 mrad in case D, a factor of 17.6 — the same
order, the shortfall being that footfall ringing sits well above 2 Hz and is
attenuated more while some content sits below and is attenuated less.

**The axis.** The trot's dominant translational acceleration is the trunk's
fore-and-aft surge at twice the gait frequency, roughly 0.2 g on this robot. That
is along body x, so the apparent-gravity vector swings in the **x–z plane**,
which is a rotation about body **y**. The rectified DC therefore lands almost
entirely on **pitch**. Lateral sway is the smaller motion and puts correspondingly
little into roll. Case C measures this directly, because with `ki > 0` its
equilibrium *is* `ρ̄`:

| | roll | pitch |
| --- | ---: | ---: |
| case C steady RMSE | 4.717 | 23.572 |
| case C steady mean pitch | — | −23.47 |

Pitch is nearly pure DC (mean and RMSE agree to 0.4%). Roll's 4.7 is mostly
gait-band oscillation rather than offset, i.e. `ρ̄_roll ≈ 0` while `ρ_roll(t)` is
not. That asymmetry is the whole reason roll and pitch behave differently across
the four cases.

## Case-by-case closure

The realized gyro bias is read off the comparison report's corrected-mean table
for cases A and B, which — because `b̂ ≡ 0` there — is the raw gyro bias itself:

```
b_x = 2.719 mrad/s      b_y = 1.431 mrad/s
```

These are **unequal**, and the inequality is not a modelling choice: it is the
particular realization of the configured 0.0001 rad/s/sqrt(s) bias random walk
under seed 12345 starting from a common 2.0 mrad/s. Roll and pitch differ in
these cases partly because their biases differ by a factor of 1.9.

`kp = 0.1` in A and B, so the bias term is worth `1/kp = 10 s` of amplification:
2.7 mrad/s of rate becomes 27 mrad of tilt.

### Case B — the clean cell: pure bias term

`tau = 0.5` suppresses `ρ̄`, and with the reference filtered `|_accFilt| ≈ g` so
`⟨g̃⟩ ≈ 1`. The prediction is bare `b/kp`:

| axis | predicted `b/kp` | measured RMSE | error |
| --- | ---: | ---: | ---: |
| roll | 27.19 | 26.716 | −1.7% |
| pitch | 14.31 | 14.118 | −1.3% |

Both within 2%, with no fitted quantity of any kind. This cell alone establishes
that the model is the right one.

### Case A — the bias term inflated, plus `ρ̄`

At `tau = 0` two things change together: `ρ̄` appears, *and* `⟨g̃⟩` drops below 1
because the unfiltered magnitude spends part of every stride outside the band.
The second effect **scales** the bias term while the first **adds** to it.

Taking `ρ̄_roll ≈ 0` (established from case C above), the roll cell determines
`⟨g̃⟩` and nothing else:

```
θ_roll = b_x / (kp·⟨g̃⟩) = 30.274  →  ⟨g̃⟩ ≈ 0.90
```

Carrying that single number to pitch — where it is now a prediction, not a fit:

```
b_y / (kp·⟨g̃⟩) = 15.94        ρ̄_pitch (from case C) = −23.47
predicted mean θ_pitch = 15.94 − 23.47 = −7.5 mrad
measured pitch RMSE    = 6.375 mrad
```

The bias term alone would have given **+15.9**; the measurement is **6.4**. The
reference corruption has cancelled roughly 60% of the bias error. **Case A's good
pitch is an accidental cancellation between two large errors of opposite sign,
not accuracy.**

### Case C — the reference term alone

`ki > 0` removes the bias term outright (and the comparison report confirms it:
corrected mean rate error falls from `[2.719, 1.431]` to `[0.219, 0.628]`
mrad/s). What is left is `θ_ss = ρ̄` by construction, so this case does not
predict `ρ̄` — it *measures* it, and supplies the −23.47 used above.

### Case D — both terms removed

`ki > 0` kills the bias term, `tau = 0.5` shrinks the reference term. The
residual 2.736 / 1.595 mrad is the filtered `ρ̄` plus gait-band oscillation:
what the model says is irreducible without a better gravity reference.

### Summary table

| case | bias term | reference term | predicted `θ` | measured RMSE |
| --- | ---: | ---: | ---: | ---: |
| A roll | 30.3 (inflated) | ~0 | 30.3 † | 30.274 |
| A pitch | 15.9 (inflated) | −23.5 | −7.5 | 6.375 |
| B roll | 27.2 | ~0 | 27.2 | 26.716 |
| B pitch | 14.3 | ~0 | 14.3 | 14.118 |
| C roll | 0 | `ρ̄_roll` | — ‡ | 4.717 |
| C pitch | 0 | `ρ̄_pitch` | — ‡ | 23.572 |
| D roll | 0 | filtered `ρ̄` | small | 2.736 |
| D pitch | 0 | filtered `ρ̄` | small | 1.595 |

† this cell is where `⟨g̃⟩ = 0.90` was fitted; it is not an independent prediction.  
‡ case C measures `ρ̄` rather than predicting it.

## The A -> B paradox, answered directly

> Adding the low-pass should improve bias handling. Why did pitch get worse?

**Because the low-pass has nothing to do with bias handling.** It acts on one
term of the two, and with `ki = 0` the other term has no mechanism to improve at
all. Three specific points:

**1. With `ki = 0` there is no bias estimator, so nothing about the bias can
change.** `_gyroBias` is written in exactly one place, and that line is
multiplied by `ki`. The comparison report shows this as directly as it can be
shown: the raw and corrected angular-velocity rows for A and B are **identical to
every printed digit**, in both windows, and their corrected mean errors are the
same `[2.719, 1.431, 1.716]` mrad/s. The low-pass changed the attitude and did
not touch the rate by a single bit. It could not have: no path exists.

**2. On pitch, `ρ̄` had been cancelling the bias error.** Case A's pitch carried
`+15.9` of bias error and `−23.5` of reference corruption. Cleaning the reference
removed the *cancelling* error and left the bias error standing at its full
`b_y/kp = 14.3`. Pitch did not get worse; it stopped being flattered.

**3. Combined tilt is essentially unchanged, which is the tell.** Steady tilt
RMSE is 30.937 (A) against 30.217 (B) — 2.3% apart, and B is the *smaller* one.
No error was created. One error stopped hiding another, and the total attitude
error is what it always was: the bias standing error, which only `ki` can remove.

**Why roll shows nothing.** `ρ̄_roll ≈ 0` with or without the filter, because the
surge that gets rectified is fore-and-aft. So roll displays the full `b_x/kp` in
both cases and the low-pass has almost nothing to remove; the small A-to-B roll
difference (30.3 to 26.7) is the `⟨g̃⟩` inflation going away, not `ρ̄`.

**And why A's pitch was luck rather than accuracy.** The model is linear in `b`,
and `ρ̄` is generated by the gait, entirely independent of the gyroscope. Flip
the sign of the bias realization and case A's pitch becomes `−15.9 − 23.5 = −39`
mrad while case B's stays at 14.3. A's advantage evaporates and reverses. It is
specific to this bias realization, this trot speed, and this heading — the same
category of result as the accelerometer-bias cancellation documented in
`config/default/stateestimator.toml`, where a positive bias flatters the reported
position error by cancelling the gait's foot scrub.

## Why the angular-velocity plots look good in every case

They look good because the quantity that distinguishes the cases is roughly
0.5% of the plot's vertical scale.

| quantity | magnitude |
| --- | ---: |
| gait body-rate swing | ±450 mrad/s |
| gyro white noise (1σ, configured) | 5 mrad/s |
| the bias under discussion | 1.4–2.7 mrad/s |

The bias is a factor of ~200 below the trace amplitude and a factor of ~2 below
the white noise, so it is invisible by eye and it is swamped in RMSE. That is
exactly what the comparison report's rate tables show: RMSE moves from 5.690 to
5.002 mrad/s between the worst and best case — a 12% change in a metric whose
floor is set by noise the estimator cannot touch. **Rate RMSE cannot rank these
four cases, and neither can looking at the plots.**

But that invisible DC is the only thing the two error terms are made of:

- **In attitude**, the loop amplifies it by `1/(kp·⟨g̃⟩) ≈ 11 s` at `kp = 0.1`.
  2.7 mrad/s of rate bias becomes 30 mrad of roll — a factor of 11,000 in
  visibility between the rate plot and the orientation plot, which is why the
  orientation figures separate the cases and the rate figures do not.
- **In position**, it enters the second-stage filter's foot transport term
  `ω × p_rel`. At a 0.27 m leg length, 2.7 mrad/s is 0.73 mm/s of body velocity
  error, common-mode across all four legs, integrating to ~44 mm over a 60 s run.

The discriminating metric is therefore the **mean** rate error, not its RMSE, and
the comparison report already tabulates it: `[2.719, 1.431]` for A and B against
`[0.182, −0.228]` for D. That table is the one to read when judging whether the
bias correction is working; the rate plots are for confirming amplitude and phase
are physically sensible, which they are in all four cases.

## Where the closure is not exact

Stated plainly, because the two-term model is not a fit and its residuals are
informative:

1. **`⟨g̃⟩` is the one fitted quantity.** It was inferred as 0.90 from the A-roll
   cell and then carried to A-pitch. It was not measured, because `_gain` was not
   logged when these four runs were made. It is now (see below), so the next run
   of this study has no free parameters at all.
2. **RMSE is not mean.** The comparison report tabulates orientation RMSE, which
   is `sqrt(mean² + std²)`. Predictions here are for the *mean*, so they can only
   be compared exactly where the DC dominates (cases B, C). The A-pitch cell is
   the loose one: the model predicts a mean of −7.5 and the measurement bounds
   `|mean| ≤ 6.375`, so the prediction overshoots by at least 1.1 mrad.
3. **`ρ̄` is not perfectly transferable between A and C.** Treating it as a
   property of the reference alone ignores two second-order couplings that depend
   on `kp`: the correlation `⟨g̃θ̃⟩` between the gate and the estimate's own
   oscillating error, and the `θ × ω` term dropped from the error dynamics. Both
   are bilinear in gait-frequency signals and both rectify. A and C differ by 6x
   in `kp`, so a few mrad of difference in the effective `ρ̄` is expected and is
   the most likely home of the A-pitch residual above.
4. **One seed, one gait, one speed.** As in the comparison report. The
   decomposition is structural and should hold generally; the *numbers* are one
   realization.

## Making both terms directly measurable

The `MdlOrientationEstimator_filter` log variable added since these runs records
10 doubles per cycle and is now in `supervisor.log.vars` by default:

| index | contents |
| ---: | --- |
| 0–2 | `wcorr` |
| 3 | `gain` |
| 4–6 | `_accFilt` |
| 7–9 | injected true gyro bias |

Every quantity in this report becomes a direct measurement rather than an
inference:

- **`b̂ − b`**, the true bias-estimation error, is `state[13+i] − filter[7+i]`.
  Previously the injected truth was not recorded at all, so case C's false bias
  could only be argued for from attitude behaviour.
- **`ρ`** is the angle between `filter[4:7]` and the true body-frame gravity
  `R_trueᵀ·(0,0,9.81)` built from the logged simulator quaternion. Its mean over a
  steady window is `ρ̄` — the quantity this report had to obtain indirectly from
  case C.
- **`⟨g̃⟩`** is the mean of `filter[3]`, removing the report's only fitted number.
- **`wcorr`** separates "the reference was wrong" from "the correction was not
  applied", which are indistinguishable from the attitude output alone.

`figs/estimation/plot_estimation.py` now renders the first two of these as
`gyro_bias.png` (estimated against injected bias, per axis) and
`filtered_acceleration.png` (filtered reference against the ideal
`R_trueᵀ·(0,0,9.81)`, with the raw specific force overlaid so the low-pass's
effect is visible directly).

## Cross-references to the unit tests

`src/quadruped/tests/test_mahony_filter.cc` pins the two equilibria of this
report deterministically, with no gait and no simulator:

| test | what it pins | measured |
| --- | --- | --- |
| `ki_zero_standing_error` | `θ_ss = b/kp` with `ρ = 0`, `g̃ = 1` | 0.0200044 and 0.0200004 rad against an analytic 0.02 |
| `bias_learning` | `ki > 0` ⇒ `b̂ = b`, `θ_ss = 0` | bias exact to 6 digits, tilt 1.8e-9 rad |
| `dynamic_acceleration_rejection` | `ρ̄ ≈ 0` under the low-pass against a 2 Hz, 2 m/s² disturbance | mean pitch 0.0097 mrad against a 204 mrad raw reference tilt |
| `bias_clamp` | `mahony_bias_limit` bounds `b̂`, and the remainder reverts to the `b/kp` standing tilt | pinned at the limit; tilt exactly `(b − limit)/kp` |
| `yaw_unobservability` | heading integrates `b_z` one-for-one | 0.12 rad over 60 s at 2 mrad/s |

The `ki_zero_standing_error` case is the analytic counterpart of case B: same
equation, same 0.02% agreement, with the gait and the reference corruption
removed so that only the bias term remains.

## Conclusions

1. There are two error terms, not four cases. `θ_ss = ρ̄ + [ki==0]·b/(kp·⟨g̃⟩)`.
2. `mahony_ki` removes the bias term and nothing else. `accel_filter_tau`
   suppresses the reference term and nothing else. Neither substitutes for the
   other, which is why only case D is good on both axes.
3. `ρ̄` lands on pitch because the rectified disturbance is the trunk's
   fore-and-aft surge. This is a property of the gait's geometry, not of the
   gains, and it is why the four cases look so different axis by axis.
4. Case A's good pitch is a cancellation between two errors of opposite sign and
   is specific to this bias realization. Case B is not a regression from it.
5. Angular-velocity plots and rate RMSE cannot rank these cases; mean rate error
   and orientation offset can, because the loop amplifies rate DC by `1/kp`.
6. The recommendation of the comparison report stands unchanged:
   `mahony_kp = 0.6`, `mahony_ki = 0.09`, `accel_filter_tau = 0.5 s`, which is
   the only one of the four that switches both terms off.
