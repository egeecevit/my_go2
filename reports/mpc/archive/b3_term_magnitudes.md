# B3 — Magnitude of base-motion terms neglected by the fixed-base swing feedforward

> **Superseded in part.** [`reports/mpc/swing_gain_audit.md`](swing_gain_audit.md)
> disproves the framing that motivates this report — that the neglected
> fixed-base terms explain the measured swing lag via the missing
> `swing_feedforward_scale` term. Integrating the actual closed-loop error
> dynamics predicts +6 mm of longitudinal lag against the +46 mm measured,
> while correctly predicting y and z; the dominant cause is the swing loop's
> gain/bandwidth (tuned near resonance with the swing's own fundamental), not
> a missing feedforward term. The term-magnitude measurements below remain
> valid; only the feedforward causal story built on top of them is withdrawn.

Read-only measurement. No file under `src/`, `include/`, `config/`, or `bin/`
was modified, and no simulation was run for this report — all numbers come
from existing logs in `runs/stageA/*.mat`, processed by
`runs/stageA/base_motion_terms.py`.

Yardstick used throughout: `lag [mm] = |a| / omega_n^2 * 1000`, with
`omega_n = 28 rad/s` (`config/default/trot.toml: swing_natural_frequency`),
so `omega_n^2 = 784`. The lag we are chasing is 44 mm at 0.55 m/s and 27 mm
at 0.30 m/s (against mean `|a_ref|` of 21.2 and 15.6 m/s^2 respectively).

## qvel convention — verified, not assumed

Checked by comparing a Savitzky-Golay numerical derivative (window 11,
`delta=0.002`) of the logged state against two competing hypotheses, per
cell:

- **Linear part.** Hyp A: `qvel[0:3]` is already the world-frame base
  velocity, so it should equal `d(pos_world)/dt` directly. Hyp B: it is a
  body-frame velocity, so `R_wb @ qvel[0:3]` should equal `d(pos_world)/dt`.

  | cell | hyp A median\|err\| | hyp B median\|err\| |
  |---|---|---|
  | vx0.30 | 0.0003 m/s | 0.0065 m/s |
  | vx0.55 | 0.0005 m/s | 0.0169 m/s |
  | vx1.00 (pre-fall) | 0.0058 m/s | 0.2379 m/s |

  Hyp A wins by 1–2 orders of magnitude in every cell. **`qvel[0:3]` is the
  base's WORLD-frame linear velocity**, as documented.

- **Angular part.** Hyp C: `qvel[3:6]` is body-local, so
  `qdot = 0.5 * q ⊗ [0,omega]` (right-multiplication). Hyp D: it is
  world-frame, so `qdot = 0.5 * [0,omega] ⊗ q` (left-multiplication).
  Quaternion sign was made continuous (double-cover) before differentiating.

  | cell | hyp C median\|err\| | hyp D median\|err\| |
  |---|---|---|
  | vx0.30 | 0.00182 | 0.00235 |
  | vx0.55 | 0.00221 | 0.00326 |
  | vx1.00 (pre-fall) | 0.02951 | 1.00499 |

  Hyp C wins in every cell (decisively in the pre-fall cell, where angular
  rates are largest and the hypotheses diverge most). **`qvel[3:6]` is
  BODY-LOCAL angular velocity**, as documented. Both conventions match what
  was stated in the brief — nothing to correct.

## Windows used

- `env_vx0.30_yaw0.0`, `env_vx0.55_yaw0.0`: steady window `t in (20, 110) s`.
- `env_vx1.00_yaw0.0`: this cell falls early. Trot starts at `t=0.804 s`
  (first contact-schedule transition). Applying the same kinematic-fall
  criterion `analyze.py` uses (height < 0.20 m or |roll| or |pitch| > 60 deg,
  restricted to the trot-active window) gives first fall sample at
  `t=4.492 s`. **Window used: `t in (0.804, 4.492) s`** — only 3.7 s / ~1–2
  gait cycles before the fall. This cell's numbers should be read as a
  high-dynamics/transient sample, not a steady-state one.

Sanity check on `a_base` (mean over the *whole* window, not just swing
samples): x,y ≈ 0 in the two steady cells (≤ 0.011 m/s²), confirming no
frame or differentiation bug that would show up as a spurious net forward
acceleration. The pre-fall cell shows x=0.036, y=0.108 m/s² — non-zero, but
expected: it's a 3.7 s window entering an actual fall, not many full gait
cycles of steady locomotion, so it doesn't average out. z is small negative
in all three (−0.006 to −0.088 m/s²).

## Differentiation-sensitivity statement

Terms that require differentiating a signal (`a_base` needs `d(v_world)/dt`,
`alpha x r` needs `d(omega)/dt`) were computed with Savitzky-Golay windows of
5, 11, and 21 samples (10/22/42 ms), polyorder 2, `deriv=1`, `delta=0.002`.
Terms that use raw logged signals only (`omega x (omega x r)`,
`2 omega x v_rel`) do not depend on the window at all — confirmed
numerically identical (to the last digit shown) across all three windows in
every cell, as expected.

Percent change from window=5 to window=21 (the widest/most-smoothed vs.
narrowest/least-smoothed estimate):

| term | vx0.30 mean / p95 | vx0.55 mean / p95 | vx1.00 (pre-fall) mean / p95 |
|---|---|---|---|
| a_base | 16.7% / 28.1% | 14.0% / 22.6% | 14.5% / 11.2% |
| alpha x r | 18.8% / 39.2% | 18.3% / 32.7% | 42.1% / 56.8% |
| SUM | 24.9% / 41.2% | 21.7% / 33.0% | 39.8% / 48.8% |

**`alpha x r` (and therefore `SUM`, which it dominates) fails the ~30%
stability bar on p95 in every cell, and on mean as well in the pre-fall
cell.** Its p95 numbers below should be read as upper bounds shaped
significantly by differentiation noise, not as a converged estimate — the
window=21 value is the more trustworthy (least-noise-amplified) one, and
that is what is used for the magnitude judgement in the Verdict. `a_base`'s
mean is comparatively stable (14–17% change); its p95 is borderline (11–28%,
under the 30% bar but not by much). `omega x (omega x r)` and
`2 omega x v_rel` are exactly window-independent, so those numbers are not
differentiation artifacts at all.

## Results — aggregated over 4 legs, per cell

Values are mean/p95 of `|term|` over swing samples in the stated window;
"lag" columns are that magnitude divided by 784, in mm. Reported at all
three windows so the sensitivity above is traceable to these numbers.

### env_vx0.30_yaw0.0 (steady, t in 20..110 s), |a_ref| mean=15.672, p95=19.886 m/s²

| term | win | mean\|.\| (m/s²) | p95\|.\| | lag mean (mm) | lag p95 (mm) |
|---|---|---|---|---|---|
| a_base | 5 | 1.113 | 4.096 | 1.42 | 5.22 |
| a_base | 11 | 1.047 | 3.713 | 1.34 | 4.74 |
| a_base | 21 | 0.927 | 2.945 | 1.18 | 3.76 |
| alpha x r | 5 | 2.662 | 13.222 | 3.40 | 16.87 |
| alpha x r | 11 | 2.460 | 10.863 | 3.14 | 13.86 |
| alpha x r | 21 | 2.163 | 8.042 | 2.76 | 10.26 |
| omega x (omega x r) | any | 0.023 | 0.088 | 0.03 | 0.11 |
| 2 omega x v_rel | any | 0.208 | 0.354 | 0.27 | 0.45 |
| SUM of all four | 5 | 2.619 | 10.864 | 3.34 | 13.86 |
| SUM of all four | 11 | 2.343 | 8.870 | 2.99 | 11.31 |
| SUM of all four | 21 | 1.968 | 6.386 | 2.51 | 8.14 |

Signed mean (win=21, most trustworthy), m/s²: `a_base` = [−0.0019, 0.0000,
−0.0054]; `SUM` = [−0.0235, 0.0002, −0.0017].

### env_vx0.55_yaw0.0 (steady, t in 20..110 s), |a_ref| mean=21.337, p95=27.960 m/s²

| term | win | mean\|.\| (m/s²) | p95\|.\| | lag mean (mm) | lag p95 (mm) |
|---|---|---|---|---|---|
| a_base | 5 | 1.363 | 4.469 | 1.74 | 5.70 |
| a_base | 11 | 1.289 | 4.018 | 1.64 | 5.13 |
| a_base | 21 | 1.172 | 3.460 | 1.49 | 4.41 |
| alpha x r | 5 | 4.107 | 17.984 | 5.24 | 22.94 |
| alpha x r | 11 | 3.850 | 16.009 | 4.91 | 20.42 |
| alpha x r | 21 | 3.354 | 12.099 | 4.28 | 15.43 |
| omega x (omega x r) | any | 0.050 | 0.214 | 0.06 | 0.27 |
| 2 omega x v_rel | any | 0.388 | 0.515 | 0.50 | 0.66 |
| SUM of all four | 5 | 3.837 | 14.585 | 4.89 | 18.60 |
| SUM of all four | 11 | 3.539 | 13.527 | 4.51 | 17.25 |
| SUM of all four | 21 | 3.003 | 9.764 | 3.83 | 12.45 |

Signed mean (win=21), m/s²: `a_base` = [−0.0051, 0.0000, −0.0102]; `SUM` =
[−0.0220, −0.0001, −0.0112].

### env_vx1.00_yaw0.0 (pre-fall, t in 0.804..4.492 s), |a_ref| mean=21.778, p95=44.409 m/s²

| term | win | mean\|.\| (m/s²) | p95\|.\| | lag mean (mm) | lag p95 (mm) |
|---|---|---|---|---|---|
| a_base | 5 | 2.410 | 5.941 | 3.07 | 7.58 |
| a_base | 11 | 2.263 | 5.702 | 2.89 | 7.27 |
| a_base | 21 | 2.061 | 5.273 | 2.63 | 6.73 |
| alpha x r | 5 | 6.409 | 24.633 | 8.17 | 31.42 |
| alpha x r | 11 | 4.592 | 15.699 | 5.86 | 20.02 |
| alpha x r | 21 | 3.708 | 10.634 | 4.73 | 13.56 |
| omega x (omega x r) | any | 0.261 | 1.108 | 0.33 | 1.41 |
| 2 omega x v_rel | any | 1.112 | 3.878 | 1.42 | 4.95 |
| SUM of all four | 5 | 6.970 | 24.444 | 8.89 | 31.18 |
| SUM of all four | 11 | 5.136 | 16.308 | 6.55 | 20.80 |
| SUM of all four | 21 | 4.197 | 12.520 | 5.35 | 15.97 |

Signed mean (win=21), m/s²: `a_base` = [0.0384, 0.1099, −0.0875]; `SUM` =
[−0.0169, −0.1316, −0.2546]. (Only 3.7 s / ~1–2 cycles — least averaged of
the three cells, treat this cell's signed means as noisier than the two
steady cells'.)

## Per-leg table — env_vx0.55_yaw0.0, savgol window = 11

| leg | term | mean\|.\| (m/s²) | p95\|.\| | lag mean (mm) | lag p95 (mm) |
|---|---|---|---|---|---|
| FL | a_base | 1.290 | 4.019 | 1.65 | 5.13 |
| FL | alpha x r | 3.809 | 15.789 | 4.86 | 20.14 |
| FL | omega x (omega x r) | 0.049 | 0.206 | 0.06 | 0.26 |
| FL | 2 omega x v_rel | 0.386 | 0.497 | 0.49 | 0.63 |
| FL | SUM | 3.754 | 13.710 | 4.79 | 17.49 |
| FR | a_base | 1.288 | 4.018 | 1.64 | 5.13 |
| FR | alpha x r | 3.813 | 15.823 | 4.86 | 20.18 |
| FR | omega x (omega x r) | 0.049 | 0.205 | 0.06 | 0.26 |
| FR | 2 omega x v_rel | 0.387 | 0.499 | 0.49 | 0.64 |
| FR | SUM | 3.756 | 13.750 | 4.79 | 17.54 |
| RL | a_base | 1.288 | 4.018 | 1.64 | 5.13 |
| RL | alpha x r | 3.892 | 16.093 | 4.96 | 20.53 |
| RL | omega x (omega x r) | 0.051 | 0.216 | 0.07 | 0.28 |
| RL | 2 omega x v_rel | 0.390 | 0.521 | 0.50 | 0.67 |
| RL | SUM | 3.326 | 13.354 | 4.24 | 17.03 |
| RR | a_base | 1.290 | 4.019 | 1.65 | 5.13 |
| RR | alpha x r | 3.886 | 16.074 | 4.96 | 20.50 |
| RR | omega x (omega x r) | 0.051 | 0.217 | 0.07 | 0.28 |
| RR | 2 omega x v_rel | 0.390 | 0.519 | 0.50 | 0.66 |
| RR | SUM | 3.321 | 13.335 | 4.24 | 17.01 |

`a_base` is essentially leg-independent, as expected (it does not depend on
`r`). `alpha x r` and `SUM` differ slightly front (FL/FR) vs. rear (RL/RR)
because the lever arm `r` (foot position relative to base) differs; left vs.
right is symmetric to within noise.

## Verdict

**Which term(s) exceed ~2 mm of equivalent foot lag:** `alpha x r` is the
only term that clears the ~2 mm bar robustly. Even at the most-smoothed,
least-noise-amplified window (21 samples) — the conservative end of a term
whose p95 fails the 30%-stability bar — its *mean* magnitude alone is 2.76
mm (0.30 m/s), 4.28 mm (0.55 m/s), and 4.73 mm (1.00 m/s, pre-fall); p95 is
higher still (10–14 mm). `SUM`, which `alpha x r` dominates, tracks it:
2.51–5.35 mm mean, 8–16 mm p95. `a_base` sits at or just under the bar on
mean (1.2–1.7 mm in the two steady cells, rising to 2.6 mm in the pre-fall
cell) and clears it on p95 (3.8–7.3 mm). `omega x (omega x r)` never
approaches the bar (≤0.33 mm mean, ≤1.4 mm p95 in any cell).
`2 omega x v_rel` stays under the bar in both steady cells (≤0.5 mm mean,
≤0.66 mm p95) and only reaches it at p95 in the pre-fall cell (mean 1.42 mm,
p95 4.95 mm) — a transient, not a steady-operation number.

**Does any term have a nonzero mean (systematic), rather than being
zero-mean oscillation:** No. The signed component means of both `a_base`
and `SUM` are small in every axis and every cell — at most ≈0.11 mm
equivalent (the pre-fall cell's y and z components of `SUM`, 0.13–0.32
m/s²) — one to two orders of magnitude below those same terms' own
oscillatory (unsigned) magnitude of several mm of equivalent lag. In the two
steady cells the signed means are even smaller (≤0.04 mm equivalent). All
four terms, including the dominant `alpha x r`, average out over a gait
cycle to a magnitude far too small to read as a constant/systematic offset;
what they produce, at the magnitude reported above, is oscillation
(jitter) synchronized with the gait, not a steady bias of the kind the 44
mm/27 mm measured lag would require.
