# Mahony orientation estimator: gain and accelerometer-filter study

Date: 2026-08-19  
Estimator revision: `fec8222a990a9b4ee0d3ef03f26338d8aa36c990`

## Executive findings

- Of the four tested settings, **case D (`kp=0.6`, `ki=0.09`,
  `accel_filter_tau=0.5 s`) is clearly best for roll and pitch**. Its steady-trot
  tilt RMSE is 3.17 mrad, compared with 24.04 mrad for the same Mahony gains
  without acceleration filtering.
- With `ki=0`, low-pass filtering only moves the proportional-loop equilibrium:
  it improves roll but worsens pitch, leaving the combined steady tilt RMSE
  almost unchanged (30.94 -> 30.22 mrad). It cannot estimate or remove gyro bias.
- The integral pair makes the published angular velocity more reasonable on the
  observable axes. In case D, steady rate RMSE falls from 5.69 to 5.00 mrad/s on
  x and from 5.22 to 5.02 mrad/s on y; more importantly, mean rate error falls
  from `[2.72, 1.43]` to `[0.18, -0.23]` mrad/s.
- Yaw remains unobservable from gravity. Every case drifts at about
  1.72--1.86 mrad/s and ends 108--115 mrad away from truth. Neither gain tuning
  nor acceleration filtering can fix this without another heading reference.

## Experiment

Four independent simulations were run sequentially. Each process ran to
simulation time 60 s, logging began at 3 s, and the resulting logs span
56.989--56.997 s at 1 kHz. The same queued input sequence was used in every
process: stand, wait exactly 3,000 controller cycles, then trot. The detected
gait onset differs by only 18 ms across the four logs (2.706--2.724 s in logged
time).

| case | `mahony_kp` | `mahony_ki` | `accel_filter_tau` |
| --- | ---: | ---: | ---: |
| A | 0.1 | 0 | 0.0 s |
| B | 0.1 | 0 | 0.5 s |
| C | 0.6 | 0.09 | 0.0 s |
| D | 0.6 | 0.09 | 0.5 s |

Common settings were `attitude_source="filter"`, `zero_initial_yaw=false`,
`use_ground_truth=false`, and the configured synthetic IMU model: 0.05 m/s^2
accelerometer white noise, 0.005 rad/s gyro white noise, 0.001 m/s^2/sqrt(s)
accelerometer bias walk, 0.0001 rad/s/sqrt(s) gyro bias walk, initial biases of
0.02 m/s^2 and 0.002 rad/s on each axis, and seed 12345.

Two metric windows are reported from each of these same four logs; no additional
or concurrent simulations were used:

- **full log:** startup, standing, gait transition, and steady trot;
- **steady trot:** logged `t >= 9 s`, more than six seconds after gait onset and
  after the trot's preparation and velocity ramp.

Orientation error is the wrapped per-axis RPY difference. `tilt` is
`sqrt(roll_RMSE^2 + pitch_RMSE^2)`, useful here because errors are small; it is
not a quaternion geodesic metric. Body-rate truth is shifted back by one 1 ms
sample before comparison because `MdlSimDriver` updates its logged `qvel` after
the orientation estimator has consumed that cycle's IMU sample.

## Orientation RMSE

All values are in **mrad**.

### Full log

| case | roll | pitch | tilt | yaw |
| --- | ---: | ---: | ---: | ---: |
| A | 28.101 | 6.342 | 28.808 | 67.693 |
| B | 24.870 | 14.892 | 28.988 | 66.576 |
| C | 4.737 | 22.388 | 22.884 | 70.269 |
| D | **2.946** | **8.893** | **9.368** | 69.049 |

### Steady trot (`t >= 9 s`)

| case | roll | pitch | tilt | yaw |
| --- | ---: | ---: | ---: | ---: |
| A | 30.274 | 6.375 | 30.937 | 73.418 |
| B | 26.716 | 14.118 | 30.217 | 72.197 |
| C | 4.717 | 23.572 | 24.039 | 76.226 |
| D | **2.736** | **1.595** | **3.167** | 74.898 |

The full-log pitch RMSE of case D is dominated by the start-up transient visible
in its figure. Once trotting is established, filtering the accelerometer reduces
pitch RMSE by 93.2% and combined tilt RMSE by 86.8% relative to case C. Relative
to the filtered but non-integral case B, case D reduces steady tilt RMSE by 89.5%.

## Angular-velocity RMSE

All values are in **mrad/s** and are computed in the body frame. `raw` is
reconstructed exactly as `omegaBody + gyroBias`; `corrected` is the module's
published `omegaBody = gyro - gyroBias`.

### Full log

| case | x raw | x corrected | y raw | y corrected | z raw | z corrected |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A | 5.628 | 5.628 | 5.240 | 5.240 | 5.305 | 5.305 |
| B | 5.628 | 5.628 | 5.240 | 5.240 | 5.305 | 5.305 |
| C | 5.628 | 5.034 | 5.240 | **5.115** | 5.305 | 5.350 |
| D | 5.629 | **5.010** | 5.240 | 5.191 | 5.305 | 5.338 |

### Steady trot (`t >= 9 s`)

| case | x raw | x corrected | y raw | y corrected | z raw | z corrected |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A | 5.690 | 5.690 | 5.216 | 5.216 | 5.285 | 5.285 |
| B | 5.690 | 5.690 | 5.216 | 5.216 | 5.286 | 5.286 |
| C | 5.690 | 5.029 | 5.216 | 5.089 | 5.285 | 5.333 |
| D | 5.690 | **5.002** | 5.216 | **5.019** | 5.285 | 5.319 |

The nearly identical raw rows are a useful experiment check: all cases received
the same simulated motion and deterministic noise realization. In case D,
integral correction improves steady x rate RMSE by 12.1% and y by 3.8%. White
noise is 5 mrad/s, so it dominates RMSE and makes these percentages look modest.
The mean error shows the bias correction more directly:

| case | corrected mean x | corrected mean y | corrected mean z |
| --- | ---: | ---: | ---: |
| A | 2.719 | 1.431 | 1.716 |
| B | 2.719 | 1.431 | 1.716 |
| C | 0.219 | 0.628 | 1.858 |
| D | **0.182** | **-0.228** | 1.817 |

The common raw mean is approximately `[2.719, 1.431, 1.715]` mrad/s. Cases A
and B cannot change it because `ki=0` keeps `gyroBias` exactly zero. Case D
removes almost all observable x/y DC error. The z correction slightly worsens
RMSE (0.6%) because gravity supplies no heading information and the small learned
body-z bias is not a reliable yaw-bias estimate.

## Figures

| case | orientation | body angular velocity |
| --- | --- | --- |
| A | [orientation](mahony_accel_filter_comparison/body_orientation_case_a_kp0p1_ki0_tau0.png) | [rate](mahony_accel_filter_comparison/body_angular_velocity_case_a_kp0p1_ki0_tau0.png) |
| B | [orientation](mahony_accel_filter_comparison/body_orientation_case_b_kp0p1_ki0_tau0p5.png) | [rate](mahony_accel_filter_comparison/body_angular_velocity_case_b_kp0p1_ki0_tau0p5.png) |
| C | [orientation](mahony_accel_filter_comparison/body_orientation_case_c_kp0p6_ki0p09_tau0.png) | [rate](mahony_accel_filter_comparison/body_angular_velocity_case_c_kp0p6_ki0p09_tau0.png) |
| D | [orientation](mahony_accel_filter_comparison/body_orientation_case_d_kp0p6_ki0p09_tau0p5.png) | [rate](mahony_accel_filter_comparison/body_angular_velocity_case_d_kp0p6_ki0p09_tau0p5.png) |

## Estimator behavior

### Why the accelerometer low-pass matters

With `tau=0`, the Mahony correction sees the trot's translational acceleration
and footfall content as if it were a tilted gravity vector. Case C shows the
failure clearly: the integral loop learns a false y-axis correction and holds a
-23.47 mrad mean pitch error during steady trot. Adding the 0.5 s low-pass in
case D averages approximately one gait period, reducing the steady mean pitch
error to -1.33 mrad and its oscillatory standard deviation from 2.20 to
0.88 mrad.

Filtering alone is insufficient. In cases A and B the constant gyro bias must be
balanced by a standing proportional attitude error of roughly `bias / kp`.
Changing `tau` changes which body-axis error supplies that balance: roll improves
while pitch worsens, and combined tilt barely changes.

### Why angular velocity must be checked

The acceleration correction has two distinct paths in the implementation:

1. `mahony_kp * wcorr` is added only to the quaternion propagation rate. It is an
   attitude innovation, not a physical angular-velocity measurement.
2. `mahony_ki` integrates that tilt innovation into `gyroBias`. The module then
   publishes `omegaBody = gyro - gyroBias`.

Consequently, good-looking orientation does **not** prove that the exported rate
is good. With `ki=0`, acceleration can pull roll and pitch toward gravity while
the angular-velocity output remains the raw biased gyro; cases A and B demonstrate
this exactly. The rate also matters outside the orientation plot: the position
estimator uses it in the foot transport term `omega x p_rel`. A residual rate
bias therefore becomes a body-velocity bias of approximately rate error times
leg length and then integrates into position drift.

The plotted rates are physically reasonable in amplitude and phase: simulator,
raw gyro, and corrected gyro overlap through approximately +/-0.45 rad/s gait
motion after the one-sample alignment. The RMSE and mean-error annotations are
needed because the milliradian-per-second bias is visually hidden by that motion.

### Yaw behavior

The correction vector is formed from measured and estimated gravity. Rotating
an attitude about gravity changes neither vector, so heading and its bias remain
unobservable. The measured yaw-error slopes are 1.72--1.86 mrad/s, consistent
with the injected z gyro bias and random walk. A magnetometer, visual heading,
motion-capture heading, or another independent yaw reference is required if
bounded yaw is a requirement.

## Implementation gaps and limitations

1. **The Mahony path has no focused unit tests.** Existing estimator tests cover
   yaw zeroing, frame transforms, passthrough angular velocity, and the downstream
   `omega x p` term, but not filter convergence, correction signs, `tau=0`, bias
   learning, dynamic-acceleration rejection, or yaw unobservability.
2. **Filter mode still initializes from the reported quaternion.** This is
   convenient in simulation but gives the filter one perfect ground-truth
   attitude sample. It also conflicts with the documented use case of an IMU
   whose attitude is absent or untrusted; roll/pitch should be initializable from
   gravity and yaw from an explicit chosen datum.
3. **Important diagnostics are not logged.** The true injected gyro bias,
   filtered acceleration, `wcorr`, and correction gain are absent. Raw gyro can
   be reconstructed from existing fields, but true gyro-bias RMSE cannot be
   measured directly. Case C's false bias can only be inferred from attitude and
   corrected-rate behavior.
4. **The integral state has no clamp, leakage, or explicit observability
   projection.** Case C demonstrates that periodic linear acceleration can drive
   it to a wrong value. The low-pass is effective for this trot, but robustness
   under sustained acceleration, a different gait frequency, impacts, falls, or
   aggressive body rotation remains untested.
5. **Yaw-axis wording is too strong.** The code comments say the gyro-bias yaw
   component is always zero. The innovation is perpendicular to the instantaneous
   gravity vector, not necessarily to body z, and the logs contain small nonzero
   z bias estimates. Global yaw is still unobservable, but body-axis cross-coupling
   can move this component and slightly degrade the published z rate.
6. **Logs contain no estimator-configuration provenance.** Parameterized file
   names and recorded commands were required to associate each log with its
   gains. Gains, noise seed, binary revision, and run events should be stored in
   future experiment artifacts.
7. **This is one deterministic seed and one steady trot.** It is a controlled
   comparison, not a confidence interval or hardware qualification. Multiple
   seeds, stationary bias-convergence tests, commanded turns, sustained linear
   acceleration, other gait frequencies, and real IMU logs are still needed.
8. **A public API comment is stale.** `step(..., dt)` documents `dt` as used only
   by synthetic bias walk, although it is also the attitude propagation and bias
   integration timestep.

## Recommendation

Keep `mahony_kp=0.6`, `mahony_ki=0.09`, and `accel_filter_tau=0.5 s` as the best
of these four settings for the current 0.5 s trot. Continue plotting both
orientation and angular velocity: use orientation RMSE to judge gravity tracking,
and use time-aligned rate RMSE plus mean error to judge whether gyro-bias
correction is genuinely improving the signal consumed downstream.

Before calling the estimator hardware-ready, add deterministic Mahony unit tests
and the missing diagnostic/provenance fields, then repeat the study over several
seeds and motions. Treat yaw as an explicit separate requirement rather than a
gain-tuning problem.
