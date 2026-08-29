# Filter-attitude frame transport and endurance investigation

Date: 2026-08-29  
Branch: `codex/feedforward-horizon-repair`

## Executive summary

The original rapid nose-down failure with
`orientationestimator.attitude_source = "filter"` was caused by implementation
errors in the attitude estimator, not by an inherent weakness of complementary
filtering:

1. The low-pass acceleration history was stored as body-frame components but
   was not transported when the body frame rotated. During appreciable pitch or
   roll, samples expressed in different bases were averaged together. The stale
   gravity direction then generated a false Mahony correction and false gyro
   bias.
2. The first raw accelerometer sample seeded the entire half-second history.
   The simulated robot is dynamically settling at startup, so this one sample
   could represent roughly `28.8 m/s^2` and imply a gravity pitch about
   `0.60 rad` away from the quaternion datum.
3. `zero_initial_yaw` was applied to the same quaternion that was recursively
   propagated. A fixed output-frame transformation was therefore reapplied on
   every update instead of exactly once.
4. The simulation ground-truth option did not override a simultaneously selected
   filter source, weakening its value as a diagnostic bypass.

These defects have been corrected. With the fully estimated orientation and
position/velocity paths enabled, the revised filter completes 60-second trots
for three tested noise seeds. The former high-gain default, `kp=0.6`, `ki=0.09`,
was also replaced by the closed-loop-stable `kp=0.1`, `ki=0.0025` pair.

A 500-second endurance request does not complete: the repeatable failure occurs
at `282.555 s`. A targeted tail log shows that this later failure is not renewed
pitch divergence. It is the expected consequence of unobservable heading in a
gyro-plus-gravity filter. The residual z-axis gyro bias and measured yaw drift
agree within about 3%, and a heading-rotation model explains nearly all of the
growing lateral velocity error that eventually makes the controller saturate a
leg torque.

The practical conclusion is therefore split:

- The roll/pitch filter implementation defect is fixed.
- Roughly 283 seconds of fully estimated locomotion is a strong result for the
  present sensor model.
- Indefinite or 500-second operation requires an independent heading
  observation. Mahony gain tuning cannot make yaw observable from gravity.

The MPC horizon and inverse-dynamics work that preceded this investigation is
documented separately in
[`reports/mpc/feedforward_horizon_repair.md`](../mpc/feedforward_horizon_repair.md).

## Problem statement

After the force/dynamics corrections made ground-truth runs stable for 500
seconds, replacing the simulator/IMU attitude with the internal filter caused
the robot to dive nose-down. Body angular-rate tracking looked good, while the
world y-velocity estimate deteriorated and the controller could not arrest the
increasing pitch.

The important observation was that the accelerometer low pass was described as
being in the body frame. That description was locally true but incomplete: the
components were in the body frame of the sample at which they were stored, not
automatically in the body frame of the next sample.

## Root cause: filtering vectors from different body frames

Let `B(k-1)` and `B(k)` be the body bases at two consecutive IMU samples. The
stored average `a_f(k-1)` has components in `B(k-1)`, while the new
accelerometer sample `a(k)` has components in `B(k)`. The previous
implementation blended those triples directly:

```text
a_f(k) = a_f(k-1) + alpha * (a(k) - a_f(k-1))
```

That equation is valid only when the body basis does not rotate appreciably. It
worked during earlier mild-attitude gaits and became problematic after the
controller/dynamics changes allowed larger real pitch and roll motion.

The corrected update first obtains the gyro rotation increment from
`B(k-1)` to `B(k)` and carries the old history into the new basis:

```text
delta       = Exp((gyro - bias) * dt)
q_pred      = q_filter * delta
a_f_in_Bk   = delta^-1 * a_f_in_Bk_minus_1
a_f(k)      = a_f_in_Bk + alpha * (a(k) - a_f_in_Bk)
```

The predicted gravity direction, the transported acceleration history and the
new measurement are consequently evaluated at the same time index and in the
same coordinate frame. The correction is then applied as a second exact
quaternion increment:

```text
w_corr  = normalize(a_f(k)) x (q_pred^-1 * world_up)
bias    = clamp(bias - ki * gain * w_corr * dt)
q_filter = q_pred * Exp(kp * gain * w_corr * dt)
```

The implementation uses one exact quaternion-exponential helper for both
increments. This avoids giving propagation and frame transport slightly
different Euler approximations.

## Startup and yaw-datum corrections

### Acceleration-history initialization

Filter mode intentionally trusts the first supplied quaternion as its startup
attitude. That quaternion already determines the body-frame direction of
gravity, so the low-pass state is now initialized from it:

```text
a_f(0) = q_filter(0)^-1 * [0, 0, g]
```

The previous raw-sample initialization preserved an arbitrary startup shove or
contact-settling transient for approximately `accel_filter_tau`. In the captured
old trace, the first retained acceleration sample:

- had norm approximately `28.8 m/s^2`;
- implied gravity pitch about `+0.490 rad`;
- occurred while true pitch was about `-0.107 rad`.

Although the magnitude gate rejected immediate correction, the sample remained
inside the low-pass history. It subsequently drove the y-axis bias estimate to
about `-8.69 mrad/s` while the injected bias was about `+2.05 mrad/s`.

### Separating recursive and published attitude

The estimator now carries two quaternions:

- `_qFilter`: recursive body-to-world filter state in the IMU/world datum;
- `_q`: published attitude, optionally left-multiplied by the fixed inverse
  initial-yaw datum.

This separation makes `zero_initial_yaw` an output-coordinate convention. The
fixed rotation is never fed back into recursive propagation.

### Exact ground-truth bypass

`use_ground_truth=true` now wins over a stale
`attitude_source="filter"` setting. A ground-truth run reproduces every supplied
simulator quaternion rather than silently running part of the Mahony path.
This restores the option's intended diagnostic contract: it isolates everything
downstream of attitude estimation.

## Gain selection under the revised closed loop

The older `0.6/0.09` gain choice came from a 55-second sweep on a mild,
already-stable trajectory. It emphasized rapid bias convergence. With the
large-attitude inverse-dynamics gait, that pair gives residual translational
acceleration too much authority and reaches torque saturation even after the
frame and startup fixes.

Closed-loop trials selected the slower critically damped pair:

```toml
mahony_kp = 0.1
mahony_ki = 0.0025
```

Its nominal natural frequency is `sqrt(ki)=0.05 rad/s`, with damping ratio
`kp/(2*sqrt(ki))=1`. The startup quaternion removes the need for fast initial
attitude convergence, while the lower bandwidth better rejects gait-specific
force.

## Regression tests

The following deterministic cases were added to
`src/quadruped/tests/test_mahony_filter.cc`.

### Rotating-frame invariance

An ideal IMU pitches at `1 rad/s` for `1.5 s` using exact angular rate and exact
body-frame gravity. Before the fix:

- estimated pitch: `1.27792 rad`, truth: `1.5 rad`;
- y-axis bias estimate: saturated at the `0.02 rad/s` clamp.

After transport:

- estimated pitch: `1.5 rad`;
- y-axis bias: approximately `7.8e-16 rad/s`.

### Dynamic first accelerometer sample

The first sample contains a horizontal `0.6 g` transient and every later sample
contains exact gravity. Before quaternion-derived initialization:

- peak pitch error: `0.101788 rad`;
- y-axis bias error: `0.008983 rad/s`.

After initialization from the quaternion:

- peak pitch error: numerical zero;
- bias norm: approximately `2.5e-18 rad/s`.

### Initial-yaw semantics

Two tests verify that a `0.5 rad` initial yaw is removed once and that a later
`0.2 rad` relative turn is still reported. The pre-fix stationary case reached
approximately `1.783 rad` after only ten calls because the datum was reapplied
recursively. The corrected stationary result is numerical zero and the relative
turn result is `0.2 rad`.

### Ground-truth bypass

With both ground truth and filter mode configured, a sequence of changing truth
quaternions must be reproduced to `1e-12 rad`, with no learned gyro bias.

### Bias-clamp isolation

The analytic clamp test now sets `accel_filter_tau=0`. This keeps that test about
anti-windup alone rather than mixing in the physically correct rotation of the
stored gravity reference.

The complete focused executable reports:

```text
test_mahony_filter PASSED
```

## Closed-loop trial record

Unless stated otherwise, these trials used the complete orientation and
position/velocity estimators, `attitude_source="filter"`, synthetic IMU noise,
the default `0.5 s` gait period and `0.55 m/s` forward command.

| Case | Result | Interpretation |
|---|---|---|
| Ground-truth dynamics/attitude reference run | 500 s completed | The force/dynamics repair removed the earlier controller-side failure. |
| Filter attitude with ground-truth position/velocity | 20 s completed | The corrected attitude path alone no longer produces immediate nose-down divergence. |
| IMU attitude passthrough with estimated position/velocity | 60 s completed | The downstream estimator/controller can sustain the gait with externally supplied attitude. |
| Frame/startup fixes, old gains `0.6/0.09` | Failed near 7–9 s | Pitch error was much smaller, but residual velocity/reference error still caused torque saturation. |
| Pure gyro, `kp=ki=0` | Failed near 20.3 s | Removing accelerometer feedback postpones rather than solves drift. |
| `kp=0.2`, `ki=0.01`, seed 12345 | 60 s completed | Stable alternative, but worse final velocity RMS than the chosen pair. |
| `kp=0.1`, `ki=0.0025`, seed 12345 | 60 s completed | Clean stop and sit; final velocity RMS `[0.0312, 0.0247, 0.0338] m/s`. |
| Same gains, seed 67890 | 60 s completed | Clean stop and sit; final velocity RMS `[0.0322, 0.0390, 0.0336] m/s`. |
| Same gains, seed 424242 | 60 s completed | Clean stop and sit; final velocity RMS `[0.0261, 0.0279, 0.0417] m/s`. |
| Same gains, seed 12345, 500 s requested | Failed at `282.555 s` | Leg 0 torque saturation; targeted analysis below identifies accumulated yaw drift. |

The 60-second results matter because they include different realizations of
white noise and bias random walk. All three retained every velocity RMS
component below `0.05 m/s` and completed the normal stop/sit state machine.

## Targeted 282-second endurance diagnosis

The failing 500-second request was reproduced with a log beginning at simulated
time `240.001 s`. It failed at the same `282.555 s`, with leg 0 torque limited
for 20 cycles at scale `0.626`.

Over the logged tail (`240.001` through `282.543 s`):

- yaw error grew from `0.334021` to `0.410359 rad`;
- fitted yaw-error slope was `1.913546 mrad/s`;
- injected z-bias averaged `2.035750 mrad/s`;
- estimated z-bias averaged only `0.173017 mrad/s`;
- measured residual z-bias was therefore `1.862733 mrad/s`;
- yaw slope and residual bias differ by about 3%;
- roll/pitch RMS remained only `10.232/6.964 mrad`.

The lack of z-bias convergence is expected. The Mahony innovation is formed
from measured and predicted gravity. A rotation around gravity changes neither
vector, so the innovation contains no independent yaw or z-bias information.

### Connection to lateral velocity

The position estimator expresses kinematic velocity using the estimated
attitude. If heading error is `dpsi`, a true horizontal velocity `v` is
reconstructed approximately as `Rz(dpsi) v`. Its predicted y-error is:

```text
dv_y = sin(dpsi) * v_x + (cos(dpsi) - 1) * v_y
```

Across the final tail:

- measured y-velocity-error RMS: `0.194828 m/s`;
- heading-predicted RMS: `0.207784 m/s`;
- correlation: `0.918283`;
- remaining RMS after subtracting the prediction: `0.027703 m/s`.

Before the final fall/sit transient, the ten-second window means were:

| Simulation time | Mean yaw error | Measured mean y error | Heading prediction |
|---|---:|---:|---:|
| 240–250 s | `0.344902 rad` | `0.184167 m/s` | `0.194284 m/s` |
| 250–260 s | `0.367018 rad` | `0.191063 m/s` | `0.206023 m/s` |
| 260–270 s | `0.386586 rad` | `0.203591 m/s` | `0.216295 m/s` |
| 270–280 s | `0.403358 rad` | `0.213138 m/s` | `0.226342 m/s` |

The estimator's periodic summaries show the same axis-specific growth:

| Time | Velocity RMS `[x, y, z]` |
|---:|---:|
| 20 s | `[0.0266, 0.0220, 0.0340] m/s` |
| 100 s | `[0.0336, 0.0411, 0.0352] m/s` |
| 200 s | `[0.0339, 0.0794, 0.0329] m/s` |
| 280 s | `[0.0309, 0.1185, 0.0309] m/s` |

The x and z components remain essentially bounded while y deteriorates with
heading. This is inconsistent with renewed nose-down pitch divergence and
consistent with forward velocity being rotated laterally by yaw error.

The controller then consumes the same drift twice:

- `MdlTrot` supplies the estimated unwrapped yaw directly as the MPC current
  yaw, while the desired yaw remains the commanded heading;
- the estimated rotation is used to construct world-frame COM velocity and
  foot moment arms.

The increasing apparent lateral motion and heading mismatch eventually demand
enough corrective force to reach the leg torque limit.

## Invalid/inconclusive intervention retained for completeness

A follow-up attempted to set `gyro_bias_init=0` and rerun 500 seconds. It failed
at `9.124 s`, both with and without startup logging. This result is deliberately
not used as evidence about yaw.

The configuration key is scalar and initializes all three gyro axes. Setting it
to zero therefore changes the observable roll and pitch startup conditions as
well as the unobservable z-axis condition. The early failure belongs to the
known short-term torque-saturation sensitivity and is not a clean z-bias-only
intervention. A causal per-axis experiment would require either a per-axis
simulation-noise configuration or an independent heading observation.

## Implemented files

- `include/quadruped/MdlOrientationEstimator.hh`
  - adds the separate recursive `_qFilter` state;
  - documents frame-consistent acceleration history;
  - changes default gains to `0.1/0.0025`.
- `src/quadruped/MdlOrientationEstimator.cc`
  - adds exact quaternion rotation increments;
  - transports the acceleration history between body frames;
  - seeds gravity history from the startup quaternion;
  - applies the yaw datum only to published output;
  - makes ground truth an exact bypass.
- `src/quadruped/tests/test_mahony_filter.cc`
  - adds rotating-frame, startup-transient, yaw-datum and ground-truth
    regression cases;
  - isolates the bias-clamp test from reference-transport dynamics.
- `config/default/stateestimator.toml`
  - selects the filter and full position/velocity estimator for simulation;
  - changes the default Mahony gains;
  - records the frame-transport, startup and closed-loop gain rationale.

## Remaining limitation and recommended next step

No further roll/pitch gain change is justified by the endurance data. The
remaining failure requires a new measurement, not another tuning parameter.
Suitable heading aids include:

1. use the IMU's fused yaw while retaining the internal filter for roll/pitch;
2. fuse a calibrated magnetometer;
3. fuse visual, motion-capture, GNSS-course or other external heading;
4. add a kinematic/contact yaw observation if the platform and motion provide
   enough excitation.

For hardware whose onboard attitude is unreliable in tilt but whose heading is
usable, the first option is the smallest extension: keep the frame-consistent
internal roll/pitch filter and use only the sensor's yaw channel as an aiding
measurement. If no independent heading exists, yaw drift and a finite endurance
limit must be treated as part of the estimator specification.
