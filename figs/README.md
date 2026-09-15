# Log figures

`plot_log.py` plots a Supervisor `.mat` log -- whatever it happens to contain.
`visualize.sh` runs it out of a local virtualenv it creates on first use:

```
figs/visualize.sh --setup                          # create/refresh the venv
figs/visualize.sh --profile estimator  bin/deneme.mat
figs/visualize.sh --profile controller runs/base2.mat
figs/visualize.sh --list runs/base2.mat            # what would be written
```

It replaces the pair of plotters that used to live in `figs/estimation` and
`figs/mpc`. Those each began by `require()`ing their own variable set and
exiting on anything else, so neither could read the other's logs and a newly
registered variable appeared in no figure at all.

## How a figure gets chosen

A table in `plot_log.py` (`REGISTRY`) describes each known variable as a
sequence of **column blocks**, and a block may tag itself with the physical
**quantity** it represents and the **role** it plays in it — truth, estimate,
command, measured, requested, applied, raw.

* Blocks sharing a quantity are drawn on one figure, overlaid, annotated with
  the RMS of every other role against the reference one.
* **A quantity that resolves only one role is not a comparison and is dropped**,
  unless its profile says it stands alone.
* Which figures are drawn is a `--profile`: a hand-curated list, deliberately
  not derived from what the log happens to contain, because deriving it is what
  produced thirty figures a run.

So adding a variable to `supervisor.log.vars` is enough to see it, tagging it in
`REGISTRY` is enough to compare it, and listing it in `PROFILES` is what decides
whether it is worth drawing.

Reference-like roles (`truth`, `command`, `requested`, `injected`) are dashed,
actual-like ones (`estimate`, `measured`, `applied`) are solid, a raw sensor
trace sits underneath both as a wide translucent band. The palette is
Okabe-Ito, colourblind-safe, and one colour means one role on every figure.

## Profiles

| profile | what it answers | figures |
|---|---|---|
| `estimator` | is the state estimator right? | `body_position` `body_velocity` `body_orientation` `orientation_quaternion` `body_angular_velocity` `gyro_bias` `gravity_reference` `attitude_filter` `foot_position_<LEG>` `estimator_cov_{position,velocity,foot}` |
| `controller` | is the controller tracking? | `torque_<LEG>` `foot_reference_<LEG>` `grf_<LEG>` `grf_total` `body_velocity` `body_yaw` `body_yaw_rate` `contact` |
| `plant` | what did the joints do? | `joint_angle_<LEG>` `joint_velocity_<LEG>` |
| `all` | everything, as a safety net | every column of every variable reaches a figure, with a coverage report; leftovers become `generic_<variable>_<block>.png` |

Without `--profile`, the one the log best supports is picked and named. The
estimator set and the trot/MPC set are disjoint recordings, so in practice a run
selects its own profile.

`estimator_cov_*` needs `MdlPosVelEstimator_cov`, which is in no shipped
`supervisor.log.vars` list and does not fit the 128-double ceiling alongside the
rest of the estimator set. Those three figures appear the moment you log it.

`body_position` keeps truth-vs-estimate but drops the commanded trace: body
position is integrated from the velocity command, not tracked, so the command
says nothing about the controller and crowds the comparison that does.

## Annotations

No figure prints a bare `rms` unless there is a reference role to measure
against. Where there is not, the statistic follows what the signal *is*:

| signal | annotation |
|---|---|
| innovation (`wcorr`) | `rms innovation 15.8 mrad, peak 27.9 mrad` |
| flag (`contact`) | `duty 55.1%, 62 cycles` |
| weight (`gain`, `torque_scale`) | `mean 0.994, min 0.977` |
| status (QP return) | `0 nonzero of 14501` |
| duration (solve time) | `p50 / p95 / max` in ms |
| variance | median and max of the standard deviation |
| anything else | range and min/max — the *mean* of a world-frame position is how far the robot walked, which is not what the figure is about |

## What each comparison figure reads

| figure | quantity | roles it can carry |
|---|---|---|
| `body_position` | body position, world | `MdlSimDriver_qpos.base_pos` truth · `MdlPosVelEstimator_state.pos` estimate · `MdlTrot_bodyref.des_pos` command |
| `body_velocity` | body velocity, world | `MdlSimDriver_qvel.base_vel` truth · `MdlPosVelEstimator_state.vel` estimate · `MdlTrot_bodyref.des_vel` command |
| `body_orientation` | attitude, rpy | `rpy(MdlSimDriver_qpos.base_quat)` truth · `MdlOrientationEstimator_state.rpy` estimate |
| `orientation_quaternion` | attitude, quaternion | `MdlSimDriver_qpos.base_quat` truth · `MdlOrientationEstimator_state.quat` estimate |
| `body_angular_velocity` | body rate | `MdlSimDriver_qvel.base_angvel` truth · `MdlOrientationEstimator_state.omega_body` estimate · reconstructed raw gyro |
| `body_yaw` | yaw | `MdlTrot_bodyref.des_yaw` command · `.meas_yaw` measured, with the `.yaw_clamp` band shaded |
| `body_yaw_rate` | yaw rate | `MdlTrot_bodyref.cmd_yawrate` command · `MdlSimDriver_qvel.base_angvel[z]` truth |
| `gyro_bias` | gyro bias | `MdlOrientationEstimator_state.gyro_bias` estimate · `..._filter.injected_bias` injected |
| `gravity_reference` | gravity reference, body | `MdlOrientationEstimator_filter.acc_filt` estimate · ideal gravity from truth · reconstructed raw specific force |
| `foot_position_<LEG>` | foot position, world | `MdlPosVelEstimator_foottruth.pos` truth · `..._footholds.pos` estimate |
| `foot_reference_<LEG>` | foot position, body | `MdlTrot_footref.cmd_pos` command · foot truth rotated into the body frame |
| `torque_<LEG>` | joint torque | `MdlTrot_torquereq.torque` requested · `MdlSimDriver_ctrl.torque` applied, with the limit lines |
| `grf_<LEG>`, `grf_total` | ground reaction force, world | `MdlConvexMPC_forces.u0` command · `MdlSimDriver_grf.force` measured, with a body-weight reference on total `F_z` |
| `contact` | contact | `MdlTrot_contact.stance` schedule · `MdlPosVelEstimator_contacts.trust` trust ramp |
| `joint_angle_<LEG>`, `joint_velocity_<LEG>` | joint state | `MdlSimDriver_qpos.joint_pos`, `..._qvel.joint_vel` (no comparison exists; `plant` profile) |
| `attitude_filter` | Mahony internals | `..._filter.wcorr` innovation, `.gain` trust, and `gain x wcorr` — the product the bias actually integrates |

A quantity whose every trace would have to be computed from another variable
is not drawn -- it would carry nothing its sources do not already show.

Constants from `src/quadruped/QuadrupedConfigs.cc` and `config/default/mpc.toml`
are hard-coded rather than logged: the per-joint torque limits
**(23.7, 23.7, 45.43) N·m**, the leg sign convention, and the **15.2064 kg**
aggregate mass behind the body-weight line.

## Logged variable reference

Widths are the number of doubles. Leg-major means FL, FR, RL, RR
(`QuadrupedKinematics::LegIndex`); joints are abduction, hip (flexion), knee.

| variable | w | columns |
|---|---|---|
| `MdlOrientationEstimator_state` | 16 | `0:4` quaternion wxyz body→world · `4:7` rpy ZYX [rad] · `7:10` bias-corrected gyro [rad/s, body] · `10:13` specific force, gravity included [m/s², world] · `13:16` estimated gyro bias [rad/s] |
| `MdlOrientationEstimator_filter` | 10 | `0:3` Mahony gravity innovation · `3` fade gain · `4:7` low-passed gravity reference [m/s², body] · `7:10` injected gyro bias [rad/s] |
| `MdlPosVelEstimator_state` | 9 | `0:3` estimated position [m, world] · `3:6` estimated velocity [m/s, world] · `6:9` the same velocity in the body frame |
| `MdlPosVelEstimator_footholds` | 12 | estimated foot position [m, world], leg-major |
| `MdlPosVelEstimator_error` | 9 | `0:3` position error [m] · `3:6` velocity error [m/s] · `6` drift [%] · `7` distance travelled [m] · `8` net displacement [m] |
| `MdlPosVelEstimator_cov` | 18 | EKF covariance diagonal: `0:3` position [m²] · `3:6` velocity · `6:18` foot position, leg-major |
| `MdlPosVelEstimator_contacts` | 4 | per-leg contact trust weight in [0,1], **ramped, not a flag** |
| `MdlPosVelEstimator_footerr` | 4 | per-leg ‖estimate − truth‖ [m]; **negative marks unavailable/untrusted** |
| `MdlPosVelEstimator_foottruth` | 12 | simulator foot position [m, world], leg-major |
| `MdlTrot_scales` | 2 | speed ramp, lift ramp; duplicates `bodyref[9:11]` |
| `MdlTrot_footref` | 36 | per leg, 9 each: `+0:3` commanded position [m, body] · `+3:6` velocity · `+6:9` acceleration |
| `MdlTrot_contact` | 4 | scheduled stance (1) or swing (0), per leg |
| `MdlTrot_torquescale` | 4 | per-leg saturation factor in (0,1] |
| `MdlTrot_torquereq` | 12 | requested joint torque [N·m] before saturation, **controller sign** |
| `MdlTrot_bodyref` | 12 | `0:3` desired position [m] · `3` desired yaw · `4:7` desired velocity [m/s] · `7` commanded yaw rate · `8` measured unwrapped yaw · `9` speed ramp · `10` lift ramp · `11` yaw clamp |
| `MdlConvexMPC_forces` | 12 | first horizon input: commanded GRF [N, world], leg-major |
| `MdlConvexMPC_solver` | 2 | qpOASES return value (0 = success), solve wall time [s] |
| `MdlSimDriver_qpos` | 19 | `0:3` base position [m] · `3:7` base quaternion wxyz · `7:19` 12 joint angles [rad] |
| `MdlSimDriver_qvel` | 18 | `0:3` base linear velocity [m/s, world] · `3:6` base angular velocity [rad/s, **body**] · `6:18` 12 joint velocities |
| `MdlSimDriver_qacc` | 18 | generalized acceleration, same layout as `qvel` |
| `MdlSimDriver_ctrl` | 12 | applied joint torque [N·m], **MuJoCo sign** — FR and RR abduction are inverted relative to the controller; the script maps them before comparing |
| `MdlSimDriver_grf` | 12 | measured foot ground reaction force [N, world], leg-major |

## Recording a log

Logging is off by default. Turn it on from `bin/`, which is the only place the
binaries can be launched from:

```
cd bin
./sim.sh -c 'supervisor.log.enable = true' -c 'supervisor.log.file_name = "estrun.mat"'
```

Then **S** to stand, **T** to trot, let it run, **Q** to quit -- the file is
only finalised on a clean exit, so killing the sim leaves an unreadable file.

Which variables are recorded is `supervisor.log.vars` in
`config/default/supervisor.toml`. `plot_log.py` plots whichever of them are in
the file, so that list can be changed freely without touching the plotter.

**There is a hard ceiling of 128 doubles per logged line**
(`rtclient::LogTask::addVar`, `RHEX_LOG_MAX_MSG/2`). Overflowing it -- or naming
a variable the running binary does not define -- produces **no `.mat` file at
all**, while the binary still prints "Logging enabled" and exits 0. There is no
error to catch; the only symptom is a missing file after a run that looked
normal. If a run comes back clean but the file is not on disk, suspect that
first, not a crash. The shipped list is 125 doubles, and the 22 registered
variables total 257, so any one run can record at most half of what exists.

## Reading the figures

**What `body_orientation` means depends on `attitude_source`,** and the figure
labels itself with which one produced the log. Under `"imu"` it measures
nothing: in simulation the IMU quaternion *is* `qpos[3:7]`, the array plotted as
truth, and the module passes it through untouched -- so agreement is by
construction. Under `"filter"` the attitude is estimated from the rates and
gravity and the comparison is real: roll and pitch should track within a couple
of milliradians, and yaw drifts at the gyroscope's yaw bias times elapsed time
because gravity says nothing about heading.

**`body_angular_velocity` checks a different output of the same stage.** The
proportional Mahony correction changes quaternion propagation; it is not itself
a physical rate. The integral correction estimates gyroscope bias, and the
module publishes `gyro - gyroBias` for the position estimator's `omega x p`
foot-velocity transport term. The figure therefore shows simulator body rate,
reconstructed raw gyro and the published bias-corrected gyro. With
`mahony_ki = 0` the bias is identically zero and the raw and corrected traces
coincide exactly; that is the correct output, not a broken plot.

`MdlSimDriver` runs after the estimators, so logged truth is one *control
cycle* newer than the estimator state. Undoing that by shifting one *logged
sample* is only valid at `supervisor.log.period = 1`; at the shipped period of
10 it moves truth 10 ms instead of 1 and inflates the reported error by most of
an order of magnitude. The script applies the shift only when the log was
recorded at the control rate, and says on the figure which happened.

**Body position x carries the gait's drift; y carries the heading's.** Nothing
in this filter observes absolute horizontal position, so both accumulate, from
different sources and with different fixes:

- x is the stance scrub, about -40 mm over a 0.75 m path. The lever is the
  gait, not the filter -- see the `swing_kp` sweep in `config/default/trot.toml`,
  where drift tracks foot slip almost linearly.
- y is mostly the drifting heading. The filter's velocity measurement is
  expressed through the estimated attitude, so a heading error `dyaw` renders a
  forward velocity `vx` as a lateral `dyaw * vx`, and that integrates. On a
  straight run it comes to roughly `dyaw_final * x_final / 2`, tens of
  millimetres, and looks exactly like a translation error.

`-a` subtracts that integral and re-plots. It is the view to use when asking
whether a change to the position filter helped: on a typical run y RMS drops
from 20 mm to 6 mm and the ramp disappears, leaving a bounded oscillation. Note
that this is an integral over the run, not a rotation of the endpoint --
rotating the final error by `dyaw` would remove a fraction of a millimetre and
leave the whole ramp in place.

**The foot figures are the body figure plus a leg.** Each foothold state is
anchored to the body estimate through the body-to-foot measurement rows, so it
inherits the body's horizontal offset; the per-leg z traces are the ones that
show something independent, stepping between the ground datum and the swing
arc. In the raw y figures the front and rear legs splay to opposite sides,
which is the same heading error acting through each leg's lever arm
`dyaw * p_rel_x`; `-a` removes that too.

**The torque figures are the reason `torquereq` is logged at all.** The applied
torque is what `MdlLegControl` sent after clipping to ±limit, so on its own it
can never distinguish "the controller asked for exactly the limit" from "the
controller asked for 130% and got clipped". The requested trace makes that
visible, and the annotation turns it into two numbers: what fraction of samples
exceeded the limit, and how far over the peak request went.

**`body_yaw`'s shaded band is where to check for clamp activity.** The
controller limits how fast the yaw setpoint may move per cycle
(`MdlTrot_bodyref.yaw_clamp`); the band is `measured ± clamp`. A desired-yaw
trace running along the edge means the clamp is binding and the commanded yaw
rate is not being fully realised.

**Swing shading is the gait *schedule*, not measured contact.** It comes from
`MdlTrot_contact`, falling back to `MdlPosVelEstimator_contacts < 0.5` with the
source named in the subtitle, which matters because the trust signal is a ramp
and thresholding it displaces the edges. Measured ground reaction force is
deliberately *not* in that chain: shading `foot_reference_<LEG>` with measured
contact would make the figure self-referential exactly where it is read for
touchdown timing. The three notions are compared in `contact` and nowhere else.

Tracking error concentrated near a swing-to-stance transition is a footfall
placement issue (Raibert-style toe-off/touchdown timing); error that persists
through the middle of stance, where a good MPC/whole-body solve should hold the
foot static in the body frame, points at the WBC or MPC solve rather than the
gait schedule.

**`grf_total` is the force budget.** Summed over the legs, vertical GRF against
the body-weight line says directly whether the controller is asking for enough
force to hold the robot up; no per-leg figure makes that readable. The
commanded trace is the MPC's first horizon input, held between solves, so it is
a staircase at the solve rate sampled at the logging rate -- steps in it are
the solver, not the plant.

## Other tools in this directory

`mpc/score_run.py` prints a numeric scorecard for one log instead of plotting
it, and imports the log reader and the sign/limit constants from `plot_log.py`.
`mpc/mpcdump.py` parses the binary solve dump `MdlConvexMPC::_dumpSolve()`
writes. `mpc/run_cell.sh` is a headless deterministic run harness. See
`mpc/README.md`.
