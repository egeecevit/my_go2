# State estimator figures

Plots `MdlOrientationEstimator` and `MdlPosVelEstimator` against the simulator's
own state. One figure per quantity, three subplots for x, y and z, ground truth
in solid blue and the estimate in solid red on every one.

| file | estimate | ground truth |
|---|---|---|
| `body_position.png` | `MdlPosVelEstimator_state[0:3]` | `MdlSimDriver_qpos[0:3]` |
| `body_velocity.png` | `MdlPosVelEstimator_state[3:6]` | `MdlSimDriver_qvel[0:3]` |
| `body_orientation.png` | `MdlOrientationEstimator_state[4:7]` | `MdlSimDriver_qpos[3:7]` → rpy |
| `foot_position_{FL,FR,RL,RR}.png` | `MdlPosVelEstimator_footholds[3i:3i+3]` | `MdlPosVelEstimator_foottruth[3i:3i+3]` |

Each subplot is annotated with the RMS difference over the plotted window.

## Recording a log

Logging is off by default. Turn it on from `bin/`, which is the only place the
binaries can be launched from:

```
cd bin
./sim.sh -c 'supervisor.log.enable = true' -c 'supervisor.log.file_name = "estrun.mat"'
```

Then **S** to stand, **T** to trot, let it run, **Q** to quit — the file is only
finalised on a clean exit. A 30 s run is about 47 MB at the default 1 kHz;
`supervisor.log.period = 10` thins it by ten if that matters.

The variables the script needs are all in `supervisor.log.vars` by default.
Note the size limit documented there: the whole line must fit in 128 doubles, and
overflowing it looks like a misspelled variable name rather than like a size
problem.

## Plotting

The script needs numpy, scipy and matplotlib, none of which are system packages
on this machine. A local virtualenv keeps them out of the way:

```
python3 -m venv figs/estimation/.venv
figs/estimation/.venv/bin/pip install numpy scipy matplotlib
figs/estimation/.venv/bin/python figs/estimation/plot_estimation.py
```

It defaults to `bin/estrun.mat` and writes the PNGs next to itself.

```
plot_estimation.py [logfile] [-o OUTDIR] [--start S] [--stop S] [--show]
                   [-a | --heading-aligned]
```

`--start` and `--stop` window the data before anything is computed, so the RMS
annotations describe exactly what is on screen. Use them to cut the stand-up
transient: `--start 9` drops everything before the trot in a run driven as above.

`-a` removes the heading contribution from the position figures; see below.

## Reading the figures

**What the orientation figure means depends on `attitude_source`.** The figure
labels itself with which one produced the log.

Under `"imu"` it measures nothing. In simulation the IMU quaternion *is*
`qpos[3:7]` — `MdlSimDriver::_readIMUData()` copies the same array
`getGroundTruth()` reports — and the module passes it through untouched, so the
two curves are the same numbers and the residual 2e-4 rad is a one-cycle sampling
skew (`MdlSimDriver` runs last, so the logged truth leads the logged estimate by
a millisecond).

Under `"filter"`, the current default, the attitude is estimated from the rates
and gravity and the comparison is real: roll and pitch should track within a
couple of milliradians, and yaw drifts at the gyroscope's yaw bias times elapsed
time because gravity says nothing about heading.

Either way the figure shows what the trunk actually does, which is worth knowing:
during a kinematic trot roll swings about ±0.03 rad and yaw about ±0.04 rad at
the gait frequency. The yaw *oscillation* is a common suspect for the horizontal
drift and is not the cause. Causality runs the other way — two feet genuinely
fixed to the ground cannot let the trunk yaw, so a yawing trunk is evidence the
stance feet are scrubbing, and that scrub is what biases the foot-velocity
measurement the filter's velocity rows depend on.

**Body position x carries the gait's drift; y carries the heading's.** Nothing in
this filter observes absolute horizontal position, so both accumulate, but from
different sources and with different fixes:

- x is the stance scrub, about -40 mm over a 0.75 m path. The lever is the gait,
  not the filter — see the `swing_kp` sweep in `config/default/trot.toml`, where
  drift tracks foot slip almost linearly.
- y is mostly the drifting heading. The filter's velocity measurement is
  expressed through the estimated attitude, so a heading error `dyaw` renders a
  forward velocity `vx` as a lateral `dyaw * vx`, and that integrates. On a
  straight run it comes to roughly `dyaw_final * x_final / 2`, tens of
  millimetres, and it looks exactly like a translation error.

`-a` subtracts that integral and re-plots. It is the view to use when asking
whether a change to the position filter helped: on the run above, y RMS drops
from 20 mm to 6 mm and the ramp disappears entirely, leaving a bounded
oscillation. Note that this is an integral over the run, not a rotation of the
endpoint — rotating the final error by `dyaw` would remove a fraction of a
millimetre and leave the whole ramp in place.

**The foot figures are the body figure plus a leg.** Each foothold state is
anchored to the body estimate through the body-to-foot measurement rows, so it
inherits the body's horizontal offset; the per-leg z traces are the ones that show
something independent, stepping between the ground datum and the swing arc. In
the raw y figures the front and rear legs splay to opposite sides, which is the
same heading error acting through each leg's lever arm `dyaw * p_rel_x`; `-a`
removes that too and the four legs collapse back onto the body's own error.
