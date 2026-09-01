# MPC / trot tracking figures

Plots `MdlTrot`'s commanded trajectories and requested torque against what the
simulator actually did, for one or two logs. When two logs are supplied,
`<LOG>` is the corresponding log's filename without its `.mat` extension; with
one log, the original unsuffixed filenames are retained.

Commanded/requested traces are dashed blue (`#0072B2`), actual/applied traces
are solid vermillion (`#D55E00`), and limit lines are dotted grey (`#999999`)
-- a colourblind-safe palette with thin lines chosen so overlapping traces
stay distinguishable, unlike the thicker truth/estimate pair in
`figs/estimation`.

| file | commanded / requested | actual / applied |
|---|---|---|
| `torque_{FL,FR,RL,RR}[_<LOG>].png` | `MdlTrot_torquereq[3i:3i+3]` | `MdlSimDriver_ctrl[3i:3i+3]`, clipped at +/-`TORQUE_LIMIT` |
| `body_velocity_tracking[_<LOG>].png` | `MdlTrot_bodyref[4:7]` (world) | `MdlSimDriver_qvel[0:3]` (world) |
| `body_angular_velocity_tracking[_<LOG>].png` | `(0, 0, MdlTrot_bodyref[7])` (body) | `MdlSimDriver_qvel[3:6]` (body) |
| `body_reference[_<LOG>].png` | `MdlTrot_bodyref[0:3]` (pos), `MdlTrot_bodyref[3]` (yaw) | `MdlSimDriver_qpos[0:3]` (pos), `MdlTrot_bodyref[8]` (measured unwrapped yaw) |
| `foot_reference_{FL,FR,RL,RR}[_<LOG>].png` | `MdlTrot_footref[9i:9i+3]` (body frame) | `MdlPosVelEstimator_foottruth[3i:3i+3]` rotated into the body frame with `R_bw = R_wb(MdlSimDriver_qpos[3:7])^T` and re-based on `MdlSimDriver_qpos[0:3]` |

Torque limits, **(23.7, 23.7, 45.43) N*m** per leg for (abduction, hip, knee),
are not logged -- they are compile-time constants from
`createGo2DynamicsConfig()` in `src/quadruped/QuadrupedConfigs.cc`, applied in
`MdlLegControl.cc:63` -- so `plot_mpc.py` hard-codes them rather than spending
log budget on them.

Each subplot is annotated with an RMS difference over the plotted window. The
torque subplots additionally report the fraction of samples where
`|requested|` exceeds the limit and the max `|requested|` as a percentage of
the limit: the applied trace is clipped by construction, so it alone can never
show whether the controller is actually asking for more torque than the
motors can give.

The `body_reference` yaw panel shades `measured yaw +/- MdlTrot_bodyref[11]`,
the clamp band the controller enforces, so it is visible directly on the plot
whether the clamp is binding rather than having to cross-reference the config.

The `foot_reference` and `torque` figures shade intervals where
`MdlTrot_contact` says the leg is scheduled to swing (`contact < 0.5`). That is
the gait *schedule*, not measured ground contact, so the shading reads gait
phase, useful for judging whether tracking lag concentrates at
touchdown/liftoff. A log without `MdlTrot_contact` falls back to
`MdlPosVelEstimator_contacts`, the estimator's contact trust ramp over the same
footfalls; the subtitle then says so, since it is a different quantity.

## Logs missing a variable

Nothing here is a hard error. A missing *command* -- `MdlTrot_torquereq`,
`MdlTrot_bodyref`, `MdlTrot_footref` -- still produces the figure from the
measured trace alone (applied torque against the limit, actual body velocity,
measured foot position), annotated with mean and rms in place of a tracking
error. Only a missing *measurement* -- `MdlSimDriver_ctrl`, `MdlSimDriver_qvel`,
`MdlSimDriver_qpos`, `MdlPosVelEstimator_foottruth` -- drops a figure, and the
printed note then names only the variable actually absent rather than every
variable the figure could use.

## Recording a log

Logging is off by default. Turn it on from `bin/`, which is the only place the
binaries can be launched from:

```
cd bin
./sim.sh -c 'supervisor.log.enable = true' \
         -c 'supervisor.log.file_name = "mpcrun.mat"' \
         -c 'supervisor.log.vars = ["MdlSimDriver_qpos","MdlSimDriver_qvel","MdlSimDriver_ctrl","MdlTrot_torquereq","MdlTrot_footref","MdlPosVelEstimator_foottruth","MdlTrot_contact","MdlTrot_bodyref"]'
```

Then **S** to stand, **T** to trot, let it run, **Q** to quit -- the file is
only finalised on a clean exit.

**Watch the size ceiling.** The line above totals 125 doubles
(19 + 18 + 12 + 12 + 36 + 12 + 4 + 12) against the logger's hard 128-double
ceiling. Adding one more field, or the ground-truth/orientation-estimator
variables from `figs/estimation`'s list, overflows it. Overflowing the
ceiling -- or naming a variable the running binary does not actually define
-- produces **no log file at all**, while the binary still prints
"Logging enabled" and exits 0. There is no error to catch; the only symptom is
a missing `.mat` file after a run that looked normal. If a run comes back
clean but the file is not on disk, that is the first thing to suspect, not a
crash.

## Plotting

The script needs numpy, scipy and matplotlib. `visualize_mpc.sh` runs it out
of a local virtualenv it creates on first use:

```
figs/mpc/visualize_mpc.sh --setup      # create/refresh the venv explicitly
figs/mpc/visualize_mpc.sh              # or just run it; setup happens lazily
```

Equivalently, by hand:

```
python3 -m venv figs/mpc/.venv
figs/mpc/.venv/bin/pip install numpy scipy matplotlib
figs/mpc/.venv/bin/python figs/mpc/plot_mpc.py
```

With no argument it plots `bin/mpcrun.mat`. Relative log paths are first
resolved from the current directory and then from the repository root, so
`bin/mpcrun.mat` works regardless of where the command is launched. It writes
PNGs next to the script by default.

```
plot_mpc.py [logfile1 [logfile2]] [-o OUTDIR] [--start S] [--stop S]
           [--show] [--dpi N]
```

`--start` and `--stop` window the data before anything is computed, so the RMS
and saturation annotations describe exactly what is on screen. Use them to cut
the stand-up transient: `--start 9` drops everything before the trot in a run
driven as above.

`--dpi` defaults to 200 (`figs/estimation`'s plots default to 140) because
these figures are meant for close inspection of saturation and tracking lag at
publication resolution.

## Reading the figures

**The torque figures are the reason `torquereq` is logged at all.** The
applied torque (`MdlSimDriver_ctrl`) is what `MdlLegControl` sent after
clipping to +/-limit, so on its own it can never distinguish "the controller
asked for exactly the limit" from "the controller asked for 130% of the limit
and got clipped." The requested trace makes that visible, and the annotation
turns it into two numbers: what fraction of samples exceeded the limit, and
how far over the peak request went.

**Body velocity/angular-velocity tracking are in world/body frame
respectively, matching the log layout directly** -- no rotation is needed
because `MdlTrot_bodyref`'s linear velocity is already logged in world frame,
the same frame `MdlSimDriver_qvel[0:3]` uses. Roll and pitch rate are always
commanded to zero; only yaw rate is a real setpoint, so those two subplots on
`body_angular_velocity_tracking` show pure gait-induced sway, not tracking
error.

**`body_reference`'s yaw panel is the one to check for clamp activity.** The
controller limits how fast the yaw setpoint can move per cycle
(`MdlTrot_bodyref[11]`, `yawClamp`); the shaded band is `measured +/- clamp`.
If the desired-yaw trace runs along the edge of the band, the clamp is
binding and the controller is not fully realising the commanded yaw rate; if
it stays well inside, the clamp is not the limiting factor for that run.

**The foot-reference figures are read against the shaded swing phase.**
Tracking error concentrated near a swing-to-stance transition is a footfall
placement issue (Raibert-style toe-off/touchdown timing); error that persists
through the middle of stance, where the foot should be static in the body
frame while a good MPC/whole-body solve holds it there, is a different
failure mode and points at the WBC or MPC solve rather than the gait
schedule.
