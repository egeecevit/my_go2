# MPC analysis tools

Plotting moved up a directory: `figs/plot_log.py` plots any Supervisor log,
including every MPC and trot variable that used to need a separate script. See
`figs/README.md` for the figures, the logged-variable reference and how to
record a log. What is left here scores and produces runs rather than drawing
them.

All three need the virtualenv one level up:

```
figs/visualize.sh --setup          # creates figs/.venv
figs/.venv/bin/python figs/mpc/score_run.py runs/base2.mat
```

## `score_run.py` -- numeric scorecard

Prints a scorecard for one log instead of plotting it: solver health, body
tracking, gait, effort, force, and with `--dump` a prediction-validity section.

```
score_run.py LOGFILE [--dump DUMPFILE] [--start S] [--stop S]
```

The analysis window defaults to the first scheduled contact transition plus a
10 s settling margin, through the last transition; how it was chosen is always
printed in the header. `--start`/`--stop` override either edge.

It imports `load_log`, `quat_rotate`, `JOINT_DIRECTION`, `TORQUE_LIMIT`,
`JOINT_NAMES` and `LEG_NAMES` from `figs/plot_log.py`, so the log reader, the
leg and joint naming, and the torque limits and sign conventions are defined in
exactly one place. Those six names are therefore part of `plot_log.py`'s
interface; renaming them there breaks this.

## `mpcdump.py` -- solve dump parser

Standalone parser for the little-endian float64 dump
`MdlConvexMPC::_dumpSolve()` writes (`src/quadruped/MdlConvexMPC.cc:644-686`).
`read_dump(path)` returns `(header, {t, x0, xref, U, Xpred, contact})`. Run it
directly to print the header and record count. Depends only on numpy.

## `run_cell.sh` -- deterministic run harness

One headless trot run, written to `runs/<name>.{mat,bin,log,manifest}`.

```
figs/mpc/run_cell.sh <name> [extra -c overrides ...]
  DUR=<seconds>   trot duration, default 40
  OUT=<dir>       output directory, default <repo>/runs
```

Runs are serialized by an `flock`: the simulator's `LogServer` binds a fixed
ENet port (5500), so two simulators cannot coexist. The manifest records the
date, git SHA, dirty state, duration and overrides. It sets `log.period = 2`
and its own 119-double `supervisor.log.vars` list -- the only place
`MdlSimDriver_grf`, `MdlConvexMPC_forces` and `MdlConvexMPC_solver` are
recorded. It greps the run log for errors and warns explicitly if no `.mat` was
written, which is the symptom of overflowing the 128-double logging ceiling.
