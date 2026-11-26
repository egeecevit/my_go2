# fig-gen: state-estimator figures

Self-contained plotting for Supervisor `.mat` logs (needs numpy, scipy, matplotlib; the
`figs/.venv` interpreter has them). The scripts know only the logged variable names, so they
work on a quick run recorded from `bin/` with the default `supervisor.log.vars` as well as on
benchmark runs; variables a log does not carry are skipped.

Every figure plots **errors against simulator truth**, one quantity per axis, in the body
frame, with fixed y-ranges so two runs can be compared by eye. Whole-run panels show the
per-gait-cycle mean (bias) with a band for the within-cycle spread; raw samples appear only in
the two-cycle zoom and in the phase-folded views.

```
figs/.venv/bin/python fig-gen/plot_est.py bin/deneme.mat
    -> bin/deneme_figs/story.{png,pdf}   velocity error per cycle + zoom, tilt/heading, gyro bias, height, consistency
       bin/deneme_figs/phase.{png,pdf}   errors, foot motion and trust folded on the gait cycle (trot logs)
    options: --out DIR  --window LO HI  --label NAME  --probe T [--gains KP KI TAU] (convergence figure)

figs/.venv/bin/python fig-gen/compare_est.py A.mat B.mat --labels before after
    -> before_after.{png,pdf}: the same quantities, A grey, B colour, one axis each

figs/.venv/bin/python fig-gen/compare_est.py runs/est_baseline runs/est
    -> runs/est/figures/before_after-<run>.*, dumbbell-trot_speeds.*, explain-{rolling,touchdown,heading}.*
```

`estplots.py` is the library (loader `Log`, gait helpers `gait_info`/`cycle_windows`/`fold`,
and the `fig_*` functions). The benchmark `figs/estimation/estbench.py figures` calls these two
scripts for every run. `figs/plot_log.py` remains the generic viewer for any log variable.
