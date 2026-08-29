# Feedforward and horizon repair

## What the reports were measuring

With `mpc.dt = 0.01` and a 0.5 s gait period, a horizon of 25 spans 0.25 s.
That is the entire stance duration; 0.125 s is *half* of that stance duration,
not the H25 horizon.  It is also the mean time to the next diagonal contact
swap.  A longer horizon should therefore see a contact transition, but it must
not optimize forces for a foot that is physically still in swing.

The old formulation retained three force variables per foot for every horizon
step, then constrained swing forces to zero.  H25 consequently solved a much
larger, poorly conditioned problem than H10 while gaining no extra authority.
It also scheduled support before rear feet had landed.  In the recorded H25
trace, rear touchdown lagged the schedule by 24 ms median (32 ms maximum).

## Implemented changes

- Compact MPC decision variables to scheduled contacts only.  A pure diagonal
  trot at H25 now has 150 force variables rather than 300.  The prior logged
  H25 solve distribution was 0.32 ms median, 2.47 ms p95 and 4.31 ms maximum,
  within the 10 ms solve period.
- Use integer microsecond solve deadlines and round simulated time rather than
  truncating it, eliminating the one-cycle 11 ms cadence misses caused by a
  binary floating-point value slightly below an exact millisecond.
- Correct simulator generalized-state log sizes: qpos uses `nq`; qvel and qacc
  use `nv`.  The old accessors copied `nq` values into `nv` buffers.
- Make requested ground truth exact: it bypasses the attitude filter as well as
  the position/velocity filter.
- Add a Go2 fixed-base leg-dynamics model, validated against MuJoCo inverse
  dynamics, and stateful C2 swing trajectories.  Cartesian swing gains are
  apparent-mass gains.  The full inverse-dynamics torque is implemented and
  tested, but is gated at zero by default: applying it to the legs alone creates
  equal-and-opposite trunk momentum that the SRBD MPC does not model.  In a
  ground-truth H10 test, scale 1 toppled at about 6.6 s; scale 0 retained the
  apparent-mass feedback and completed the run.
- Anchor stance sweep velocity to the commanded stride, with only 25% of the
  filtered velocity residual.  A pure estimated-velocity sweep turned the
  approximately 3 cm/s estimator bias into continuous stance-foot scrubbing.
- Add deterministic headless `autorun_trot_duration`, so these are repeatable
  state-machine runs rather than keyboard-timing experiments.

## Acceptance runs

All runs use headless MuJoCo, no real-time throttling, 0.55 m/s command,
0.5 s period and a 42 s autorun duration.  The supervisor exits only after the
normal trot stop and sit sequence.

| Horizon | State source | Result |
| --- | --- | --- |
| H10 | explicit ground truth | completed; 20.57 m net displacement at stop |
| H25 | explicit ground truth | completed in the repaired benchmark; about 18.1 m net at 40 s |
| H25 | normal estimator | completed; 22.61 m net at sit, no supervisor error |

The normal estimator's horizontal position drifts by about 9% of travelled
distance over the long run.  That is a real remaining limitation of this
foot-only filter: global horizontal position is unobservable without vision,
GPS, motion capture, or a map.  It no longer feeds directly into destructive
foot scrubbing, but it should not be treated as a navigation-grade pose.

## Verification

`ctest --test-dir build --output-on-failure` passed all 14 tests.  Added tests
cover Go2 dynamics against MuJoCo, dynamics/kinematics agreement, apparent-mass
and feedforward torque equations and saturation, exact ground-truth attitude
bypass, C2 swing continuity, integer MPC cadence, and contact-variable
compaction.

