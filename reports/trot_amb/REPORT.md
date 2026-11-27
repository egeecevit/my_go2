# Angular-momentum balancing inside the whole-body trot: static hold, trot in place, slow forward trot

Generated 2026-11-27 19:17 from `reports/trot_amb/results.json`. Branch `amb-trot`. Simulated Go2, trot in place, whole-body path (`trot.wbc = true`). The controller's state input is simulator ground truth (attitude and position/velocity) on every run except the two marked filter runs. The attitude integral is on unless a row says otherwise.

## 1. Result

1. **AMB makes a slow trot in place possible, with one addition.** With two-foot phases of 0.75 to 2.0 s and a 0.5 s four-foot phase, every two-foot phase is a real two-foot stand and roll and pitch stay within 1.8 deg, in both realizations of AMB (Table 4). Without AMB the same gaits never stand on two feet: the body tips about 15 deg onto a swing foot in every phase, or falls.
2. **The addition is CoM centring in the four-foot phase.** The swing legs can only absorb a CoM that starts within about 3 mm of the support line (held from -3.1 to 2.7 mm, lost at 5.5 mm and at -6.0 mm; Table 2). On the trot's own footprint the CoM is 9.2 mm off each diagonal at the first lift-off. While all four feet are down, the body reference is therefore shifted until the CoM is over the next support line. Centring without AMB does not hold a phase; AMB without centring falls after 2.7 s.
3. **Both realizations work in the in-place gait; outside it the reference offset is the reliable one.** The pass-through (thesis Eqs. 6.23, 6.26) needs the joint friction cancelled, as `MdlAMB` did. Of the 4 long holds and forward runs here (Tables 3, 3b, 9) it lost 3, the offset mode 0.
4. **It is not tied to one period and duty.** Two-foot phases from 0.75 to 30 s and four-foot phases from 0.2 to 2 s were run (Tables 3b, 4, 5). The lower bound on the two-foot phase comes from the 0.2 s lift and descent plus AMB's return; the four-foot phase only has to let the CoM reach the next line.
5. **It walks forward, slowly, on a zig-zag.** Commanded 0.02 to 0.15 m/s it moves at 0.016 to 0.099 m/s with every two-foot phase held (Table 8). The body follows the command only along the current support line, and the landing feet are placed so that the next line passes under the CoM. Without the law the same zig-zag tips onto a swing foot in almost every phase; with nothing it falls.
6. **Limits.** A 0.1 s lateral push in the middle of a two-foot phase is survived up to 20 N / 40 N (offset, +y / -y) and 20 N / 20 N (pass-through). With the position/velocity filter in the loop two-foot phases of 2 s run for 60 s, but a 30 s hold fails, with or without AMB: the estimate itself sinks and slides in a gait that slow. Forward, a commanded 0.20 m/s falls after 4.3 s, and the achieved speed is about 0.7 of the commanded one. At the normal 0.3 s gait AMB does not act at all: the 0.15 s swing is shorter than its window.
7. **Two things had to change in the trot itself** before any of this could be measured: the swing has to leave the ground quickly (`swing_lift_time`), and AMB has to be back on the planned trajectory before the foot comes down (Section 3).

## 2. What was built

*Table 1. The pieces. All keys are in `[trot]`; the defaults leave the trot unchanged.*

| Piece | Where | What it does |
|---|---|---|
| Support pair as a parameter | `MdlAMB::geometry`, `robotMomentum`, `project` (`pair_t`) | The projection onto the support line for either diagonal. The second diagonal is ordered RL to FR: it is the mirror image of FL to RR, and reversing the line keeps every sign convention. Checked against the mirrored state in `test_trot_amb`. |
| `TrotAMB` | `TrotAMB.{hh,cc}`, called from `MdlTrot::_wbcTick` | The thesis law (`MdlAMB::ambLaw`, same template and gains) on the current stance pair in every two-foot phase, state reset at each phase. `amb`, `amb_mode`. |
| Mode `offset` | `TrotAMB::rotateReference` | The law's angular acceleration is integrated into a rotation of each planned swing-foot reference about its hip, axis parallel to the support line; returned to zero before the descent. |
| Mode `passthrough` | `MdlTrot::wbcSolve` (`amb_dir`, `amb_share`, `amb_acc`) | Thesis (6.23), (6.26): the swing task's error is removed along the tangential direction and the acceleration is a pure feedforward. Friction cancellation and joint PD off on that leg (6.28). |
| Swing lift time | `SwingTrajectory::sampleLifted`, `swing_lift_time` | The foot rises in a fixed time, holds, and comes down in the same time. |
| CoM centring | `MdlTrot::_wbcTick`, `amb_centre` | The base position task takes its horizontal error from the planted feet, not from the position estimate. Across the support line it keeps the CoM the robot would have with its swing feet on their planned references over the line; along the line it follows the commanded velocity's component along it, which is also the velocity reference of the MPC. |
| Foot placement | `MdlTrot::_wbcTick`, `SwingTrajectory::shiftEnd`, with `amb_centre` | The two feet that lift get touchdown points whose line passes under the CoM, so the body does not have to cross to the next support line while four feet are down. |

Settings of the runs below: `swing_lift_time = 0.2`, `wbc_touchdown_depth = 0.012` (Section 7), `amb_centre = true` where centring is on, law gains [950.0, 360.0, 41.0], template as `[amb]` (the centreline fit was not redone: it held), `amb_alpha_max = 0.9`, `amb_return = 0.15`.

Checks: `ctest` 100% tests passed, 0 tests failed out of 22 (22 with the new `test_trot_amb`; `test_trot_wbc` gained a pass-through case). With `amb = false` and the other new keys at their defaults, the default trot on both paths, a slow whole-body trot and the key-B `MdlAMB` balance log the same values tick for tick as before the work (compared on 2026-11-27 with the final binary).

## 3. Four findings on the way

1. **A long swing did not leave the ground.** The trot's clearance profile grows with the cube of the swing's progress: an 8 s swing is 0.7 mm up after 0.4 s. The foot scheduled to swing stayed on the ground, was loaded as the body began to tip (46 N for 70 ms measured) and kicked the body about the support line. The "two-foot phases" of the earlier baseline were three-foot phases: in Table 3 no phase of any gait without AMB counts as a two-foot stand.
2. **The CoM is not over the diagonals.** On the default footprint it is about 9 mm from each support line at the first lift-off, which is three times what the swing legs can absorb.
3. **A foot must not land with AMB's offset on it.** With the lift profile the descent starts before the touchdown; a foot that met the ground while still rotated by AMB landed 3 to 7 cm off its target. AMB now finishes its return before the descent begins.
4. **A held foot must stay up** (Ege saw this in the simulator: after the fast lift the legs came down slowly, rose again before the switch, and could touch the ground). Two causes. The touchdown depth was spread over the whole swing, so the planned height sank 12 mm during a 2 s hold; it is now applied in the descent only. And AMB turns each free leg on an arc about its hip: the foot swinging outward rises, the one swinging in under its hip comes down, 3 cm below its planned height at 35 deg. In offset mode a foot no longer goes below its planned height (lowest swing foot over the Table 4 gaits: 95 mm of the 100 mm clearance; pass-through, still on the arc: 82 mm). The outward foot still rises, up to 180 mm: held level, it reached only half the angle asked for and the balance was lost. Plots: `figs/foot_heights_before.png`, `figs/foot_heights_after.png`.

## 4. Static hold on the trot's own footprint (step 1)

One two-foot phase of 8 s. The footprint is moved fore-aft (`trot.origin`) so the CoM starts at different distances from the support line; no centring.

*Table 2. Static hold against the CoM's starting offset from the support line. Held: neither swing foot came back to the ground and the body stayed within 10 deg.*

| Footprint origin x [mm] | CoM offset at lift-off [mm] | offset: held | offset: peak roll / pitch [deg] | offset: leg travel [deg] | pass-through: held | pass-through: peak roll / pitch [deg] | pass-through: leg travel [deg] |
|---|---|---|---|---|---|---|---|
| +0 | 9.2 | no | – | – | no | – | – |
| -5 | 5.5 | no | – | – | no | – | – |
| -9 | 2.7 | yes | 0.7 / 1.7 | 38 | yes | 0.6 / 1.0 | 35 |
| -13 | -0.2 | yes | 0.4 / 0.5 | 10 | yes | 0.2 / 0.3 | 10 |
| -17 | -3.1 | yes | – | 23 | yes | 0.4 / 0.4 | 24 |
| -21 | -6.0 | no | – | – | no | – | – |

The offset mode held the phase started at -3.1 mm and fell afterwards, at the stop. At the origin of -13 mm, AMB off tips onto a swing foot (18.3 / 19.7 deg, 0 of 1 held). The pass-through without friction cancellation falls after 4.6 s; with the cancellation it holds whether the swing joint PD is on (0.3 / 0.3 deg) or off (0.2 / 0.3 deg).

*Table 3. Both diagonals in one run on the default footprint, with centring: 8 s on one diagonal, 0.5 s on four feet, 8 s on the other.*

| Case | First diagonal | Outcome | Two-foot phases held | Peak roll / pitch [deg] | CoM offset at the lift-offs [mm] |
|---|---|---|---|---|---|
| centring only | FL-RR | no fall | 0 of 2 | 14.0 / 17.9 | 4.6, -8.5 |
| centring only | FR-RL | no fall | 0 of 2 | 14.0 / 17.9 | -4.6, 8.5 |
| offset | FL-RR | no fall | 2 of 2 | 0.4 / 1.1 | 4.6, -0.4 |
| offset | FR-RL | no fall | 2 of 2 | 0.4 / 1.1 | -4.6, 0.4 |
| passthrough | FL-RR | falls after 6.8 s | 0 of 1 | – | 4.6 |
| passthrough | FR-RL | falls after 7.1 s | 0 of 1 | – | -4.6 |

The first lift-off comes 0.5 s after TROT entry, before the centring has settled (the CoM starts 9 mm off): the first phase is the hardest of a run, and the CoM offsets at the lift-offs in the last column show what each case started from.

*Table 3b. One two-foot phase of 30 s after 2 s on four feet, default footprint: can a centred robot stay on a diagonal?*

| Case | Outcome | Phase held | Peak roll / pitch [deg] | Roll / pitch rms [deg] | Leg travel [deg] |
|---|---|---|---|---|---|
| nothing | falls after 2.7 s | 0 of 1 | – | – | – |
| offset | no fall | 1 of 1 | 0.7 / 0.4 | 0.10 / 0.06 | 16 |
| passthrough | no fall | 1 of 1 | 0.5 / 0.2 | 0.08 / 0.06 | 12 |
| offset, position/velocity filter in the loop | falls after 2.9 s | 0 of 1 | – | – | 60 |
| nothing, position/velocity filter in the loop | falls after 3.0 s | 0 of 1 | – | – | – |

On ground truth the answer is yes for the offset mode: once the CoM is over the line and the body at rest, the swing legs settle at a constant angle and nothing drifts over the 30 s. The pass-through in this run: no fall. In two earlier runs of the same case on 2026-11-27, before changes elsewhere in the centring, it fell after 12.7 s and 19.6 s with its legs creeping steadily outward: nothing in that mode ties the legs to a position. With the position/velocity filter in the loop a hold this long fails, and so does the same gait with AMB off: in the 2 s on four feet before the lift the body already sinks about 2 cm and slides several centimetres, which is the filter's estimate in a gait this slow (period 64 s), not the balance. At two-foot phases of 2 s the filter is tolerated (Table 5).

## 5. Trot in place with long two-foot phases (step 2)

*Table 4. Four-foot phase 0.5 s. Nothing: AMB and centring off, same lift profile.*

| Two-foot phase [s] | Case | Outcome | Two-foot phases held | Peak roll / pitch [deg] | Roll / pitch rms [deg] | CoM offset rms [mm] | Leg travel [deg] |
|---|---|---|---|---|---|---|---|
| 0.75 | nothing | falls after 6.5 s | 0 of 5 | – | – | 90.4 | – |
| 0.75 | centring only | no fall | 4 of 12 | 14.8 / 16.8 | 3.96 / 5.66 | 33.1 | 5 |
| 0.75 | offset | no fall | 12 of 12 | 0.6 / 0.7 | 0.17 / 0.20 | 1.7 | 18 |
| 0.75 | passthrough | no fall | 12 of 12 | 0.5 / 0.4 | 0.11 / 0.10 | 0.8 | 16 |
| 1.00 | nothing | no fall | 0 of 10 | 16.8 / 15.8 | 9.43 / 9.28 | 77.4 | – |
| 1.00 | centring only | no fall | 0 of 10 | 14.3 / 17.9 | 5.50 / 8.46 | 48.8 | 5 |
| 1.00 | offset | no fall | 10 of 10 | 0.5 / 0.6 | 0.14 / 0.18 | 1.3 | 15 |
| 1.00 | passthrough | no fall | 10 of 10 | 0.5 / 0.4 | 0.10 / 0.10 | 0.6 | 16 |
| 1.50 | nothing | no fall | 0 of 8 | 15.2 / 16.3 | 8.71 / 11.20 | 77.0 | – |
| 1.50 | centring only | no fall | 0 of 8 | 12.0 / 17.4 | 6.27 / 9.91 | 56.2 | 6 |
| 1.50 | offset | no fall | 8 of 8 | 0.7 / 1.3 | 0.15 / 0.20 | 1.4 | 19 |
| 1.50 | passthrough | no fall | 8 of 8 | 0.7 / 1.3 | 0.10 / 0.12 | 0.5 | 13 |
| 2.00 | nothing | no fall | 0 of 8 | 15.0 / 17.3 | 9.03 / 12.06 | 77.1 | – |
| 2.00 | centring only | no fall | 0 of 8 | 12.2 / 17.3 | 6.89 / 10.95 | 60.5 | 7 |
| 2.00 | offset | no fall | 8 of 8 | 0.9 / 1.8 | 0.16 / 0.25 | 1.7 | 34 |
| 2.00 | passthrough | no fall | 8 of 8 | 1.0 / 1.8 | 0.14 / 0.21 | 1.1 | 40 |

![headline.png](figs/headline.png)

*Figure 1. Two-foot phase 1.0 s, four-foot phase 0.5 s. Without AMB the body tips until a swing foot is back on the ground (height 0) in every phase. With AMB and centring the swing feet stay up and the CoM stays on the support line.*

*Table 5. Variants of the 1.0 s / 0.5 s gait, AMB in offset mode.*

| Variant | Outcome | Two-foot phases held | Peak roll / pitch [deg] | Roll / pitch rms [deg] |
|---|---|---|---|---|
| as Table 4 | no fall | 10 of 10 | 0.5 / 0.6 | 0.14 / 0.18 |
| no centring | falls after 2.7 s | 0 of 2 | – | – |
| four-foot phase 0.3 s | no fall | 12 of 12 | 0.5 / 0.7 | 0.14 / 0.20 |
| four-foot phase 0.2 s | no fall | 13 of 13 | 0.8 / 1.0 | 0.18 / 0.23 |
| attitude integral off | no fall | 10 of 10 | 0.4 / 0.4 | 0.12 / 0.14 |
| attitude integral off, two-foot 1.5 s | no fall | 8 of 8 | 0.7 / 1.3 | 0.13 / 0.17 |
| position/velocity filter in the loop | no fall | 10 of 10 | 0.8 / 1.0 | 0.21 / 0.19 |
| position/velocity filter in the loop, two-foot 2.0 s, 60 s | no fall | 24 of 24 | 0.7 / 1.7 | 0.30 / 0.23 |
| ground truth, two-foot 2.0 s, 60 s | no fall | 24 of 24 | 0.9 / 1.8 | 0.11 / 0.16 |
| both filters in the loop | no fall | 10 of 10 | 1.3 / 1.7 | 0.44 / 0.44 |

The attitude integral does not disturb AMB: with it off the result is the same or slightly smoother. The filters in the loop (position/velocity alone is the simulator's default configuration) cost a factor of three to five in rms and no phase. A first version of the centring shifted the estimator-based position reference: on ground truth it gave the same results, with the position/velocity filter in the loop the 2.0 s gait fell after 17 s, because the position estimate is off by 1 to 3 cm at a touchdown of a gait this slow.

*Table 6. Lateral 0.1 s push at the base CoM in the middle of a two-foot phase (t = 11.304 s), in 10 N steps to the first fall.*

| Case | Largest push survived, +y [N] | Largest push survived, -y [N] | Note |
|---|---|---|---|
| nothing | 90 | 110 (sweep cap) | stands on three feet in every phase, pushed or not |
| offset | 20 | 40 | all phases held at the surviving force |
| passthrough | 20 | 20 |  |

A two-foot stand held by the swing legs has a small basin, a few N s. The trot without AMB shrugs off far larger pushes because it is already leaning on a third foot.

## 6. Slow forward trot: a zig-zag (first part of step 3)

A body that moves straight ahead has to cross each support line, and the swing legs absorb only about 3 mm of that: commanded forward with the in-place centring, the 1.0 s gait walked at 0.02 m/s and fell at 0.05 m/s. Two changes make it walk. The body follows the commanded velocity only along the current support line (which lies 37 deg off the forward axis, alternately to either side), and the MPC and the position task take that as their velocity reference. And the feet that lift are given touchdown points whose line passes under the CoM: the footstep law alone leads both feet straight ahead, which put the next line 3 cm in front of the CoM at 0.05 m/s, more than 0.5 s on four feet could cover.

*Table 8. Two-foot phase 1.0 s, four-foot phase 0.5 s, commanded forward. Zig-zag only: centring and foot placement on, the law off.*

| Commanded [m/s] | Case | Outcome | Achieved forward [m/s] | Two-foot phases held | Peak roll / pitch [deg] | Leg travel [deg] |
|---|---|---|---|---|---|---|
| 0.02 | nothing | falls after 10.0 s | 0.010 | 0 of 7 | – | – |
| 0.02 | zig-zag only | no fall | -0.031 | 0 of 10 | 14.4 / 16.8 | 5 |
| 0.02 | offset | no fall | 0.016 | 10 of 10 | 1.1 / 1.3 | 36 |
| 0.05 | nothing | falls after 4.3 s | – | 0 of 3 | – | – |
| 0.05 | zig-zag only | no fall | -0.013 | 1 of 10 | 11.0 / 14.9 | 4 |
| 0.05 | offset | no fall | 0.038 | 10 of 10 | 2.3 / 2.7 | 54 |
| 0.10 | nothing | falls after 3.1 s | – | 0 of 2 | – | – |
| 0.10 | zig-zag only | no fall | 0.078 | 0 of 10 | 16.5 / 15.7 | 6 |
| 0.10 | offset | no fall | 0.074 | 10 of 10 | 4.5 / 5.0 | 55 |
| 0.15 | nothing | falls after 3.6 s | – | 0 of 3 | – | – |
| 0.15 | zig-zag only | falls after 10.0 s | 0.094 | 1 of 7 | – | 8 |
| 0.15 | offset | no fall | 0.099 | 10 of 10 | 2.5 / 4.0 | 55 |
| 0.20 | nothing | falls after 3.8 s | – | 0 of 3 | – | – |
| 0.20 | zig-zag only | no fall | 0.036 | 0 of 10 | 14.6 / 18.4 | 10 |
| 0.20 | offset | falls after 4.3 s | – | 2 of 3 | – | 58 |

*Table 9. Variants, AMB with centring and foot placement.*

| Variant | Outcome | Achieved x / y [m/s], yaw [rad/s] | Two-foot phases held | Peak roll / pitch [deg] |
|---|---|---|---|---|
| two-foot 0.75 s, four-foot 0.2 s, 0.10 m/s | no fall | 0.073 / -0.000, 0.001 | 16 of 16 | 2.6 / 3.3 |
| four-foot 0.2 s, 0.10 m/s | no fall | 0.085 / -0.000, -0.000 | 12 of 13 | 13.3 / 13.4 |
| two-foot 2.0 s, 0.05 m/s | falls after 4.4 s | – | 1 of 2 | – |
| pass-through, 0.10 m/s | falls after 8.5 s | 0.071 / 0.033, -0.008 | 3 of 6 | – |
| position/velocity filter in the loop, 0.10 m/s | no fall | 0.070 / -0.008, 0.000 | 10 of 10 | 5.0 / 5.4 |
| both filters in the loop, 0.10 m/s | no fall | 0.072 / 0.003, -0.002 | 10 of 10 | 3.3 / 3.9 |
| lateral 0.05 m/s | no fall | 0.000 / 0.021, -0.000 | 10 of 10 | 0.7 / 1.0 |
| yaw 0.1 rad/s | no fall | 0.002 / 0.000, 0.100 | 10 of 10 | 0.7 / 0.8 |

The leg travel is at its bound from 0.05 m/s on: the forward stride of the two swing legs is itself a motion across the support line, and what the centring does not take up is left to the law. The speed is limited by the stride: the stance feet are 2 s on the ground at these phase lengths, so the body travels 2 s of its speed over them, and a 2.0 s two-foot phase at 0.05 m/s already falls. A lateral or a yaw command works the same way.

## 7. Baseline without AMB, and the touchdown depth (step 0)

*Table 7. The trot as it was before this work (AMB off, default swing profile, 8 mm touchdown depth), attitude integral on.*

| Two-foot phase [s] | four-foot 0.0 s | four-foot 0.2 s | four-foot 0.5 s |
|---|---|---|---|
| 0.50 | no fall, peak 2.6 / 1.8 deg | no fall, peak 6.3 / 5.8 deg | no fall, peak 8.8 / 6.6 deg |
| 0.75 | falls after 2.9 s | falls after 7.2 s | falls after 6.5 s |
| 1.00 | falls after 12.6 s | falls after 13.9 s | no fall, peak 22.3 / 14.6 deg |
| 1.50 | falls after 9.8 s | no fall, peak 16.7 / 15.3 deg | no fall, peak 16.9 / 15.7 deg |
| 2.00 | no fall, peak 25.4 / 14.8 deg | no fall, peak 19.1 / 15.1 deg | no fall, peak 17.2 / 15.6 deg |

At least one column falls at two-foot phases of 0.75, 1.00, 1.50 s; the runs that do not fall survive by resting on a swing foot. No phase in this table is a two-foot stand. Touchdown depth on the 1.0 s / 0.5 s gait with AMB: 3 mm: no fall, rms 0.47 / 0.47 deg; 8 mm: no fall, rms 0.16 / 0.20 deg; 12 mm: no fall, rms 0.14 / 0.18 deg. 12 mm is used.

## 8. Departures from the thesis and from the plan

1. Mode `offset` replaces Eqs. (6.23) and (6.28): the tangential references are the integrated motion, not the measured one, and the joint PD stays on. Its PD share of the tangential acceleration was 0.24 m/s<sup>2</sup> rms against 2.73 m/s<sup>2</sup> of feedforward on the headline gait.
2. The second diagonal is ordered RL to FR, not RF to LH as in Sec. 6.3.1.
3. CoM centring, the foot placement under the CoM and the swing lift time are not in the thesis. The centring has the role `MdlDiagonalStance` has before `MdlAMB`.
4. The template was not refitted for the trot posture: the centreline parameters held every case here.
5. The stop key is placed in a four-foot phase. A stop during a long two-foot phase can fall (seen after one static hold); the stop ramp takes AMB's authority away with the lift scale.

## 9. Open

1. Four-foot phases below 0.2 s and two-foot phases below 0.75 s were not run.
2. The small push basin, which is the swing legs' authority, and the pass-through's creep in long holds.
3. AMB at normal trot speed (second part of step 3): at the 0.3 s gait it is idle. Acting there needs a different form of the law, on the deviation of L from the gait's own profile.
4. The zig-zag reaches about 0.7 of the commanded speed and 0.10 m/s at most; the centring is still to be reworked (Ege, 2026-11-27: fine for now, to be fixed later).
5. Two-foot phases of 0.5 s: with a 0.2 s lift and descent AMB has no window left (`amb_min_swing`).
6. The position/velocity filter in very long phases (Table 3b): the estimate sinks and slides while the robot stands on four feet.

## 10. Reproduce

```bash
cd /home/ege/legged/iros/project
cmake --build build -j 12 && cmake --install build && (cd build && ctest)
/home/ege/legged/iros/.venv/bin/python figs/trot_amb/run_experiments.py     # all runs, six at a time
cd bin && ./sim.sh -c "trot.wbc = true" -c "trot.amb = true" -c "trot.amb_centre = true" \
  -c "trot.swing_lift_time = 0.2" -c "trot.wbc_touchdown_depth = 0.012" \
  -c "trot.forward_velocity = 0.0" -c "trot.period = 3.0" -c "trot.duty = 0.6666667"   # S, T, D
# forward: the same with trot.forward_velocity = 0.10
```
