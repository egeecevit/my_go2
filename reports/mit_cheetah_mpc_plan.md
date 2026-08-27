# Minimal MIT Cheetah 3 convex MPC implementation plan

Date: 2026-08-20
Target codebase: `quadcontrol`
Primary reference: *Dynamic Locomotion in the MIT Cheetah 3 through Convex
Model-Predictive Control* (Di Carlo et al., IROS 2018)

## Goal

Replace `MdlTrot`'s position-controlled, open-loop stance behavior with the
paper's convex MPC ground-reaction-force controller while retaining the
existing fixed-timing trot and foot trajectory. `MdlTrot` remains the behavior
coordinator:

- it owns gait phase, future contact timing, body references, and swing-foot
  references;
- one new `MdlConvexMPC` module maps the estimated body state and future gait
  data to four world-frame ground-reaction forces;
- `MdlTrot` uses Cartesian feedback for swing legs and maps MPC forces to motor
  torques for stance legs through `MdlLegControl`.

The implementation target is exclusively the existing Go2 MuJoCo simulation.
Do not add hardware enablement, hardware qualification, or support for another
robot model. Mass and inertia remain configuration data because they are inputs
to the MPC equations, not to create a hardware abstraction.

## Scope boundary

Implement only the controller shown in the paper's block diagram and equations
(1), (4), and (16)-(32):

1. the 13-state single-rigid-body model;
2. fixed-horizon condensed QP over per-foot 3D contact forces;
3. unilateral normal-force and square friction-pyramid constraints;
4. the first MPC force from each solution, held until the next solve;
5. Cartesian swing-foot PD from equation (1), with `tau_ff = 0` for this
   implementation;
6. stance torque generated from the MPC ground-reaction force.

Explicitly do **not** add whole-body control, terrain estimation, online contact
detection, other gaits, a generic footstep planner, sparse-MPC alternatives,
integral roll/pitch compensation, drag compensation, or Cheetah-Software's
legacy C wrappers. Hardware integration is also outside scope. Do not port
`RobotState`, `Gait`, `SolverMPC`, or
`convexMPC_interface` as separate layers; their required behavior fits in one
project-native module and the existing `TrotGait`.

This implementation excludes the inverse-dynamics swing feedforward in paper
equation (2) and the operational-inertia gain scheduling in equation (3).
Those require a full multibody dynamics model, while the feedback part of
equation (1) is complete with the repository's existing analytical leg
Jacobian and `tau_ff = 0`. Do not add Pinocchio or another dynamics dependency
to the current implementation.

## Implementation split (practice mode)

This plan is carried out under CLAUDE.md's implementation practice mode.
Claude writes everything around the core control math; three named chunks are
left for Ege.

Claude's scaffolding is: module lifecycle
(`init`/`uninit`/`activate`/`deactivate`/`update`), the
`setParams()`/`getParams()`/`reset()` hooks, configuration reading and
validation, build and framework wiring (`CONTROLSRC`, `ModuleConfig.hh`,
`CoreModules.cc`, qpOASES `FetchContent`, `list.toml`), the condensed
`A_qp`/`B_qp` stacking, the `H`/`g` cost, the force constraints, the qpOASES
call and first-force extraction, the motor reads/validation/emit path in
`MdlLegControl`, the `MdlTrot` horizon construction and dispatch skeleton,
logging, and every test in "Tests and acceptance criteria" — written in full.

Each chunk below is left as a `// TODO(ege): implement` stub that returns a
value of the correct shape and type, so the project builds, links, and runs
with all three empty. Above each stub sit two to four orientation bullets:
what has to happen, in what order, and the gotchas worth knowing — not
pseudocode. No reference implementation may be hidden anywhere: not in a
comment, not in a docstring, not in a fallback branch, not in a disabled code
path.

The tests are written in full and are the acceptance criteria. They are
expected to **fail** while the chunks are stubbed — `T_CHECK` prints and
`exit(1)`s, since `assert()` is compiled out in the default Release build —
and that failing-to-passing transition is the intended verification loop.

### Chunk A: dynamics construction in `MdlConvexMPC`

Stub boundary: one private function, e.g.
`_discretizeDynamics(yaw_mean, moment_arms[k]) -> A_d, B_d[k]`. The stub
returns `A_d = I13` and `B_d = 0`, which is a valid shape for the condensed
stacking around it. Orientation bullets:

- Fill only the nonzero blocks of `A_c` (13x13): the
  `Theta_dot = Rz(yaw)^T * omega_W` block, the `p_dot = v` identity block, and
  the gravity column that feeds `v_dot.z` from the 13th state. Everything else
  is zero.
- `B_c[k]` (13x12), per leg `i`: `I_W^-1 * [r_i]_x` into the omega rows and
  `(1/m) * I3` into the v rows, with `I_W = Rz(yaw) * I_B * Rz(yaw)^T`.
- Discretize with the augmented matrix exponential
  `exp(dt * [A_c B_c; 0 0])` (Eigen's unsupported `MatrixFunctions`,
  `.exp()`); do not substitute forward Euler.
- Gotchas: one `A_c` at the mean reference yaw, but a fresh `B_c[k]` for every
  horizon step; the gravity state carries the value `-g`, so the column signs
  must come out as `v_dot.z = ... + g_z`.

### Chunk B: equation (1) core in `MdlLegControl`

Stub boundary: the compute block of the pure helper described under "Small
`MdlLegControl` extension"; the motor reads, validation, and emit path around
it are Claude's. The stub returns a zero `tau_ff` and success, so the wrapper's
emit path still runs and a stubbed leg simply receives joint damping and no
torque. Orientation bullets:

- From the measured `q`: forward kinematics gives `p`, `jacobian()` gives `J`,
  and then `v = J*qdot` — never the IK-desired joint velocity.
- `f_cmd = kp.*(p_ref - p) + kd.*(v_ref - v) + f_ff`, all in body frame, and
  `tau_ff = J.transpose() * f_cmd`.
- Gotcha: `f_ff` is the force the *actuators apply at the foot* — the caller
  has already negated and rotated the ground-reaction force, so do not flip
  the sign again here.
- Reject non-finite results before anything is emitted.

### Chunk C: stance/ground-force dispatch in `MdlTrot`'s `TROT` state

Stub boundary: the stance branch body of the `TROT` leg dispatch. Claude
scaffolds the dispatch skeleton — the per-leg loop, the swing branch calling
chunk B's command with the existing references, Cartesian gains and zero
feedforward, the MPC cadence and contact-change solve triggering, and the
failure counters. The stub leaves the stance leg commanded with the same zero
feedforward as swing. Orientation bullets:

- Transform: `f_foot_B = -R_BW^T * f_grf_W`, then issue the Cartesian force
  command with zero Cartesian `kp`/`kd`, that feedforward, and the small
  positive joint damping.
- Order within a cycle: update swing references, solve the MPC if it is due or
  the contact mask changed, then command every leg exactly once.
- Gotchas: verify the stance sign with the static MuJoCo test before trusting
  it; on a solve failure hold the last valid force for at most the configured
  number of solves and then go to `ERROR` — never fall back to position stance.

## Existing project seams to preserve

The current module structure already provides the needed ownership chain:

```text
MdlOrientationEstimator ---\
                            +--> MdlTrot --> MdlConvexMPC::solve() --> f_GRF^W
MdlPosVelEstimator --------/          |                                  |
                                      |                                  v
TrotGait -----------------------------+--> swing/stance dispatch --> MdlLegControl --> MotorHW
```

- `Supervisor` grabs one behavior at a time.
- `MdlTrot` grabs all four `MdlLegControl` instances and will additionally grab
  `MdlConvexMPC`.
- Both estimators remain shared read-only sensor modules and run before the
  behavior.
- `MdlLegControl` remains the only layer that reads joint state and emits motor
  commands.
- `MdlTrot`'s `WAIT`, `PREP`, `TROT`, `CENTERING`, and `DONE` states remain.
- `TrotGait` remains the single source of current and future contact timing.

`MdlConvexMPC` should follow the same service-module pattern as
`MdlLegControl`: framework-managed lifecycle and ownership, an empty scheduled
`update()`, and a public synchronous operation. `MdlTrot` calls `solve()` only
at the configured MPC cadence or at a contact transition. This avoids an input
mailbox, a one-cycle result delay, and a circular scheduler dependency.

## Frames, signs, and state layout

Use this layout everywhere in `MdlConvexMPC`:

```text
x = [roll, pitch, yaw,
     p_COM_W.x, p_COM_W.y, p_COM_W.z,
     omega_W.x, omega_W.y, omega_W.z,
     v_COM_W.x, v_COM_W.y, v_COM_W.z,
     g_z]
```

- The project body frame is `+X` forward, `+Y` left, `+Z` up.
- `R_BW` below means the project's body-to-world rotation returned by
  `MdlOrientationEstimator::getRotation()`.
- The rigid-body state is at the whole-robot center of mass (COM), as in the
  paper. `MdlPosVelEstimator` estimates the body-frame origin, so `MdlTrot`
  must convert position and velocity using the configured nominal COM offset:
  `p_COM_W = p_body_W + R_BW*r_COM_B` and
  `v_COM_W = v_body_W + omega_W.cross(R_BW*r_COM_B)`.
- MPC forces `f_grf_W[i]` are ground forces **on the robot**, in world
  coordinates, so standing forces have positive `z` and appear directly in the
  rigid-body dynamics.
- `QuadrupedKinematics::jacobian()` maps joint rate to foot velocity relative
  to the body: `v_foot_B = J * qdot`.
- The Cartesian feedforward field used by the leg command represents force the
  actuators apply at the foot. For stance, pass
  `f_foot_B = -R_BW.transpose() * f_grf_W` before applying
  `tau = J.transpose() * f_foot_B`, matching the public Cheetah controller's
  force-command convention. Confirm and lock this sign with a MuJoCo test.

Keep all QP dynamics and forces in world coordinates. Perform only the final
force transform at the leg-command boundary.

## New module: `MdlConvexMPC`

Add:

- `include/quadruped/MdlConvexMPC.hh`
- `src/quadruped/MdlConvexMPC.cc`
- `config/default/mpc.toml`
- `src/quadruped/tests/test_convex_mpc.cc`

### Minimal public interface

Use fixed-size input/output records and a fixed first-pass horizon of 10 steps:

```cpp
class MdlConvexMPC : public rtcore::Module {
 public:
  static constexpr int NUM_LEGS = 4;
  static constexpr int HORIZON = 10;

  struct body_state_t {
    Eigen::Vector3d rpy;
    Eigen::Vector3d com_position_world;
    Eigen::Vector3d angular_velocity_world;
    Eigen::Vector3d com_velocity_world;
  };

  struct input_t {
    body_state_t current;
    body_state_t reference[HORIZON];
    Eigen::Matrix<double, 3, NUM_LEGS> moment_arm_world[HORIZON];
    bool contact[HORIZON][NUM_LEGS];
  };

  struct output_t {
    Eigen::Vector3d force_world[NUM_LEGS];
    bool valid;
    int solver_status;
  };

  bool solve(const input_t& input, output_t& output);
};
```

Also expose the project-standard `setParams()`, `getParams()`, and `reset()`
hooks so the solver can be unit-tested without a `ModuleManager`. `MdlTrot`
reads only `dt` and `com_offset_body` through the const parameter accessor when
building its horizon.

Keep solver matrices and qpOASES storage private and allocate/resize them once
in `init()`. `activate()` clears solver warm-start/result state. `solve()` must
reject non-finite input and must publish a new output only after qpOASES returns
a successful solution whose first force block is finite.

### Continuous dynamics

For each prediction step, form the paper's 13-state model:

```text
Theta_dot = Rz(yaw)^T * omega_W
p_dot     = v_W
omega_dot = I_W^-1 * sum_i (r_i x f_i)
v_dot     = (1/m) * sum_i f_i + [0, 0, g_z]
g_z_dot   = 0
```

with

```text
I_W = Rz(yaw) * I_B * Rz(yaw)^T.
```

As in paper Section IV-C, use one `A_c` evaluated at the mean reference yaw and
a `B_c[k]` evaluated from the planned moment arms at each horizon step. Use the
augmented matrix exponential for a zero-order-hold discretization; do not
replace it with forward Euler:

```text
exp(dt_mpc * [A_c  B_c[k]; 0  0]) -> A_d, B_d[k].
```

The gravity state is initialized and referenced as `-gravity_magnitude`.

### Condensed QP

Stack states `x_1 ... x_N` and forces `u_0 ... u_(N-1)`, with every `u_k`
containing `[fx, fy, fz]` for FL, FR, RL, and RR. Build the linear time-varying
condensed dynamics recursively:

```text
X = A_qp * x_0 + B_qp * U.
```

Then solve the paper's dense QP:

```text
min  0.5 * U^T H U + g^T U

H = 2 * (B_qp^T L B_qp + alpha * I)
g = 2 * B_qp^T L * (A_qp*x_0 - X_ref).
```

`L` repeats the configured 12 state weights and gives the gravity state zero
weight. Symmetrize `H` before passing it to the solver. Positive `alpha` is
mandatory.

For a stance foot, impose:

```text
f_min <= fz <= f_max
-mu*fz <= fx <= mu*fz
-mu*fz <= fy <= mu*fz.
```

For a swing foot, set the variable bounds of all three components to exactly
zero. Keep the full fixed-size force vector instead of adding the paper/source
optimization that removes swing variables; at a 10-step horizon this saves code
and keeps the matrix structure constant.

Return only `u_0`, split into four world-frame forces. Do not filter the result.

### Solver dependency

Use qpOASES because it is the solver named by the paper and used by the public
Cheetah controller. Use CMake `FetchContent` to download the official
`coin-or/qpOASES` release `releases/3.2.2` during normal `cmake ..`
configuration, disable its examples, and link the `quadruped` target
transitively to `qpOASES`. Pin the exact release so configuration is
reproducible; do not add a system-package branch or require a preinstalled QP
solver. A network/download failure must stop configuration with a clear error.

The pinned qpOASES release declares CMake 3.18, so raise this project's current
3.5 minimum to 3.18 as part of the dependency integration.

Use one persistent `SQProblem` so changing `H`, gradient, and bounds can reuse
the previous active set. Do not add another solver abstraction or a second QP
backend.

### Configuration

`[mpc]` should contain only quantities that change the mathematical problem or
the selected robot:

```toml
[mpc]
dt = 0.04                    # 10 steps -> 0.4 s horizon, 25 Hz nominal solve
mass = <Go2 aggregate mass>
com_offset_body = [<nominal whole-robot COM relative to body origin>]
inertia_body = [<3x3 row-major aggregate inertia about COM>]
gravity = 9.81
friction = 0.6
force_min = 0.0
force_max = <Go2 limit>
force_weight = 1.0e-6
state_weights = [1, 1, 1, 0, 0, 50, 0, 0, 1, 1, 1, 1]
```

The state weights and force weight above are the paper's Table I baseline in
this plan's state order. Do **not** copy Cheetah 3's 43 kg mass, inertia, or 666
N force limit into a Go2 controller. Source the aggregate mass, COM offset, and
inertia about that COM by summing and composing the 13 `<inertial>` elements of
`models/unitree_go2/go2.xml` (each carries `pos`, `quat`, `mass`, and
`diaginertia`; the trunk is 6.921 kg) at the nominal standing configuration,
and record the values with a comment citing that file as their source.
`force_max` must be chosen against the simulated Go2 actuator limits and
checked in MuJoCo.

Add `%include mpc.toml` to `config/default/list.toml`. Treat a missing/invalid
mass, COM offset, inertia, force range, friction, or weight array as an
initialization error rather than silently running a physically wrong model.

Register only compact diagnostics: the 12 first-step forces, solver status, and
solve time. No MPC trajectory visualization is needed.

## Small `MdlLegControl` extension

Preserve the existing IK/joint-PD `setFootCommand()` for stand, sit, prep,
centering, and other behaviors. Add one measured-state Cartesian force command
for `MdlTrot`:

```cpp
bool setCartesianForceCommand(
    const Eigen::Vector3d& position_ref_body,
    const Eigen::Vector3d& velocity_ref_body,
    const Eigen::Vector3d& kp_cartesian,
    const Eigen::Vector3d& kd_cartesian,
    const Eigen::Vector3d& force_feedforward_body,
    double joint_damping);
```

Split it into a hardware-free helper plus a thin module wrapper, mirroring the
estimator modules' testable `step()` pattern, because the repository has no
mocking infrastructure of any kind and a `MotorHW` cannot be faked. The pure
helper takes measured state and gains and returns the feedforward torque:

```cpp
bool computeCartesianForceCommand(
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& qdot,
    const Eigen::Vector3d& position_ref_body,
    const Eigen::Vector3d& velocity_ref_body,
    const Eigen::Vector3d& kp_cartesian,
    const Eigen::Vector3d& kd_cartesian,
    const Eigen::Vector3d& force_feedforward_body,
    Eigen::Vector3d& tau_ff) const;
```

It uses only the private `QuadrupedKinematics` member, touches no hardware, and
is therefore directly unit-testable. Its complete operation is:

1. compute measured `p` and `J` from `q`, and `v = J*qdot`;
2. compute `f_cmd = kp.*(p_ref-p) + kd.*(v_ref-v) + f_feedforward`;
3. compute `tau_ff = J.transpose()*f_cmd` and reject non-finite results.

`setCartesianForceCommand()` is then a thin wrapper that:

1. requires all three motors to be ready, and reads measured `q` and `qdot`
   through `MotorHW::getState(3*leg+j, ...)`, following
   `MdlPosVelEstimator::_readSensors`;
2. delegates to the helper;
3. emits the current measured joint positions, zero desired joint velocity,
   zero joint `kp`, a small positive joint `kd`, and `tau_ff` through the
   existing `setJointCommand()` path.

The motor-ready check is **new behavior** for a setter: today only
`getFootPosition()` checks readiness, and the existing `MdlLegControl` setters
do not. Add it deliberately rather than copying an existing pattern.

The small positive joint damping is required because `MdlSimDriver` substitutes
`kd=5` whenever a non-positive value is sent, and it does so by *mutating the
stored command* (`src/hardware/mujocohw/MdlSimDriver.cc:767`), so the
substitution is sticky and cannot be undone by a later command that omits `kd`.
Default it to the public Cheetah controller's `0.2`, but keep it in `[trot]`
for robot tuning.

This one method implements both control modes without an additional leg
controller:

- swing: Cartesian `kp/kd`, zero feedforward force;
- stance: zero Cartesian `kp/kd`, MPC-derived feedforward force.

## `MdlTrot` changes

### Dependencies and lifecycle

- Look up both `MdlOrientationEstimator` and `MdlPosVelEstimator`, plus
  `MdlConvexMPC`, in `init()`.
- Grab `MdlConvexMPC` together with the legs in `activate()` and release it in
  `deactivate()` and every activation-error cleanup path.
- Do not enter torque-controlled `TROT` until both estimators are ready and the
  first MPC solve succeeds.
- Keep `WAIT`, `PREP`, `CENTERING`, and `DONE` on the existing IK/joint-PD path.
  Only the `TROT` state's leg dispatch changes.
- On entry to `CENTERING`, use each leg's measured foot position as the blend
  start so the switch back from force control does not reuse a stale trajectory
  reference.
- Incidentally fix the stale `MdlStateEstimator` reference in
  `include/quadruped/MdlTrot.hh:109` while editing the dependency block; the
  matching stale references in `config/default/trot.toml` disappear when that
  file is rewritten, which is already in scope.

### Body reference

At `_trotEntry()`:

- convert the estimated body-origin state to COM position/velocity;
- initialize desired world `x/y/z` from that COM position after `PREP`;
- initialize desired yaw from the estimated unwrapped yaw;
- clear MPC cadence, contact-mask, and failure counters.

Convert the measured body-origin state to the COM state before filling
`input.current`. Hold the entry COM height as the horizon's desired `z`; this
preserves the height reached by the existing nominal-footprint `PREP` without
duplicating foot-radius or ground-height assumptions. This keeps translation
and moment arms about the point for which `inertia_body` is defined.

`_vcmd` is a body-frame forward/lateral command. During `TROT`, rotate it by the
desired yaw to obtain the world velocity, then integrate that velocity and the
yaw rate at the 1 kHz behavior rate. Before constructing an MPC horizon, clamp
desired `x/y` to a small configurable distance (initially 0.1 m) from the
current estimate so a slip or disturbance cannot leave an indefinitely growing
position error. Build each horizon reference as:

```text
roll, pitch                 = 0
yaw                         = integrated yaw
x, y                        = integrated commanded velocity
z                           = COM height captured at TROT entry
roll rate, pitch rate       = 0
yaw rate                    = commanded yaw rate
vx, vy                      = commanded world velocity
vz                          = 0
```

Unwrap current and reference yaw onto the same continuous branch before placing
them in the QP. Since the condensed state vector is `x_1 ... x_N`,
`reference[k]` is the desired state at `now + (k+1)*mpc.dt`; do not place the
current state in the first reference slot.

### Contact and moment-arm horizon

Do not port Cheetah's `Gait` class. For control interval `k`, query the existing
`TrotGait` at `elapsed + k*mpc.dt`:

- `inStance()` supplies `contact[k][leg]`;
- the planned body-frame foot position is assembled inside `MdlTrot`, which
  owns both pieces: `TrotGait::sample()` returns a *delta* from the nominal
  footprint, and the nominal footprint comes from `MdlTrot`'s own private
  `_originFoot(leg)` — not from `TrotGait`. The planned position is therefore
  `_originFoot(leg) + dp`, the same composition already used at
  `src/quadruped/MdlTrot.cc:443`;
- subtract the configured body-frame COM offset, then rotate by the step's
  reference yaw to obtain the COM-to-foot world moment arm used in `B_c[k]`.

`TrotGait` itself is query-pure at an arbitrary `t` — `inStance()`, `sample()`,
and `legPhase()` all wrap the argument and mutate no state — so evaluating it
at `elapsed + k*mpc.dt` for the whole horizon is safe.

For `k=0`, overwrite the moment arm of each currently contacting foot with the
measured COM-to-foot vector rotated into world coordinates. This keeps the
first dynamics matrix correct after tracking error or a disturbance, as
required by paper Section IV-C. Future steps use the gait reference.

Use estimated horizontal CoM velocity for the existing touchdown placement
geometry, matching paper equation (33); use commanded velocity in the future
body-state reference. The existing optional open-loop stance-velocity feedback
block then becomes obsolete and should be removed rather than left as a second
body feedback path. Retain only a light velocity low-pass if needed to prevent
the swing target from moving with every 1 kHz estimator oscillation.

Solve every `mpc.dt` and immediately whenever the current four-leg contact mask
changes. The latter prevents a new stance leg from waiting up to one full MPC
interval for a valid force when non-grid-aligned gait parameters are used.

### Leg dispatch in `TROT`

After the current swing references and any new MPC result are ready, command
each leg once:

```text
if swing:
    Cartesian force command(
        existing foot position/velocity reference,
        configured swing Cartesian kp/kd,
        zero feedforward force)

if stance:
    f_foot_B = -R_BW^T * f_grf_W
    Cartesian force command(
        unused/current foot reference,
        zero Cartesian kp/kd,
        f_foot_B)
```

This is a change of units, not merely a rename. Only `p` and `pdot` are
foot-space in the existing `setFootCommand()`; its `kp`, `kd`, and `tau_ff` are
joint-space, so today's `stance_kp`/`swing_kp` in `[trot]` are joint gains
(`Nm/rad`, `Nm/(rad/s)`). The new swing gains are Cartesian (`N/m`,
`N/(m/s)`); rename them so the units are explicit in the key names, and do not
carry the old numeric values over. Replace the stance position gains with the
single small joint-damping value.

The current joint-command tracking-error check remains valid in `WAIT`, `PREP`,
and `CENTERING`, but not in direct-torque `TROT`; there, use motor-ready,
finite-command, and MPC-solver checks instead.

On a solve failure, retain the last valid force for no more than a small
configured number of MPC solves. A failed first solve or repeated failures set
`MdlTrot::ERROR`; do not fall back silently to the old open-loop stance
controller.

## Framework and build wiring

Make the following mechanical updates:

1. Add `MdlConvexMPC.cc` to `CONTROLSRC` and link `quadruped` to qpOASES.
2. Add `MdlConvexMPC` to `moduleConfig` in `ModuleConfig.hh` as
   `{{"MdlConvexMPC", -1}, {1, 0, BEHAVIORAL_CONTROLLERS}}`, placed next to
   `MdlTrot`. Its scheduled update is a no-op, so the order value is irrelevant
   and no leg-module order changes are needed; the entry exists only because
   `CREATE_MODULE` fatal-errors on a name missing from that table.
3. Create it before `MdlTrot` in `AddCoreModules()` so `MdlTrot::init()` can find
   it.
4. Deactivate/destroy `MdlTrot` before `MdlConvexMPC` so the behavior releases
   ownership first.
5. Include `mpc.toml` from `config/default/list.toml`.
6. Update `config/default/trot.toml` to describe closed-loop MPC stance and the
   Cartesian swing gains; remove the old open-loop/velocity-feedback claims.

No `Supervisor` state or key change is required.

## Implementation order

Each step is annotated with ownership: *scaffold* is written by Claude,
*TODO chunk* is left stubbed for Ege.

### 1. Solver module in isolation

- Add qpOASES build integration. (scaffold)
- Implement `MdlConvexMPC` parameters, condensed matrices, force constraints,
  cost assembly, the qpOASES call, and first-force extraction. (scaffold)
- Continuous `A_c`/`B_c[k]` and the zero-order-hold matrix-exponential
  discretization. (**TODO chunk A**)
- Make the math callable without running MuJoCo, following the estimator
  modules' testable `setParams()`/`solve()` pattern. (scaffold)
- Add deterministic QP tests before connecting any motor command; they are
  written in full and fail until chunk A is implemented. (scaffold)

### 2. Cartesian leg command

- Add the pure `computeCartesianForceCommand()` helper and the
  `setCartesianForceCommand()` wrapper — motor-ready check, motor reads,
  validation, emit path — to `MdlLegControl`. (scaffold)
- The equation (1) compute block inside the helper. (**TODO chunk B**)
- Test measured-state `p`, `v`, `J`, Cartesian feedback, non-finite rejection,
  and `J^T f` torque mapping against the pure helper. (scaffold)
- Verify the stance force sign with a static MuJoCo test: positive optimizer
  normal force must raise/support the body, not pull it downward. (scaffold;
  meaningful only once chunks B and C are filled in)

### 3. Module wiring

- Add the module configuration and `CoreModules` lifecycle entries. (scaffold)
- Add `[mpc]` loading and configuration-layering coverage. (scaffold)
- Confirm `MdlTrot` can grab/release the solver with no ownership leak on every
  failure path. (scaffold)

### 4. Close `MdlTrot`

- Add estimator/MPC dependencies and body-reference state. (scaffold)
- Build the fixed contact/moment-arm/reference horizon from `TrotGait`.
  (scaffold)
- Run MPC at its cadence and at contact changes. (scaffold)
- Dispatch the swing branch through the Cartesian force command. (scaffold)
- The stance branch of the `TROT` dispatch: ground-force transform and stance
  force command. (**TODO chunk C**)
- Remove the obsolete open-loop stance-velocity feedback path and update the
  behavior comments/configuration. (scaffold)

### 5. Simulation validation

This step is gated on all three TODO chunks being filled in; with the stubs in
place the build runs but the robot does not stand up under MPC.

- Build and run all unit tests.
- In MuJoCo, exercise `stand -> trot -> stop -> sit` first at zero commanded
  horizontal velocity, then at the existing 0.15 m/s command.
- Inspect estimated pose, MPC force, contact mask, joint torque, solver status,
  and solve time before changing gains or speed.
- Increase simulated speed only after the stationary and slow-trot cases pass.

## Tests and acceptance criteria

### `test_convex_mpc`

1. **Static symmetric support:** with level pose, zero desired motion, symmetric
   feet, and four contacts, the solution is finite, respects all constraints,
   produces near-zero horizontal resultant/moment, and approximately balances
   weight.
2. **Diagonal support:** with FL/RR contact, FR/RL forces are exactly zero and
   the active forces remain within normal/friction bounds.
3. **Reference correction direction:** height, roll, pitch, and velocity
   perturbations produce a resultant force/moment that opposes the error.
4. **Horizon contact transition:** the first output follows interval zero, with
   no one-step shift in the gait table.
5. **Numerics:** `H` is finite and symmetric, repeated warm-started solves
   succeed, and invalid input does not overwrite the last valid result.

Cases 1-4 depend on the discretized dynamics and are therefore expected to fail
against the chunk-A stub (`A_d = I13`, `B_d = 0`); they are Ege's acceptance
criteria for that chunk. Case 5 exercises only Claude's scaffolding and should
pass from the start.

### Leg-control tests

The repository has no mocking infrastructure and no fake `MotorHW`, so these
tests target the pure `computeCartesianForceCommand()` helper directly, with
the test binary declaring `HARDWARE_IMPL(ClockHW/MotorHW/IMUHW)` null
singletons exactly as `src/quadruped/tests/test_trot_gait.cc:36` already does
to satisfy the link. The wrapper's motor-ready and emit behavior is covered in
simulation, not by a mock.

1. `v = J*qdot` is used rather than desired-joint velocity from IK.
2. The returned torque equals `J.transpose()*f_cmd` for all four leg
   conventions.
3. Zero Cartesian gains leave only feedforward torque, so the emitted command
   carries only `tau_ff` and the configured joint damping.
4. Failed kinematics and non-finite inputs reject the command without
   partially updating a leg; the non-ready-motor rejection is covered by
   inspection of the wrapper.

These are also written in full and fail against the chunk-B stub, which
returns zero torque.

### Existing tests

- Extend `test_trot_gait` only for future contact samples and transition
  alignment; keep all current phase/trajectory continuity checks.
- Extend `test_config_layering` to require the `[mpc]` table in each supported
  configuration chain.
- Run the full simulation build and `ctest --output-on-failure`.

### Simulation acceptance

- No NaN/Inf motor command or MPC force.
- Swing legs always receive zero MPC feedforward force; non-contact QP forces
  are zero to solver tolerance.
- Stance forces obey configured friction and normal-force limits.
- The first MPC solve succeeds before position-controlled stance is disabled.
- Solver time remains below `mpc.dt`, with no sustained solver failures.
- At zero velocity command the body height and roll/pitch remain bounded; at
  0.15 m/s the estimated horizontal velocity follows the command materially
  better than the current position-controlled stance.
- Stop/centering/sit remains functional after torque-controlled trotting.

## Confirmed decisions

1. **Target:** Go2 MuJoCo simulation only; hardware and other robot models are
   outside scope.
2. **QP dependency:** fetch pinned qpOASES `releases/3.2.2` automatically during
   normal CMake configuration.
3. **Swing control:** implement equation (1) with the existing analytical
   measured-state leg Jacobian and `tau_ff = 0`. Do not add Pinocchio, equation
   (2) feedforward, or equation (3) gain scheduling to this implementation.
4. **Practice-mode split:** Claude scaffolds everything and writes all tests in
   full; three chunks are left as `// TODO(ege): implement` stubs with
   orientation bullets, with these user-confirmed boundaries:
   - chunk A is the continuous `A_c`/`B_c[k]` **and** the zero-order-hold
     matrix-exponential discretization in `MdlConvexMPC`; the condensed
     `A_qp`/`B_qp` stacking, `H`/`g` cost, constraints, qpOASES call, and
     first-force extraction are scaffolding;
   - chunk B is the equation (1) core — `f_cmd` and `tau = J^T f_cmd` — inside
     `MdlLegControl`'s pure Cartesian-force helper; motor reads, validation,
     and the emit path are scaffolding;
   - chunk C is the stance/ground-force branch of `MdlTrot`'s `TROT` dispatch;
     the swing branch and the solve triggering are scaffolding.

## Future work: full-dynamics swing control

Outside the scope of this implementation, swing control can later be derived
from a full multibody dynamics model instead of using feedback alone. That
future extension should:

1. add a single shared dynamics helper, separate from `MdlConvexMPC`, using a
   model library such as Pinocchio or an equivalent selected at that time;
2. use model-consistent foot Jacobian, joint-space inertia, Coriolis/gravity,
   and `Jdot*qdot` terms to implement equation (2);
3. extend `TrotGait` with analytical swing-foot reference acceleration rather
   than numerically differentiating its velocity;
4. compute operational-space inertia `Lambda` and apply equation (3)'s
   configuration-dependent swing gains;
5. verify joint ordering, signs, frames, inertial parameters, and computed
   torques against the Go2 MuJoCo model.

This future dynamics work must remain independent of the centroidal MPC and
must not introduce whole-body control into `MdlTrot`.

## References used

- Local paper:
  `/home/arch-ege/Downloads/Dynamic_Locomotion_in_the_MIT_Cheetah_3_Through_Convex_Model-Predictive_Control.pdf`,
  especially Sections II-D/E, III, IV, and V-A.
- MIT controller orchestration and swing/stance dispatch:
  [ConvexMPCLocomotion.cpp](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/user/MIT_Controller/Controllers/convexMPC/ConvexMPCLocomotion.cpp).
- MIT fixed-timing contact horizon:
  [Gait.cpp](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/user/MIT_Controller/Controllers/convexMPC/Gait.cpp).
- MIT condensed dense QP implementation:
  [SolverMPC.cpp](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/user/MIT_Controller/Controllers/convexMPC/SolverMPC.cpp).
- MIT Cartesian feedback/feedforward force mapping through `J^T`:
  [LegController.cpp](https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/src/Controllers/LegController.cpp).
- Official qpOASES source:
  [coin-or/qpOASES](https://github.com/coin-or/qpOASES/tree/releases/3.2.2).
