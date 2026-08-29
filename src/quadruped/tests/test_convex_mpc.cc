/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

// Tests for the convex MPC ground reaction force solver.
//
// Every case here is posed as a physics question rather than as a comparison
// against a stored answer: a stored answer would pin the solver's arithmetic
// without saying anything about whether the forces make sense, and the whole
// point of the module is that the forces make sense. So the checks are that the
// robot is held up, that airborne feet push on nothing, that an error produces
// a correction opposing it, and that the schedule is read at the right offset.
//
// Cases 1 to 4 all depend on the discretized rigid body dynamics and therefore
// FAIL until _discretizeDynamics() is implemented; against its stub (A_d = I,
// B_d = 0) the QP has no way for a force to influence a state, so the optimum is
// the zero force vector. That failure is the acceptance criterion for that
// chunk, not a defect in these tests. Case 5 exercises only the surrounding
// machinery and passes from the start.
//
// The solver is driven through setParams()/reset()/solve() with no
// ModuleManager, the same way test_state_estimator drives the Kalman filter.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "hardware/IMUHW.hh"
#include "hardware/MotorHW.hh"
#include "quadruped/MdlConvexMPC.hh"
#include "rtcore/ClockHW.hh"
#include "rtcore/Hardware.hh"

using namespace rtcore;

// Linking pulls in the whole MdlConvexMPC translation unit, whose module layer
// references the hardware singletons. Defining their statics satisfies the
// linker; none of them is ever instantiated here, so instance() returns null.
HARDWARE_IMPL(ClockHW);
HARDWARE_IMPL(MotorHW);
HARDWARE_IMPL(IMUHW);

// Name of the case currently running, so a failure line says which one it was
// without the reader having to scroll back.
static const char* g_case = "(none)";

// NDEBUG-proof check: the main build is Release, where assert() vanishes.
#define T_CHECK(cond)                                                              \
  do {                                                                             \
    if (!(cond)) {                                                                 \
      std::cerr << "FAIL [" << g_case << "] " << __FILE__ << ":" << __LINE__       \
                << ": " #cond << std::endl;                                        \
      std::exit(1);                                                                \
    }                                                                              \
  } while (0)

#define T_NEAR(a, b, tol)                                                          \
  do {                                                                             \
    const double _d = std::fabs((a) - (b));                                        \
    if (!(_d <= (tol))) {                                                          \
      std::cerr << "FAIL [" << g_case << "] " << __FILE__ << ":" << __LINE__       \
                << ": |" #a " - " #b "| = " << _d << " > " << (tol) << std::endl;  \
      std::exit(1);                                                                \
    }                                                                              \
  } while (0)

// ---------------------------------------------------------------------------
//  Fixture
// ---------------------------------------------------------------------------

// The horizon these cases are written against. It is a run-time parameter now,
// so this is the value go2Params() installs rather than a property of the class,
// and the two are tied together below.
static constexpr int H = 10;
static constexpr int NL = MdlConvexMPC::NUM_LEGS;

// The shipped Go2 numbers from config/default/mpc.toml. Duplicated rather than
// loaded so this test needs no configuration search path; if the two ever
// diverge that is a bookkeeping problem, not a wrong test, since nothing below
// depends on the exact values.
static MdlConvexMPC::params_t go2Params() {
  MdlConvexMPC::params_t p;
  p.dt = 0.04;
  p.horizon = H;
  p.mass = 15.2064;
  p.com_offset_body = Eigen::Vector3d(-0.00172, 0.0, -0.02227);
  p.inertia_body << 0.177718, 0.000122, -0.016809,
                    0.000122, 0.488065, -0.000031,
                   -0.016809, -0.000031, 0.535074;
  p.gravity = 9.81;
  p.friction = 0.6;
  p.force_min = 0.0;
  p.force_max = 200.0;
  p.force_weight = 1.0e-6;
  const double w[12] = {1, 1, 1, 0, 0, 50, 0, 0, 1, 1, 1, 1};
  for (int i = 0; i < 12; i++) p.state_weights[i] = w[i];
  // Matches mpc.toml. It has to: the warm-start case below alternates the
  // support diagonal on *every* horizon step between one solve and the next, so
  // all 120 decision variables change bound status at once and qpOASES needs at
  // least that many working set changes to follow. Measured minimum for the
  // sequence is 125; a real trot schedule, which shifts by one step per solve,
  // needs 71.
  p.max_working_set = 250;
  return p;
}

// Nominal foot positions in the body frame: the trot.toml footprint, hips at
// +-0.1934 / +-0.0465 shifted by [0, +-0.10, -0.28]. Leg order FL, FR, RL, RR.
static Eigen::Vector3d nominalFoot(int leg) {
  const double x = (leg < 2) ? 0.1934 : -0.1934;
  const double y = (leg % 2 == 0) ? 0.1465 : -0.1465;
  return Eigen::Vector3d(x, y, -0.28);
}

static const double kStandHeight = 0.28;

// A level robot standing still on all four feet, with the reference equal to
// the current state, which is the "nothing to correct" baseline every other
// case perturbs.
static void makeStandingInput(const MdlConvexMPC::params_t& p,
                              MdlConvexMPC::input_t& in) {
  in.current.rpy.setZero();
  in.current.com_position_world = Eigen::Vector3d(0.0, 0.0, kStandHeight);
  in.current.angular_velocity_world.setZero();
  in.current.com_velocity_world.setZero();

  // p.horizon, not H: this helper is shared with the case that varies the
  // horizon, and filling a fixed ten steps would leave the rest of the input
  // uninitialized for any longer one.
  for (int k = 0; k < p.horizon; k++) {
    in.reference[k] = in.current;
    for (int leg = 0; leg < NL; leg++) {
      // Attitude is level and yaw is zero, so the world moment arm is just the
      // body one measured from the COM rather than from the frame origin.
      in.moment_arm_world[k].col(leg) = nominalFoot(leg) - p.com_offset_body;
      in.contact[k][leg] = true;
    }
  }
}

static Eigen::Vector3d resultantForce(const MdlConvexMPC::output_t& out) {
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  for (int leg = 0; leg < NL; leg++) f += out.force_world[leg];
  return f;
}

static Eigen::Vector3d resultantMoment(const MdlConvexMPC::input_t& in,
                                       const MdlConvexMPC::output_t& out) {
  Eigen::Vector3d m = Eigen::Vector3d::Zero();
  for (int leg = 0; leg < NL; leg++)
    m += Eigen::Vector3d(in.moment_arm_world[0].col(leg)).cross(out.force_world[leg]);
  return m;
}

// Unilateral normal force plus the square friction pyramid, which is the whole
// contract the solver has with physics.
static void checkContactConstraints(const MdlConvexMPC::params_t& p,
                                    const MdlConvexMPC::input_t& in,
                                    const MdlConvexMPC::output_t& out) {
  const double tol = 1e-6;
  for (int leg = 0; leg < NL; leg++) {
    const Eigen::Vector3d& f = out.force_world[leg];
    T_CHECK(f.allFinite());
    if (!in.contact[0][leg]) {
      // A foot that is not on the ground pushes on nothing at all, and the
      // bound that says so is an equality, so this is exact up to the solver's
      // own tolerance rather than approximate.
      T_NEAR(f.x(), 0.0, 1e-9);
      T_NEAR(f.y(), 0.0, 1e-9);
      T_NEAR(f.z(), 0.0, 1e-9);
      continue;
    }
    T_CHECK(f.z() >= p.force_min - tol);
    T_CHECK(f.z() <= p.force_max + tol);
    T_CHECK(std::fabs(f.x()) <= p.friction * f.z() + tol);
    T_CHECK(std::fabs(f.y()) <= p.friction * f.z() + tol);
  }
}

// ---------------------------------------------------------------------------
//  1. Static symmetric support
// ---------------------------------------------------------------------------

void test_static_support() {
  g_case = "static_support";
  std::cout << "test_static_support..." << std::endl;

  const MdlConvexMPC::params_t p = go2Params();
  MdlConvexMPC mpc;
  mpc.setParams(p);
  T_CHECK(mpc.reset());

  MdlConvexMPC::input_t in;
  makeStandingInput(p, in);

  MdlConvexMPC::output_t out;
  T_CHECK(mpc.solve(in, out));
  T_CHECK(out.valid);

  checkContactConstraints(p, in, out);

  const double weight = p.mass * p.gravity;
  const Eigen::Vector3d f = resultantForce(out);
  const Eigen::Vector3d m = resultantMoment(in, out);

  // Standing still means the feet carry the weight and nothing else. The
  // tolerances are loose because this is a finite-horizon optimum with a force
  // penalty, not an equality-constrained balance: what is being tested is that
  // the solution is the physics, not that it is any particular number.
  T_NEAR(f.z(), weight, 0.15 * weight);
  T_NEAR(f.x(), 0.0, 2.0);
  T_NEAR(f.y(), 0.0, 2.0);
  T_NEAR(m.x(), 0.0, 2.0);
  T_NEAR(m.y(), 0.0, 2.0);
  T_NEAR(m.z(), 0.0, 2.0);

  // Every foot is actually carrying something. A solution that put the whole
  // weight on one diagonal would satisfy everything above and be useless.
  for (int leg = 0; leg < NL; leg++) T_CHECK(out.force_world[leg].z() > 0.1 * weight / 4.0);

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  2. Diagonal support
// ---------------------------------------------------------------------------

void test_diagonal_support() {
  g_case = "diagonal_support";
  std::cout << "test_diagonal_support..." << std::endl;

  const MdlConvexMPC::params_t p = go2Params();
  MdlConvexMPC mpc;
  mpc.setParams(p);
  T_CHECK(mpc.reset());

  MdlConvexMPC::input_t in;
  makeStandingInput(p, in);

  // FL and RR down, FR and RL airborne, for the whole horizon: the support
  // condition in the middle of a trot's stance interval.
  for (int k = 0; k < H; k++) {
    in.contact[k][0] = true;
    in.contact[k][1] = false;
    in.contact[k][2] = false;
    in.contact[k][3] = true;
  }

  MdlConvexMPC::output_t out;
  T_CHECK(mpc.solve(in, out));
  T_CHECK(out.valid);

  checkContactConstraints(p, in, out);

  // The two feet that are down carry the robot between them.
  const double weight = p.mass * p.gravity;
  T_NEAR(resultantForce(out).z(), weight, 0.2 * weight);
  T_CHECK(out.force_world[0].z() > 0.0);
  T_CHECK(out.force_world[3].z() > 0.0);

  // And the bounds handed to the solver say so explicitly, for every step of
  // the horizon and not just the one whose forces come back.
  const Eigen::VectorXd& lb = mpc.getLowerBound();
  const Eigen::VectorXd& ub = mpc.getUpperBound();
  for (int k = 0; k < H; k++) {
    for (int leg : {1, 2}) {
      for (int c = 0; c < 3; c++) {
        const int idx = k * MdlConvexMPC::NUM_INPUTS + 3 * leg + c;
        T_NEAR(lb[idx], 0.0, 0.0);
        T_NEAR(ub[idx], 0.0, 0.0);
      }
    }
    const int fz = k * MdlConvexMPC::NUM_INPUTS + 2;  // FL normal component
    T_NEAR(lb[fz], p.force_min, 0.0);
    T_NEAR(ub[fz], p.force_max, 0.0);
  }

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  3. Reference correction direction
// ---------------------------------------------------------------------------

void test_correction_direction() {
  g_case = "correction_direction";
  std::cout << "test_correction_direction..." << std::endl;

  const MdlConvexMPC::params_t p = go2Params();
  const double weight = p.mass * p.gravity;

  MdlConvexMPC mpc;
  mpc.setParams(p);
  T_CHECK(mpc.reset());

  // Baseline: nothing to correct. Everything below is measured against it, so a
  // constant offset in the solution cannot pass for a correction.
  MdlConvexMPC::input_t in;
  makeStandingInput(p, in);
  MdlConvexMPC::output_t base;
  T_CHECK(mpc.solve(in, base));
  const Eigen::Vector3d fBase = resultantForce(base);
  const Eigen::Vector3d mBase = resultantMoment(in, base);

  // -- Too low: push harder than the weight to climb back.
  {
    MdlConvexMPC::input_t low;
    makeStandingInput(p, low);
    low.current.com_position_world.z() = kStandHeight - 0.05;
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(low, out));
    checkContactConstraints(p, low, out);
    T_CHECK(resultantForce(out).z() > fBase.z() + 0.1 * weight);
  }

  // -- Too high: back off.
  {
    MdlConvexMPC::input_t high;
    makeStandingInput(p, high);
    high.current.com_position_world.z() = kStandHeight + 0.05;
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(high, out));
    checkContactConstraints(p, high, out);
    T_CHECK(resultantForce(out).z() < fBase.z() - 0.1 * weight);
  }

  // -- Rolled left: the correcting moment opposes it.
  {
    MdlConvexMPC::input_t rolled;
    makeStandingInput(p, rolled);
    rolled.current.rpy.x() = 0.1;
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(rolled, out));
    checkContactConstraints(p, rolled, out);
    T_CHECK(resultantMoment(rolled, out).x() < mBase.x() - 0.5);
  }

  // -- Pitched: likewise about y.
  {
    MdlConvexMPC::input_t pitched;
    makeStandingInput(p, pitched);
    pitched.current.rpy.y() = 0.1;
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(pitched, out));
    checkContactConstraints(p, pitched, out);
    T_CHECK(resultantMoment(pitched, out).y() < mBase.y() - 0.5);
  }

  // -- Moving forward with a reference that says stop: decelerate.
  {
    MdlConvexMPC::input_t fast;
    makeStandingInput(p, fast);
    fast.current.com_velocity_world.x() = 0.3;
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(fast, out));
    checkContactConstraints(p, fast, out);
    T_CHECK(resultantForce(out).x() < fBase.x() - 1.0);
  }

  // -- And the mirror image, so the sign is not accidental.
  {
    MdlConvexMPC::input_t back;
    makeStandingInput(p, back);
    back.current.com_velocity_world.x() = -0.3;
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(back, out));
    checkContactConstraints(p, back, out);
    T_CHECK(resultantForce(out).x() > fBase.x() + 1.0);
  }

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  4. Horizon contact transition
// ---------------------------------------------------------------------------

void test_horizon_alignment() {
  g_case = "horizon_alignment";
  std::cout << "test_horizon_alignment..." << std::endl;

  const MdlConvexMPC::params_t p = go2Params();
  MdlConvexMPC mpc;
  mpc.setParams(p);
  T_CHECK(mpc.reset());

  MdlConvexMPC::input_t in;
  makeStandingInput(p, in);

  // Interval 0 is carried by FL/RR; every later interval by FR/RL. The returned
  // force is the one applied over interval 0, so it must follow interval 0's
  // mask. Reading the table one step late -- an easy thing to get wrong, since
  // reference[k] genuinely is offset by one from contact[k] -- puts the force
  // on the pair that has not landed yet, and this is the case that catches it.
  for (int k = 0; k < H; k++) {
    const bool first = (k == 0);
    in.contact[k][0] = first;
    in.contact[k][1] = !first;
    in.contact[k][2] = !first;
    in.contact[k][3] = first;
  }

  MdlConvexMPC::output_t out;
  T_CHECK(mpc.solve(in, out));
  T_CHECK(out.valid);
  checkContactConstraints(p, in, out);

  // Explicitly: the pair that lands later gets nothing now.
  T_NEAR(out.force_world[1].norm(), 0.0, 1e-9);
  T_NEAR(out.force_world[2].norm(), 0.0, 1e-9);
  // And the pair that is down now is actually being used.
  T_CHECK(out.force_world[0].z() + out.force_world[3].z() > 0.5 * p.mass * p.gravity);

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  5. Numerics, warm starting and rejection
// ---------------------------------------------------------------------------

void test_numerics() {
  g_case = "numerics";
  std::cout << "test_numerics..." << std::endl;

  const MdlConvexMPC::params_t p = go2Params();
  MdlConvexMPC mpc;

  // Parameters that do not describe a robot are refused, and refusing leaves
  // the module unable to solve rather than half configured.
  {
    MdlConvexMPC::params_t bad = p;
    bad.mass = 0.0;
    mpc.setParams(bad);
    T_CHECK(!mpc.reset());
    T_CHECK(!mpc.isReady());

    bad = p;
    bad.force_weight = 0.0;  // the only thing keeping the Hessian definite
    mpc.setParams(bad);
    T_CHECK(!mpc.reset());

    bad = p;
    bad.inertia_body(0, 0) = -1.0;  // not positive definite
    mpc.setParams(bad);
    T_CHECK(!mpc.reset());

    bad = p;
    bad.inertia_body(0, 1) = 5.0;  // not symmetric
    mpc.setParams(bad);
    T_CHECK(!mpc.reset());

    bad = p;
    bad.dt = std::nan("");
    mpc.setParams(bad);
    T_CHECK(!mpc.reset());

    bad = p;
    bad.force_max = bad.force_min;  // empty normal force range
    mpc.setParams(bad);
    T_CHECK(!mpc.reset());
  }

  // An unconfigured solver refuses to solve rather than producing something.
  {
    MdlConvexMPC::input_t in;
    makeStandingInput(p, in);
    MdlConvexMPC::output_t out;
    T_CHECK(!mpc.solve(in, out));
    T_CHECK(!out.valid);
  }

  mpc.setParams(p);
  T_CHECK(mpc.reset());
  T_CHECK(mpc.isReady());

  MdlConvexMPC::input_t in;
  makeStandingInput(p, in);

  MdlConvexMPC::output_t out;
  T_CHECK(mpc.solve(in, out));

  // The Hessian qpOASES was handed is finite and exactly symmetric. Exactly,
  // not nearly: it is symmetrized on purpose, because an asymmetry in the last
  // few digits shows up as a Cholesky factorization that fails on one solve out
  // of hundreds, which is a miserable thing to chase.
  const MdlConvexMPC::RowMatrix& Hm = mpc.getHessian();
  T_CHECK(Hm.allFinite());
  T_CHECK(Hm.rows() == H * MdlConvexMPC::NUM_INPUTS);
  T_CHECK((Hm - Hm.transpose()).cwiseAbs().maxCoeff() == 0.0);
  T_CHECK(mpc.getGradient().allFinite());
  // Positive definite, which the force penalty guarantees whatever the contact
  // schedule does to B_qp.
  T_CHECK(Hm.diagonal().minCoeff() > 0.0);

  // A run of warm started solves over a moving state, which is the only regime
  // the controller ever actually uses.
  for (int i = 0; i < 50; i++) {
    MdlConvexMPC::input_t moving;
    makeStandingInput(p, moving);
    const double s = 0.002 * i;
    moving.current.com_position_world += Eigen::Vector3d(s, -s, 0.3 * s);
    moving.current.com_velocity_world = Eigen::Vector3d(0.15, 0.0, 0.0);
    moving.current.rpy = Eigen::Vector3d(0.01 * std::sin(i), 0.01 * std::cos(i), 0.0);
    // Alternate the diagonals, so the active set really does change between
    // solves and the hotstart has something to do.
    for (int k = 0; k < H; k++) {
      const bool evens = ((i + k) % 2) == 0;
      moving.contact[k][0] = evens;
      moving.contact[k][1] = !evens;
      moving.contact[k][2] = !evens;
      moving.contact[k][3] = evens;
    }
    MdlConvexMPC::output_t o;
    T_CHECK(mpc.solve(moving, o));
    T_CHECK(o.valid);
    T_CHECK(mpc.getSolveTime() >= 0.0);
    T_CHECK(mpc.getSolveTime() < p.dt);
    checkContactConstraints(p, moving, o);
  }

  // Take a known good result, then feed the solver garbage and check that what
  // it hands back is still the good one, flagged as stale rather than fresh.
  MdlConvexMPC::input_t good;
  makeStandingInput(p, good);
  MdlConvexMPC::output_t last;
  T_CHECK(mpc.solve(good, last));
  T_CHECK(last.valid);

  const double badValues[] = {std::nan(""),
                              std::numeric_limits<double>::infinity()};
  for (double v : badValues) {
    // Non-finite in the current state...
    {
      MdlConvexMPC::input_t bad = good;
      bad.current.com_velocity_world.y() = v;
      MdlConvexMPC::output_t out2;
      T_CHECK(!mpc.solve(bad, out2));
      T_CHECK(!out2.valid);
      for (int leg = 0; leg < NL; leg++)
        T_CHECK((out2.force_world[leg] - last.force_world[leg]).cwiseAbs().maxCoeff() ==
                0.0);
    }
    // ...in a reference...
    {
      MdlConvexMPC::input_t bad = good;
      bad.reference[H - 1].rpy.z() = v;
      MdlConvexMPC::output_t out2;
      T_CHECK(!mpc.solve(bad, out2));
      T_CHECK(!out2.valid);
    }
    // ...and in a moment arm.
    {
      MdlConvexMPC::input_t bad = good;
      bad.moment_arm_world[3](1, 2) = v;
      MdlConvexMPC::output_t out2;
      T_CHECK(!mpc.solve(bad, out2));
      T_CHECK(!out2.valid);
    }
  }

  // The stored result survived all of that untouched, so a caller that asks
  // again after a bad cycle gets the last thing that was actually solved for.
  T_CHECK(mpc.getLastOutput().valid);
  for (int leg = 0; leg < NL; leg++)
    T_CHECK((mpc.getLastOutput().force_world[leg] - last.force_world[leg])
                .cwiseAbs()
                .maxCoeff() == 0.0);

  // And a good input after a bad one still works: a rejection must not leave
  // the solver wedged.
  MdlConvexMPC::output_t recovered;
  T_CHECK(mpc.solve(good, recovered));
  T_CHECK(recovered.valid);

  std::cout << "  PASS" << std::endl;
}

// ---------------------------------------------------------------------------
//  6. Discretization and condensation consistency
// ---------------------------------------------------------------------------

// The continuous single-rigid-body dynamics the solver claims to discretize,
// written out independently: rpy_dot = Rz(yawMean)^T * omega_W, p_dot = v,
// omega_dot = I_W^-1 * sum(r_i x f_i), v_dot = sum(f_i)/m + [0 0 g_state],
// g_state constant. Everything below integrates THIS with RK4 and demands the
// solver's exact-exponential prediction lands on it, so an index slip between a
// step's moment arms and its inputs, a wrong yaw frame, or a broken condensed
// stack has nowhere to hide.
static Eigen::Matrix<double, 13, 1> srbDeriv(const MdlConvexMPC::params_t& p,
                                             double yawMean,
                                             const Eigen::Matrix<double, 3, NL>& arms,
                                             const Eigen::Matrix<double, 13, 1>& x,
                                             const Eigen::VectorXd& u) {
  Eigen::Matrix<double, 13, 1> dx = Eigen::Matrix<double, 13, 1>::Zero();
  const Eigen::Matrix3d Rz =
      Eigen::AngleAxisd(yawMean, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d Iw = Rz * p.inertia_body * Rz.transpose();

  dx.segment<3>(0) = Rz.transpose() * x.segment<3>(6);
  dx.segment<3>(3) = x.segment<3>(9);

  Eigen::Vector3d tau = Eigen::Vector3d::Zero();
  Eigen::Vector3d f = Eigen::Vector3d::Zero();
  for (int leg = 0; leg < NL; leg++) {
    const Eigen::Vector3d fl = u.segment<3>(3 * leg);
    tau += Eigen::Vector3d(arms.col(leg)).cross(fl);
    f += fl;
  }
  dx.segment<3>(6) = Iw.llt().solve(tau);
  dx.segment<3>(9) = f / p.mass;
  dx[11] += x[12];
  return dx;
}

// One horizon interval under a zero-order hold, in enough RK4 substeps that the
// integrator's own error is far below the tolerance the test asserts.
static Eigen::Matrix<double, 13, 1> rk4Interval(const MdlConvexMPC::params_t& p,
                                                double yawMean,
                                                const Eigen::Matrix<double, 3, NL>& arms,
                                                Eigen::Matrix<double, 13, 1> x,
                                                const Eigen::VectorXd& u) {
  const int substeps = 32;
  const double h = p.dt / substeps;
  for (int s = 0; s < substeps; s++) {
    const auto k1 = srbDeriv(p, yawMean, arms, x, u);
    const auto k2 = srbDeriv(p, yawMean, arms, x + 0.5 * h * k1, u);
    const auto k3 = srbDeriv(p, yawMean, arms, x + 0.5 * h * k2, u);
    const auto k4 = srbDeriv(p, yawMean, arms, x + h * k3, u);
    x += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
  return x;
}

// Poses one solve whose horizon contains a diagonal swap and whose moment arms
// differ at every single step, then re-integrates the solver's own solution
// through the independent integrator above and checks the condensed prediction
// X = Aqp x0 + Bqp U step by step. This is the case the horizon experiments
// lean on: it is the only check that a plan spanning a mid-horizon contact
// change is dynamically consistent, which no closed-loop symptom can separate
// from a tuning problem.
static void runConsistencyCase(double dt, int horizon, int switchStep) {
  MdlConvexMPC::params_t p = go2Params();
  p.dt = dt;
  p.horizon = horizon;

  MdlConvexMPC mpc;
  mpc.setParams(p);
  T_CHECK(mpc.reset());

  const double yaw = 0.3;

  MdlConvexMPC::input_t in;
  // A body that is off its reference in every state the cost can see, so the
  // solution carries nonzero, step-varying forces worth re-integrating.
  in.current.rpy = Eigen::Vector3d(0.03, -0.02, yaw);
  in.current.com_position_world = Eigen::Vector3d(0.10, -0.05, kStandHeight - 0.015);
  in.current.angular_velocity_world = Eigen::Vector3d(0.2, -0.1, 0.15);
  in.current.com_velocity_world = Eigen::Vector3d(0.30, 0.05, -0.10);

  for (int k = 0; k < horizon; k++) {
    in.reference[k] = in.current;
    // Every reference yaw equal to the current one keeps yawMean at exactly
    // yaw, so the independent integrator and the solver agree on the frame by
    // construction rather than to within an averaging detail.
    in.reference[k].rpy = Eigen::Vector3d(0.0, 0.0, yaw);
    in.reference[k].com_position_world =
        Eigen::Vector3d(0.10 + 0.3 * dt * (k + 1), -0.05, kStandHeight);
    in.reference[k].angular_velocity_world.setZero();
    in.reference[k].com_velocity_world = Eigen::Vector3d(0.3, 0.0, 0.0);

    const bool firstPair = k < switchStep;
    in.contact[k][0] = firstPair;
    in.contact[k][1] = !firstPair;
    in.contact[k][2] = !firstPair;
    in.contact[k][3] = firstPair;

    for (int leg = 0; leg < NL; leg++) {
      // A distinct arm for every (step, leg), drifting the way a stance sweep
      // does, so using step j's arms for step k's dynamics cannot cancel out.
      in.moment_arm_world[k].col(leg) =
          nominalFoot(leg) - p.com_offset_body +
          Eigen::Vector3d(-0.03 * dt * k, 0.002 * k, 0.001 * (k % 3));
    }
  }

  MdlConvexMPC::output_t out;
  T_CHECK(mpc.solve(in, out));
  T_CHECK(out.valid);

  const Eigen::VectorXd& U = mpc.getSolution();

  // Swing pinning holds across the whole stacked solution, not just u_0.
  for (int k = 0; k < horizon; k++)
    for (int leg = 0; leg < NL; leg++)
      if (!in.contact[k][leg])
        T_NEAR(U.segment<3>(k * MdlConvexMPC::NUM_INPUTS + 3 * leg).norm(), 0.0, 1e-9);

  // The forces are genuinely exercising the dynamics.
  T_CHECK(U.head<3>().norm() + U.segment<3>(9).norm() > 10.0);

  Eigen::Matrix<double, 13, 1> x0;
  x0.segment<3>(0) = in.current.rpy;
  x0.segment<3>(3) = in.current.com_position_world;
  x0.segment<3>(6) = in.current.angular_velocity_world;
  x0.segment<3>(9) = in.current.com_velocity_world;
  x0[12] = -p.gravity;

  const int NS = MdlConvexMPC::NUM_STATES;
  const Eigen::VectorXd xpred = mpc.getAqp() * x0 + mpc.getBqp() * U;

  Eigen::Matrix<double, 13, 1> x = x0;
  for (int k = 0; k < horizon; k++) {
    x = rk4Interval(p, yaw, in.moment_arm_world[k],
                    x, U.segment(k * MdlConvexMPC::NUM_INPUTS,
                                 MdlConvexMPC::NUM_INPUTS));
    const double err = (xpred.segment(k * NS, NS) - x).cwiseAbs().maxCoeff();
    if (err > 1e-7) {
      std::cerr << "FAIL [" << g_case << "] step " << k << ": |X_qp - X_rk4| = "
                << err << std::endl;
      std::exit(1);
    }
  }
}

void test_discretization_consistency() {
  g_case = "discretization_consistency";
  std::cout << "test_discretization_consistency..." << std::endl;

  // The shipped grid with a swap a third of the way in, and the paper's coarse
  // grid with one in the middle. Between them: both dt regimes, and a horizon
  // long enough that a stacking error compounds visibly.
  runConsistencyCase(0.01, 12, 4);
  runConsistencyCase(0.04, 10, 5);

  std::cout << "  PASS" << std::endl;
}

// The horizon step and the solve period are separate knobs, and reset() is what
// resolves an unset solve period to dt. That resolution is what keeps a config
// written before the split behaving exactly as it did, so it is worth pinning:
// a silent zero here would make MdlTrot re-solve every single control cycle.
// The horizon sizes the QP, and qpOASES fixes its dimensions at construction, so
// changing it has to rebuild the solver rather than resize it. A stale object
// would show up as a wrong-sized solve rather than as a clean failure, which is
// worth one case of its own.
void test_horizon_is_runtime() {
  std::cout << "test_horizon_is_runtime..." << std::endl;

  MdlConvexMPC mpc;
  MdlConvexMPC::params_t p = go2Params();

  // Out of range is refused, not clamped: solving a shorter problem than the
  // caller asked for would be silent and wrong.
  for (int bad : {0, -1, MdlConvexMPC::MAX_HORIZON + 1}) {
    p.horizon = bad;
    mpc.setParams(p);
    T_CHECK(!mpc.reset());
  }

  // Grow and shrink on the same object, solving a standing problem each time.
  // The weight has to come out the same however many steps it is spread over.
  const double weight = go2Params().mass * go2Params().gravity;
  for (int h : {H, 25, MdlConvexMPC::MAX_HORIZON, 4, H}) {
    p.horizon = h;
    mpc.setParams(p);
    T_CHECK(mpc.reset());
    T_CHECK(mpc.getParams().horizon == h);

    MdlConvexMPC::input_t in;
    makeStandingInput(p, in);
    MdlConvexMPC::output_t out;
    T_CHECK(mpc.solve(in, out));
    T_CHECK(out.valid);
    checkContactConstraints(p, in, out);
    T_NEAR(resultantForce(out).z(), weight, 0.06 * weight);
  }

  std::cout << "  PASS" << std::endl;
}

void test_solve_period_defaults_to_dt() {
  std::cout << "test_solve_period_defaults_to_dt..." << std::endl;

  MdlConvexMPC mpc;
  MdlConvexMPC::params_t p = go2Params();

  // Unset, the documented "follow dt".
  p.solve_period = 0.0;
  mpc.setParams(p);
  T_CHECK(mpc.reset());
  T_NEAR(mpc.getParams().solve_period, p.dt, 1e-15);

  // Negative and non-finite are the same case, not a way to disable solving.
  for (double bad : {-1.0, -0.0, std::numeric_limits<double>::quiet_NaN()}) {
    p.solve_period = bad;
    mpc.setParams(p);
    T_CHECK(mpc.reset());
    T_NEAR(mpc.getParams().solve_period, p.dt, 1e-15);
  }

  // A real value survives, including one finer than the horizon step, which is
  // the whole point of the split.
  p.solve_period = 0.01;
  mpc.setParams(p);
  T_CHECK(mpc.reset());
  T_NEAR(mpc.getParams().solve_period, 0.01, 1e-15);
  T_NEAR(mpc.getParams().dt, 0.04, 1e-15);

  std::cout << "  PASS" << std::endl;
}

int main() {
  std::cout << "=== Convex MPC Tests ===" << std::endl;
  // Numerics runs first on purpose. It is the one case that does not depend on
  // the discretized dynamics, so with _discretizeDynamics() still stubbed the
  // output reads "test_numerics ... PASS" followed by the first dynamics case
  // failing, which is exactly the state the chunk is supposed to be handed in.
  // Run last it would never execute at all, since a failed check exits.
  test_numerics();
  test_static_support();
  test_diagonal_support();
  test_correction_direction();
  test_horizon_alignment();
  test_discretization_consistency();
  test_solve_period_defaults_to_dt();
  test_horizon_is_runtime();
  std::cout << "All convex MPC tests passed." << std::endl;
  return 0;
}
