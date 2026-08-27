/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "quadruped/MdlConvexMPC.hh"

#include <qpOASES.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>

// Used by the zero-order-hold discretization in _discretizeDynamics(). Left in
// deliberately: the augmented matrix exponential is the one place this module
// needs anything beyond Eigen/Dense, and hunting for the header is not part of
// the exercise.
#include <unsupported/Eigen/MatrixFunctions>

#include "rtcore/ConfigTable.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ModuleManager.hh"

using namespace rtcore;

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...)  // printf(__VA_ARGS__)

MdlConvexMPC::MdlConvexMPC() : Module(MPCMODULE_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlConvexMPC::MdlConvexMPC\n");
}

MdlConvexMPC::~MdlConvexMPC() {
  DBGPRINT("MdlConvexMPC::~MdlConvexMPC\n");
  delete _qp;
  _qp = nullptr;
}

namespace {
  Eigen::Matrix3d toSkewSymmetric(const Eigen::Vector3d& v) {
      Eigen::Matrix3d m;
      m <<    0, -v.z(),  v.y(),
          v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
      return m;
  }
}

// ---------------------------------------------------------------------------
//  Module interface
// ---------------------------------------------------------------------------

void MdlConvexMPC::init() {
  DBGPRINT("MdlConvexMPC::init\n");

  _readConfig();

  // Everything in [mpc] either describes the robot or changes the optimization
  // problem, so there is no defensible fallback. A wrong mass or inertia does
  // not fail loudly at run time; it produces confidently wrong forces, which is
  // far harder to notice than a refusal to start.
  if (!reset())
    _mgr->fatalError(MPCMODULE_NAME,
                     "[mpc] does not describe a usable robot; see mpc.toml");

  _logserver = (LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
  if (_logserver) {
    _logserver->registerVar(LOG_DOUBLE, NUM_INPUTS, MPCMODULE_NAME, "forces",
                            (unsigned char*)_logForces);
    _logserver->registerVar(LOG_DOUBLE, 2, MPCMODULE_NAME, "solver",
                            (unsigned char*)_logSolver);
  }
}

void MdlConvexMPC::uninit() {
  DBGPRINT("MdlConvexMPC::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(MPCMODULE_NAME, "forces");
    _logserver->deleteVar(MPCMODULE_NAME, "solver");
    _logserver = nullptr;
  }

  delete _qp;
  _qp = nullptr;
  _ready = false;
}

void MdlConvexMPC::activate() {
  DBGPRINT("MdlConvexMPC::activate\n");

  // A warm start is only worth having if the previous active set came from a
  // problem resembling this one. Across an activation boundary the robot has
  // been doing something else entirely, so the stored one is discarded and the
  // next solve pays for a cold init().
  _qpInitialized = false;
  _last = output_t();
  _solveTime = 0.0;
  for (int i = 0; i < NUM_INPUTS; i++) _logForces[i] = 0.0;
  _logSolver[0] = _logSolver[1] = 0.0;
}

void MdlConvexMPC::deactivate() { DBGPRINT("MdlConvexMPC::deactivate\n"); }

void MdlConvexMPC::update() {
  // Deliberately empty. This is a service module: the owner calls solve()
  // synchronously at its own cadence, so a scheduled update would only be able
  // to hand back forces computed for the previous cycle's state.
}

// ---------------------------------------------------------------------------
//  Configuration
// ---------------------------------------------------------------------------

void MdlConvexMPC::_readConfig() {
  ConfigTable config;
  if (!_mgr->getConfigTable("mpc", config))
    _mgr->fatalError(MPCMODULE_NAME, "No [mpc] config table; see mpc.toml");

  params_t p;
  p.dt = config.getDouble("dt", -1.0);
  p.mass = config.getDouble("mass", -1.0);
  p.gravity = config.getDouble("gravity", p.gravity);
  p.friction = config.getDouble("friction", -1.0);
  p.force_min = config.getDouble("force_min", -1.0);
  p.force_max = config.getDouble("force_max", -1.0);
  p.force_weight = config.getDouble("force_weight", -1.0);
  p.max_working_set = (int)config.getInt("max_working_set", p.max_working_set);

  ConfigArray com;
  if (config.getArray("com_offset_body", com) && com.size() == 3) {
    for (int i = 0; i < 3; i++) p.com_offset_body[i] = com.getDoubleAt(i, 0.0);
  } else {
    _mgr->fatalError(MPCMODULE_NAME, "[mpc] com_offset_body must have 3 entries");
  }

  ConfigArray inertia;
  if (config.getArray("inertia_body", inertia) && inertia.size() == 9) {
    // Row major, matching how it is written in the file.
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) p.inertia_body(r, c) = inertia.getDoubleAt(3 * r + c, 0.0);
  } else {
    _mgr->fatalError(MPCMODULE_NAME, "[mpc] inertia_body must have 9 entries (row major)");
  }

  ConfigArray weights;
  if (config.getArray("state_weights", weights) && weights.size() == NUM_STATES - 1) {
    for (int i = 0; i < NUM_STATES - 1; i++)
      p.state_weights[i] = weights.getDoubleAt(i, -1.0);
  } else {
    _mgr->fatalError(MPCMODULE_NAME, "[mpc] state_weights must have %d entries",
                     NUM_STATES - 1);
  }

  setParams(p);
}

void MdlConvexMPC::setParams(const params_t& params) {
  _params = params;
  // Nothing is usable until reset() has vetted these and sized the solver.
  _ready = false;
}

// ---------------------------------------------------------------------------
//  Sizing and validation
// ---------------------------------------------------------------------------

bool MdlConvexMPC::reset() {
  _ready = false;

  // Validation, in the order a mistake is likely to be made. Written so that
  // NaN fails every test rather than sliding through a > comparison.
  if (!(_params.dt > 0.0) || !std::isfinite(_params.dt)) return false;
  if (!(_params.mass > 0.0) || !std::isfinite(_params.mass)) return false;
  if (!_params.com_offset_body.allFinite()) return false;
  if (!_params.inertia_body.allFinite()) return false;
  if (!(_params.gravity > 0.0) || !std::isfinite(_params.gravity)) return false;
  if (!(_params.friction > 0.0) || !std::isfinite(_params.friction)) return false;
  if (!(_params.force_min >= 0.0) || !std::isfinite(_params.force_min)) return false;
  if (!(_params.force_max > _params.force_min) || !std::isfinite(_params.force_max))
    return false;
  // Strictly positive: alpha is the only thing making the condensed Hessian
  // positive definite once a foot is airborne for the whole horizon, which
  // zeroes its columns of B_qp.
  if (!(_params.force_weight > 0.0) || !std::isfinite(_params.force_weight)) return false;
  if (_params.max_working_set < 1) return false;
  for (int i = 0; i < NUM_STATES - 1; i++)
    if (!(_params.state_weights[i] >= 0.0) || !std::isfinite(_params.state_weights[i]))
      return false;

  // An inertia that is not symmetric positive definite is not an inertia, and
  // chunk A inverts it. Catching it here rather than at the first solve means
  // the message names the config file.
  const Eigen::Matrix3d& I = _params.inertia_body;
  if ((I - I.transpose()).cwiseAbs().maxCoeff() > 1e-9 * I.cwiseAbs().maxCoeff())
    return false;
  Eigen::LLT<Eigen::Matrix3d> llt(I);
  if (llt.info() != Eigen::Success) return false;

  const int NS = NUM_STATES;
  const int NX = HORIZON * NS;

  _Aqp.setZero(NX, NS);
  _Bqp.setZero(NX, NUM_VARS);
  _H.setZero(NUM_VARS, NUM_VARS);
  _g.setZero(NUM_VARS);
  _Acon.setZero(NUM_CONSTRAINTS, NUM_VARS);
  _lbA.setZero(NUM_CONSTRAINTS);
  _ubA.setZero(NUM_CONSTRAINTS);
  _lb.setZero(NUM_VARS);
  _ub.setZero(NUM_VARS);
  _solution.setZero(NUM_VARS);
  _L.setZero(NX);
  _xref.setZero(NX);
  _x0.setZero(NS);
  _LB.setZero(NX, NUM_VARS);
  _e.setZero(NX);
  _We.setZero(NX);

  // State cost, repeated over the horizon. The gravity state gets weight zero:
  // it is a constant carried along to make the model affine, not something to
  // track.
  for (int k = 0; k < HORIZON; k++) {
    for (int i = 0; i < NS - 1; i++) _L[k * NS + i] = _params.state_weights[i];
    _L[k * NS + NS - 1] = 0.0;
  }

  // Square friction pyramid, constant for the whole run. Four rows per foot per
  // step:  fx -+ mu*fz  and  fy -+ mu*fz, each one-sided, which together give
  // |fx| <= mu*fz and |fy| <= mu*fz whenever fz >= 0.
  //
  // The normal force itself is a variable bound rather than a row here, so that
  // pinning a swing foot to zero is a bound change and the constraint matrix
  // never has to be rebuilt.
  const double mu = _params.friction;
  const double INF = qpOASES::INFTY;
  int row = 0;
  for (int k = 0; k < HORIZON; k++) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      const int col = k * NUM_INPUTS + 3 * leg;

      _Acon(row, col + 0) = 1.0;
      _Acon(row, col + 2) = -mu;
      _lbA[row] = -INF;
      _ubA[row] = 0.0;
      row++;

      _Acon(row, col + 0) = 1.0;
      _Acon(row, col + 2) = mu;
      _lbA[row] = 0.0;
      _ubA[row] = INF;
      row++;

      _Acon(row, col + 1) = 1.0;
      _Acon(row, col + 2) = -mu;
      _lbA[row] = -INF;
      _ubA[row] = 0.0;
      row++;

      _Acon(row, col + 1) = 1.0;
      _Acon(row, col + 2) = mu;
      _lbA[row] = 0.0;
      _ubA[row] = INF;
      row++;
    }
  }

  if (!_qp) {
    _qp = new qpOASES::SQProblem(NUM_VARS, NUM_CONSTRAINTS);
    qpOASES::Options options;
    // The solver's own preset for exactly this use: a sequence of closely
    // related QPs solved to a modest tolerance under a hard time budget.
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    _qp->setOptions(options);
  }
  _qpInitialized = false;

  _last = output_t();
  _solveTime = 0.0;
  _ready = true;
  return true;
}

// ---------------------------------------------------------------------------
//  Dynamics
// ---------------------------------------------------------------------------

bool MdlConvexMPC::_discretizeDynamics(double yaw,
                                       const Eigen::Matrix<double, 3, NUM_LEGS>& moment_arms,
                                       StateMatrix& Ad, InputMatrix& Bd) const {
  // TODO(ege): implement
  //
  // - Fill only the nonzero blocks of the continuous A_c (13x13): the
  //   Theta_dot = Rz(yaw)^T * omega_W block, the p_dot = v identity block, and
  //   the gravity column feeding v_dot.z from the 13th state. Everything else
  //   is zero.
  // - B_c (13x12), per leg i: I_W^-1 * [r_i]_x into the omega rows and
  //   (1/m) * I3 into the v rows, with I_W = Rz(yaw) * I_B * Rz(yaw)^T and r_i
  //   the corresponding column of moment_arms.
  // - Discretize with the augmented matrix exponential
  //   exp(dt * [A_c B_c; 0 0]) (Eigen's unsupported MatrixFunctions, .exp());
  //   do not substitute forward Euler.
  // - Gotchas: A_c is built once at the mean reference yaw but B_c is fresh for
  //   every horizon step, and the gravity state carries the value -g, so the
  //   column signs have to come out as v_dot.z = ... + g_z.
  StateMatrix Ac = StateMatrix::Zero();
  Eigen::AngleAxisd rotation_vector(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d rot_mat = rotation_vector.toRotationMatrix();

  Ac.block<3, 3>(0,6) = rot_mat.transpose();
  Ac.block<3, 3>(3,9) = Eigen::Matrix3d::Identity();
  Ac(11,12) = 1.0;

  InputMatrix Bc = InputMatrix::Zero();
  const Eigen::Matrix3d Iw = rot_mat * _params.inertia_body * rot_mat.transpose();
  Eigen::LLT<Eigen::Matrix3d> inertia_solver(Iw);
  if (inertia_solver.info() != Eigen::Success)
    return false;

  for (int i = 0; i < NUM_LEGS; i++) {
    Bc.block<3, 3>(6, i*3) = inertia_solver.solve(toSkewSymmetric(moment_arms.col(i)));
    Bc.block<3, 3>(9, i*3) = Eigen::Matrix3d::Identity() / _params.mass;
  }

  static constexpr int AUGMENTED_SIZE = NUM_STATES + NUM_INPUTS;
  using AugmentedMatrix = Eigen::Matrix<double, AUGMENTED_SIZE, AUGMENTED_SIZE>;

  AugmentedMatrix augmented = AugmentedMatrix::Zero();

  augmented.block<NUM_STATES, NUM_STATES>(0, 0) = Ac;
  augmented.block<NUM_STATES, NUM_INPUTS>(0, NUM_STATES) = Bc;

  augmented *= _params.dt;
  const AugmentedMatrix discrete = augmented.exp();

  Ad = discrete.block<NUM_STATES, NUM_STATES>(0, 0);
  Bd = discrete.block<NUM_STATES, NUM_INPUTS>(0, NUM_STATES);

  return Ad.allFinite() && Bd.allFinite();
}

void MdlConvexMPC::_buildCondensedDynamics() {
  const int NS = NUM_STATES;

  // X = [x_1 ... x_N] with x_{k+1} = Ad[k] x_k + Bd[k] u_k, so block row i of
  // A_qp is Ad[i]...Ad[0] and block (i,j) of B_qp is Ad[i]...Ad[j+1] Bd[j].
  // Both follow from the previous row by one multiplication.
  _Aqp.block(0, 0, NS, NS) = _Ad[0];
  for (int i = 1; i < HORIZON; i++)
    _Aqp.block(i * NS, 0, NS, NS).noalias() =
        _Ad[i] * _Aqp.block((i - 1) * NS, 0, NS, NS);

  _Bqp.setZero();
  for (int i = 0; i < HORIZON; i++) {
    for (int j = 0; j < i; j++)
      _Bqp.block(i * NS, j * NUM_INPUTS, NS, NUM_INPUTS).noalias() =
          _Ad[i] * _Bqp.block((i - 1) * NS, j * NUM_INPUTS, NS, NUM_INPUTS);
    _Bqp.block(i * NS, i * NUM_INPUTS, NS, NUM_INPUTS) = _Bd[i];
  }
}

void MdlConvexMPC::_buildCost() {
  // H = 2 (B_qp^T L B_qp + alpha I),  g = 2 B_qp^T L (A_qp x_0 - X_ref).
  _LB.noalias() = _L.asDiagonal() * _Bqp;
  _H.noalias() = _Bqp.transpose() * _LB;
  _H *= 2.0;
  _H.diagonal().array() += 2.0 * _params.force_weight;

  // Symmetrize. The product above is symmetric in exact arithmetic but not bit
  // for bit, and qpOASES reads the array as given: a Hessian that is asymmetric
  // in the last few digits shows up as a Cholesky factorization that fails on
  // one solve out of many, which is a miserable thing to debug. Done in place
  // rather than through 0.5*(H + H^T) so nothing allocates here.
  for (int i = 0; i < NUM_VARS; i++) {
    for (int j = 0; j < i; j++) {
      const double s = 0.5 * (_H(i, j) + _H(j, i));
      _H(i, j) = s;
      _H(j, i) = s;
    }
  }

  _e.noalias() = _Aqp * _x0;
  _e -= _xref;
  _We = _L.cwiseProduct(_e);
  _g.noalias() = _Bqp.transpose() * _We;
  _g *= 2.0;
}

void MdlConvexMPC::_buildBounds(const input_t& input) {
  // Horizontal components are bounded by the friction rows against the actual
  // normal force; the box here only has to be wide enough not to cut into that,
  // which mu*force_max is by construction.
  const double flim = _params.friction * _params.force_max;

  for (int k = 0; k < HORIZON; k++) {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      const int col = k * NUM_INPUTS + 3 * leg;
      if (input.contact[k][leg]) {
        _lb[col + 0] = -flim;
        _ub[col + 0] = flim;
        _lb[col + 1] = -flim;
        _ub[col + 1] = flim;
        _lb[col + 2] = _params.force_min;
        _ub[col + 2] = _params.force_max;
      } else {
        // Exactly zero on both sides. An airborne foot pushes on nothing, and
        // pinning the variable is cheaper than removing it and cheaper still
        // than trusting the cost to drive it to zero.
        for (int c = 0; c < 3; c++) {
          _lb[col + c] = 0.0;
          _ub[col + c] = 0.0;
        }
      }
    }
  }
}

void MdlConvexMPC::_packState(const body_state_t& state,
                              Eigen::Ref<Eigen::VectorXd> x) const {
  x.segment<3>(0) = state.rpy;
  x.segment<3>(3) = state.com_position_world;
  x.segment<3>(6) = state.angular_velocity_world;
  x.segment<3>(9) = state.com_velocity_world;
  // The affine gravity term, carried as a state so the model stays linear.
  // Negative because the acceleration it produces is downwards.
  x[12] = -_params.gravity;
}

bool MdlConvexMPC::_stateFinite(const body_state_t& state) {
  return state.rpy.allFinite() && state.com_position_world.allFinite() &&
         state.angular_velocity_world.allFinite() && state.com_velocity_world.allFinite();
}

// ---------------------------------------------------------------------------
//  Solve
// ---------------------------------------------------------------------------

bool MdlConvexMPC::solve(const input_t& input, output_t& output) {
  // Whatever happens below, the caller gets the last force that was actually
  // solved for, with valid saying whether it belongs to the problem just posed.
  // Holding the previous force through a hiccup is the owner's decision to make
  // and it needs a whole vector to make it with, not a half written one.
  output = _last;
  output.valid = false;

  if (!_ready) return false;

  if (!_stateFinite(input.current)) return false;
  for (int k = 0; k < HORIZON; k++) {
    if (!_stateFinite(input.reference[k])) return false;
    if (!input.moment_arm_world[k].allFinite()) return false;
  }

  // One A_c for the whole horizon, evaluated at the mean reference yaw, as in
  // paper Section IV-C. B_c is rebuilt per step because the feet move.
  double yawMean = 0.0;
  for (int k = 0; k < HORIZON; k++) yawMean += input.reference[k].rpy.z();
  yawMean /= HORIZON;

  for (int k = 0; k < HORIZON; k++) {
    if (!_discretizeDynamics(yawMean, input.moment_arm_world[k], _Ad[k], _Bd[k]))
      return false;
    if (!_Ad[k].allFinite() || !_Bd[k].allFinite()) return false;
  }

  _buildCondensedDynamics();

  _packState(input.current, _x0);
  for (int k = 0; k < HORIZON; k++)
    _packState(input.reference[k], _xref.segment(k * NUM_STATES, NUM_STATES));

  _buildCost();
  _buildBounds(input);

  if (!_H.allFinite() || !_g.allFinite()) return false;

  const auto t0 = std::chrono::steady_clock::now();

  qpOASES::int_t nWSR = _params.max_working_set;
  qpOASES::returnValue ret;
  if (_qpInitialized) {
    ret = _qp->hotstart(_H.data(), _g.data(), _Acon.data(), _lb.data(), _ub.data(),
                        _lbA.data(), _ubA.data(), nWSR, nullptr);
  } else {
    ret = _qp->init(_H.data(), _g.data(), _Acon.data(), _lb.data(), _ub.data(),
                    _lbA.data(), _ubA.data(), nWSR, nullptr);
  }

  _solveTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  output.solver_status = (int)ret;
  _logSolver[0] = (double)ret;
  _logSolver[1] = _solveTime;

  if (qpOASES::getSimpleStatus(ret) != 0) {
    // A failed hotstart leaves the active set describing a problem that was
    // never solved, so the next attempt starts cold rather than continuing from
    // it.
    _qpInitialized = false;
    return false;
  }

  if (_qp->getPrimalSolution(_solution.data()) != qpOASES::SUCCESSFUL_RETURN) {
    _qpInitialized = false;
    return false;
  }
  _qpInitialized = true;

  // Only u_0 is used; the rest of the horizon exists to make u_0 right.
  if (!_solution.head(NUM_INPUTS).allFinite()) return false;

  for (int leg = 0; leg < NUM_LEGS; leg++)
    output.force_world[leg] = _solution.segment<3>(3 * leg);
  output.valid = true;

  _last = output;
  for (int i = 0; i < NUM_INPUTS; i++) _logForces[i] = _solution[i];
  return true;
}

