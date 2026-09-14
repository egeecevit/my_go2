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

// Matrix exponential used for zero-order-hold discretization.
#include <unsupported/Eigen/MatrixFunctions>

#include "rtcore/ConfigTable.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ModuleManager.hh"

using namespace rtcore;

// Uncomment printf in DBGPRINT to enable debug messages.
#define DBGPRINT(...)  // printf(__VA_ARGS__)

MdlConvexMPC::MdlConvexMPC() : Module(MPCMODULE_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlConvexMPC::MdlConvexMPC\n");
}

MdlConvexMPC::~MdlConvexMPC() {
  DBGPRINT("MdlConvexMPC::~MdlConvexMPC\n");
  delete _qp;
  _qp = nullptr;
  if (_dumpFile) {
    std::fclose(_dumpFile);
    _dumpFile = nullptr;
  }
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

  // Validate the complete configuration before constructing the solver.
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

  if (_dumpFile) {
    std::fclose(_dumpFile);
    _dumpFile = nullptr;
  }
}

void MdlConvexMPC::activate() {
  DBGPRINT("MdlConvexMPC::activate\n");

  // A new activation starts qpOASES without a previous active set.
  _qpInitialized = false;
  _last = output_t();
  _solveTime = 0.0;
  for (int i = 0; i < NUM_INPUTS; i++) _logForces[i] = 0.0;
  _logSolver[0] = _logSolver[1] = 0.0;
}

void MdlConvexMPC::deactivate() { DBGPRINT("MdlConvexMPC::deactivate\n"); }

void MdlConvexMPC::update() {
  // solve() is called synchronously by the owning controller.
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
  // Zero follows dt; reset() resolves it after loading all parameters.
  p.solve_period = config.getDouble("solve_period", 0.0);
  p.horizon = (int)config.getInt("horizon", p.horizon);
  p.mass = config.getDouble("mass", -1.0);
  p.gravity = config.getDouble("gravity", p.gravity);
  p.friction = config.getDouble("friction", -1.0);
  p.force_min = config.getDouble("force_min", -1.0);
  p.force_max = config.getDouble("force_max", -1.0);
  p.force_weight = config.getDouble("force_weight", -1.0);
  p.terminal_weight_scale =
      config.getDouble("terminal_weight_scale", p.terminal_weight_scale);
  p.max_working_set = (int)config.getInt("max_working_set", p.max_working_set);
  p.dump_path = config.getString("dump_path", "");

  ConfigArray com;
  if (config.getArray("com_offset_body", com) && com.size() == 3) {
    for (int i = 0; i < 3; i++) p.com_offset_body[i] = com.getDoubleAt(i, 0.0);
  } else {
    _mgr->fatalError(MPCMODULE_NAME, "[mpc] com_offset_body must have 3 entries");
  }

  ConfigArray inertia;
  if (config.getArray("inertia_body", inertia) && inertia.size() == 9) {
    // Config arrays use row-major order.
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
  // Parameters take effect after reset() validates and sizes the solver.
  _ready = false;
}

// ---------------------------------------------------------------------------
//  Sizing and validation
// ---------------------------------------------------------------------------

bool MdlConvexMPC::reset() {
  _ready = false;

  // Reject invalid values before allocating solver matrices.
  if (!(_params.dt > 0.0) || !std::isfinite(_params.dt)) return false;
  // Reject oversized horizons before indexing fixed-size arrays.
  if (_params.horizon < 1 || _params.horizon > MAX_HORIZON) return false;
  // Resolve a nonpositive period to the model timestep.
  if (!(_params.solve_period > 0.0) || !std::isfinite(_params.solve_period))
    _params.solve_period = _params.dt;
  // Warn when the command hold period exceeds the model interval.
  if (_params.solve_period > _params.dt && _mgr)
    _mgr->warning(MPCMODULE_NAME,
                  "solve_period %.4f > dt %.4f: u_0 will be held past the "
                  "interval the model planned it for",
                  _params.solve_period, _params.dt);
  if (!(_params.mass > 0.0) || !std::isfinite(_params.mass)) return false;
  if (!_params.com_offset_body.allFinite()) return false;
  if (!_params.inertia_body.allFinite()) return false;
  if (!(_params.gravity > 0.0) || !std::isfinite(_params.gravity)) return false;
  if (!(_params.friction > 0.0) || !std::isfinite(_params.friction)) return false;
  if (!(_params.force_min >= 0.0) || !std::isfinite(_params.force_min)) return false;
  if (!(_params.force_max > _params.force_min) || !std::isfinite(_params.force_max))
    return false;
  // Positive force regularization keeps the condensed Hessian positive definite.
  if (!(_params.force_weight > 0.0) || !std::isfinite(_params.force_weight)) return false;
  if (!(_params.terminal_weight_scale > 0.0) ||
      !std::isfinite(_params.terminal_weight_scale))
    return false;
  if (_params.max_working_set < 1) return false;
  for (int i = 0; i < NUM_STATES - 1; i++)
    if (!(_params.state_weights[i] >= 0.0) || !std::isfinite(_params.state_weights[i]))
      return false;

  // Require a finite symmetric positive-definite inertia matrix.
  const Eigen::Matrix3d& I = _params.inertia_body;
  if ((I - I.transpose()).cwiseAbs().maxCoeff() > 1e-9 * I.cwiseAbs().maxCoeff())
    return false;
  Eigen::LLT<Eigen::Matrix3d> llt(I);
  if (llt.info() != Eigen::Success) return false;

  const int NS = NUM_STATES;
  const int H = _params.horizon;
  const int NX = H * NS;
  const int nv = H * NUM_INPUTS;

  _Aqp.setZero(NX, NS);
  _Bqp.setZero(NX, nv);
  _activeBqp.resize(NX, 0);
  _H.resize(0, 0);
  _g.resize(0);
  _Acon.resize(0, 0);
  _lbA.resize(0);
  _ubA.resize(0);
  _lb.setZero(nv);
  _ub.setZero(nv);
  _activeLb.resize(0);
  _activeUb.resize(0);
  _activeSolution.resize(0);
  _solution.setZero(nv);
  _activeVars = 0;
  _L.setZero(NX);
  _xref.setZero(NX);
  _x0.setZero(NS);
  _LB.resize(NX, 0);
  _e.setZero(NX);
  _We.setZero(NX);
  _Xpred.setZero(NX);

  // Reopen dumps with a fresh record size after horizon changes.
  if (_dumpFile) {
    std::fclose(_dumpFile);
    _dumpFile = nullptr;
  }

  // Repeat state weights across the horizon and scale the terminal step.
  // The affine gravity state has zero tracking weight.
  for (int k = 0; k < H; k++) {
    const double s = (k == H - 1) ? _params.terminal_weight_scale : 1.0;
    for (int i = 0; i < NS - 1; i++) _L[k * NS + i] = s * _params.state_weights[i];
    _L[k * NS + NS - 1] = 0.0;
  }

  // Build qpOASES lazily because contact schedules set the active dimensions.
  // Reset invalidates the previous active set.
  delete _qp;
  _qp = nullptr;
  _qpVars = 0;
  _qpCons = 0;
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
  // Build yaw-linearized dynamics with gravity as the final affine state, then
  // discretize them with a zero-order-hold matrix exponential.
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
  const int H = _params.horizon;

  // Propagate each state and input block through the horizon recursion.
  _Aqp.block(0, 0, NS, NS) = _Ad[0];
  for (int i = 1; i < H; i++)
    _Aqp.block(i * NS, 0, NS, NS).noalias() =
        _Ad[i] * _Aqp.block((i - 1) * NS, 0, NS, NS);

  _Bqp.setZero();
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < i; j++)
      _Bqp.block(i * NS, j * NUM_INPUTS, NS, NUM_INPUTS).noalias() =
          _Ad[i] * _Bqp.block((i - 1) * NS, j * NUM_INPUTS, NS, NUM_INPUTS);
    _Bqp.block(i * NS, i * NUM_INPUTS, NS, NUM_INPUTS) = _Bd[i];
  }
}

void MdlConvexMPC::_buildCost() {
  // Form the condensed quadratic cost over scheduled contact-force columns.
  _LB.noalias() = _L.asDiagonal() * _activeBqp;
  _H.noalias() = _activeBqp.transpose() * _LB;
  _H *= 2.0;
  _H.diagonal().array() += 2.0 * _params.force_weight;

  // Enforce a symmetric Hessian before passing it to qpOASES.
  const int nv = (int)_H.rows();
  for (int i = 0; i < nv; i++) {
    for (int j = 0; j < i; j++) {
      const double s = 0.5 * (_H(i, j) + _H(j, i));
      _H(i, j) = s;
      _H(j, i) = s;
    }
  }

  _e.noalias() = _Aqp * _x0;
  _e -= _xref;
  _We = _L.cwiseProduct(_e);
  _g.noalias() = _activeBqp.transpose() * _We;
  _g *= 2.0;
}

void MdlConvexMPC::_ensureSolverDimensions(int variables, int constraints) {
  if (_qp && _qpVars == variables && _qpCons == constraints) return;

  delete _qp;
  _qp = new qpOASES::SQProblem(variables, constraints);
  _qpVars = variables;
  _qpCons = constraints;
  _qpInitialized = false;

  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  _qp->setOptions(options);
}

bool MdlConvexMPC::_buildActiveProblem(const input_t& input) {
  const int H = _params.horizon;
  const int NX = H * NUM_STATES;

  _activeVars = 0;
  for (int k = 0; k < H; ++k) {
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
      if (!input.contact[k][leg]) continue;
      const int full = k * NUM_INPUTS + 3 * leg;
      for (int axis = 0; axis < 3; ++axis)
        _activeFullIndex[_activeVars++] = full + axis;
    }
  }

  // Reject a horizon with no active contact forces.
  if (_activeVars == 0) return false;

  const int activeFeet = _activeVars / 3;
  const int constraints = 4 * activeFeet;
  _activeBqp.resize(NX, _activeVars);
  for (int col = 0; col < _activeVars; ++col)
    _activeBqp.col(col) = _Bqp.col(_activeFullIndex[col]);

  _H.resize(_activeVars, _activeVars);
  _g.resize(_activeVars);
  _LB.resize(NX, _activeVars);
  _activeLb.resize(_activeVars);
  _activeUb.resize(_activeVars);
  _activeSolution.resize(_activeVars);
  _Acon.setZero(constraints, _activeVars);
  _lbA.resize(constraints);
  _ubA.resize(constraints);

  const double mu = _params.friction;
  const double flim = mu * _params.force_max;
  const double INF = qpOASES::INFTY;
  for (int foot = 0; foot < activeFeet; ++foot) {
    const int col = 3 * foot;
    const int row = 4 * foot;

    _activeLb[col + 0] = -flim;
    _activeUb[col + 0] = flim;
    _activeLb[col + 1] = -flim;
    _activeUb[col + 1] = flim;
    _activeLb[col + 2] = _params.force_min;
    _activeUb[col + 2] = _params.force_max;

    _Acon(row + 0, col + 0) = 1.0;
    _Acon(row + 0, col + 2) = -mu;
    _lbA[row + 0] = -INF;
    _ubA[row + 0] = 0.0;

    _Acon(row + 1, col + 0) = 1.0;
    _Acon(row + 1, col + 2) = mu;
    _lbA[row + 1] = 0.0;
    _ubA[row + 1] = INF;

    _Acon(row + 2, col + 1) = 1.0;
    _Acon(row + 2, col + 2) = -mu;
    _lbA[row + 2] = -INF;
    _ubA[row + 2] = 0.0;

    _Acon(row + 3, col + 1) = 1.0;
    _Acon(row + 3, col + 2) = mu;
    _lbA[row + 3] = 0.0;
    _ubA[row + 3] = INF;
  }

  _ensureSolverDimensions(_activeVars, constraints);
  return true;
}

void MdlConvexMPC::_buildBounds(const input_t& input) {
  // Use the maximum normal force for the horizontal friction bounds.
  const double flim = _params.friction * _params.force_max;

  for (int k = 0; k < _params.horizon; k++) {
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
        // Pin unscheduled foot forces to zero.
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
  // State layout: rpy, position, angular velocity, linear velocity, negative gravity.
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
  // Start from the last complete output and mark this solve invalid.
  output = _last;
  output.valid = false;

  if (!_ready) return false;

  const int H = _params.horizon;

  if (!_stateFinite(input.current)) return false;
  for (int k = 0; k < H; k++) {
    if (!_stateFinite(input.reference[k])) return false;
    if (!input.moment_arm_world[k].allFinite()) return false;
  }

  // Use the mean reference yaw for A_c and rebuild B_c for each step.
  double yawMean = 0.0;
  for (int k = 0; k < H; k++) yawMean += input.reference[k].rpy.z();
  yawMean /= H;

  for (int k = 0; k < H; k++) {
    if (!_discretizeDynamics(yawMean, input.moment_arm_world[k], _Ad[k], _Bd[k]))
      return false;
    if (!_Ad[k].allFinite() || !_Bd[k].allFinite()) return false;
  }

  _buildCondensedDynamics();

  _packState(input.current, _x0);
  for (int k = 0; k < H; k++)
    _packState(input.reference[k], _xref.segment(k * NUM_STATES, NUM_STATES));

  _buildBounds(input);
  if (!_buildActiveProblem(input)) return false;
  _buildCost();

  if (!_H.allFinite() || !_g.allFinite() || !_Acon.allFinite() ||
      !_activeLb.allFinite() || !_activeUb.allFinite())
    return false;

  const auto t0 = std::chrono::steady_clock::now();

  qpOASES::int_t nWSR = _params.max_working_set;
  qpOASES::returnValue ret;
  if (_qpInitialized) {
    ret = _qp->hotstart(_H.data(), _g.data(), _Acon.data(), _activeLb.data(),
                        _activeUb.data(), _lbA.data(), _ubA.data(), nWSR, nullptr);
  } else {
    ret = _qp->init(_H.data(), _g.data(), _Acon.data(), _activeLb.data(),
                    _activeUb.data(), _lbA.data(), _ubA.data(), nWSR, nullptr);
  }

  _solveTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  output.solver_status = (int)ret;
  _logSolver[0] = (double)ret;
  _logSolver[1] = _solveTime;

  if (qpOASES::getSimpleStatus(ret) != 0) {
    // Discard the active set after a failed solve.
    _qpInitialized = false;
    return false;
  }

  if (_qp->getPrimalSolution(_activeSolution.data()) !=
      qpOASES::SUCCESSFUL_RETURN) {
    _qpInitialized = false;
    return false;
  }
  _qpInitialized = true;

  _solution.setZero();
  for (int col = 0; col < _activeVars; ++col)
    _solution[_activeFullIndex[col]] = _activeSolution[col];

  // Apply only the first force block; the horizon determines its value.
  if (!_solution.head(NUM_INPUTS).allFinite()) return false;

  for (int leg = 0; leg < NUM_LEGS; leg++)
    output.force_world[leg] = _solution.segment<3>(3 * leg);
  output.valid = true;

  _last = output;
  for (int i = 0; i < NUM_INPUTS; i++) _logForces[i] = _solution[i];

  if (!_params.dump_path.empty()) _dumpSolve(input);
  return true;
}

// ---------------------------------------------------------------------------
//  Prediction dump
// ---------------------------------------------------------------------------

void MdlConvexMPC::_dumpSolve(const input_t& input) {
  const int H = _params.horizon;

  if (!_dumpFile) {
    _dumpFile = std::fopen(_params.dump_path.c_str(), "wb");
    if (!_dumpFile) {
      if (_mgr)
        _mgr->warning(MPCMODULE_NAME, "Cannot open dump file %s; disabling dump",
                      _params.dump_path.c_str());
      _params.dump_path.clear();
      return;
    }
    const double header[8] = {20260828.0,          1.0,
                              (double)H,           _params.dt,
                              (double)NUM_STATES,  (double)NUM_INPUTS,
                              (double)NUM_LEGS,    _params.solve_period};
    std::fwrite(header, sizeof(double), 8, _dumpFile);
  }

  // Reconstruct the predicted state sequence from the stored error and input response.
  _Xpred = _e + _xref;
  _Xpred.noalias() += _Bqp * _solution;

  const double t = _mgr ? _mgr->readTime() : 0.0;
  std::fwrite(&t, sizeof(double), 1, _dumpFile);
  std::fwrite(_x0.data(), sizeof(double), NUM_STATES, _dumpFile);
  std::fwrite(_xref.data(), sizeof(double), H * NUM_STATES, _dumpFile);
  std::fwrite(_solution.data(), sizeof(double), H * NUM_INPUTS, _dumpFile);
  std::fwrite(_Xpred.data(), sizeof(double), H * NUM_STATES, _dumpFile);

  double mask[MAX_HORIZON * NUM_LEGS];
  for (int k = 0; k < H; k++)
    for (int leg = 0; leg < NUM_LEGS; leg++)
      mask[k * NUM_LEGS + leg] = input.contact[k][leg] ? 1.0 : 0.0;
  std::fwrite(mask, sizeof(double), H * NUM_LEGS, _dumpFile);

  // Flush each record so diagnostic data is available immediately.
  std::fflush(_dumpFile);
}
