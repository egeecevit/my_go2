/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef MDLCONVEXMPC_HH
#define MDLCONVEXMPC_HH

#include <Eigen/Dense>

#include "rtcore/Module.hh"

#define MPCMODULE_NAME "MdlConvexMPC"

namespace qpOASES {
class SQProblem;
}

namespace rtcore {
class LogServer;
}

/** \brief Convex model-predictive ground reaction force solver.

  The controller of Di Carlo et al., *Dynamic Locomotion in the MIT Cheetah 3
  through Convex Model-Predictive Control* (IROS 2018), reduced to the one thing
  a behavior actually needs from it: given where the body is, where it should be
  over the next few tenths of a second, and which feet will be on the ground
  when, produce the four world-frame ground reaction forces to apply right now.

  The model is the paper's 13-state single rigid body. The legs have no mass and
  the feet apply pure forces at known points, which is what makes the problem
  convex: the only decision variables are twelve force components per prediction
  step, and both the dynamics and the friction cone are linear in them. The
  state is

      x = ( roll pitch yaw  p_COM^W  omega^W  v_COM^W  g_z )

  with the thirteenth entry a constant carrying -g, which is the paper's trick
  for expressing the affine gravity term inside a linear model. Everything is in
  world coordinates; the caller does the single rotation into the leg frame at
  the motor command boundary.

  The rigid body state is at the whole-robot centre of mass, not the body frame
  origin. MdlPosVelEstimator estimates the latter, so MdlTrot converts before
  filling input_t and the moment arms are measured from the COM. That matters
  because inertia_body is defined about that same point; mixing the two puts a
  fixed lever arm error into every predicted moment.

  This is a service module in the same sense as MdlLegControl: the framework
  owns its lifecycle, a behavior grabs it, its scheduled update() does nothing,
  and the work happens in a synchronous solve() the owner calls at its own
  cadence. A mailbox and a scheduled solve would buy nothing here and would cost
  a one-cycle delay between the state the forces were computed for and the state
  they are applied in.

  Configuration lives in the [mpc] table; see mpc.toml. Every entry there
  changes the mathematical problem or describes the robot, so a missing or
  malformed one is an initialization error rather than something to default
  quietly: a wrong mass or inertia does not fail, it just produces confidently
  wrong forces.
 */
class MdlConvexMPC : public rtcore::Module {
 public:
  // Several fixed size members below have byte counts that are multiples of
  // sixteen, which Eigen wants aligned. C++17 makes new honour that on its own
  // and the macro expands to nothing, but it is left in so the requirement
  // survives a future change of standard. Same reasoning as
  // MdlPosVelEstimator.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int NUM_LEGS = 4;
  /** \brief Prediction steps. 10 at the default 0.04 s is a 0.4 s horizon, a
      little under one gait cycle, which is the paper's Section V-A setting. */
  static constexpr int HORIZON = 10;
  /** \brief States in the single-rigid-body model, gravity included. */
  static constexpr int NUM_STATES = 13;
  /** \brief Decision variables per prediction step: 3 force components x 4 feet. */
  static constexpr int NUM_INPUTS = 3 * NUM_LEGS;
  /** \brief Total decision variables in the condensed QP. */
  static constexpr int NUM_VARS = HORIZON * NUM_INPUTS;
  /** \brief Friction pyramid rows: |fx| <= mu*fz and |fy| <= mu*fz per foot. */
  static constexpr int NUM_CONSTRAINTS = HORIZON * NUM_LEGS * 4;

  /** \brief One body state in the layout above, minus the gravity entry, which
      is not something a caller ever chooses. */
  struct body_state_t {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_position_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d com_velocity_world = Eigen::Vector3d::Zero();
  };

  /** \brief Everything one solve needs.

    reference[k] is the desired state at now + (k+1)*dt, because the condensed
    state vector stacks x_1 ... x_N and never contains x_0. moment_arm_world[k]
    and contact[k] describe the interval that *starts* at now + k*dt, so they
    are offset from the references by one step by construction. Getting these
    two indexings confused is the classic way to end up with a controller that
    reacts one interval late and looks merely badly tuned. */
  struct input_t {
    body_state_t current;
    body_state_t reference[HORIZON];
    /** \brief COM-to-foot vectors in world coordinates, one column per leg. */
    Eigen::Matrix<double, 3, NUM_LEGS> moment_arm_world[HORIZON];
    bool contact[HORIZON][NUM_LEGS];
  };

  /** \brief The first control interval's forces, and how they were obtained. */
  struct output_t {
    /** \brief Ground reaction force on the robot, world frame. A standing robot
        therefore has positive z here. */
    Eigen::Vector3d force_world[NUM_LEGS] = {
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()};
    /** \brief True when these forces came from a successful solve of the
        problem just posed, false when they are a retained earlier result. */
    bool valid = false;
    /** \brief qpOASES returnValue of the last attempt. Zero is success. */
    int solver_status = 0;
  };

  /** \brief Tunables, settable without a ModuleManager so the solver can be
      exercised from a unit test. The defaults are deliberately unusable: mass
      and inertia have no sensible fallback, and reset() rejects them. */
  struct params_t {
    /** \brief Prediction and nominal solve interval [s]. */
    double dt = 0.04;
    /** \brief Aggregate robot mass [kg]. */
    double mass = 0.0;
    /** \brief Whole-robot COM relative to the body frame origin [m]. */
    Eigen::Vector3d com_offset_body = Eigen::Vector3d::Zero();
    /** \brief Aggregate inertia about that COM, body axes [kg m^2]. */
    Eigen::Matrix3d inertia_body = Eigen::Matrix3d::Zero();
    /** \brief Gravitational acceleration magnitude [m/s^2]. Positive. */
    double gravity = 9.81;
    /** \brief Coulomb friction coefficient for the square pyramid. */
    double friction = 0.6;
    /** \brief Normal force bounds on a stance foot [N]. */
    double force_min = 0.0;
    double force_max = 0.0;
    /** \brief alpha in the paper's cost. Must be positive: it is the only term
        making the condensed Hessian positive definite when a foot is airborne
        for the whole horizon and its columns of B_qp are therefore zero. */
    double force_weight = 1.0e-6;
    /** \brief Diagonal state cost, in the state order above minus gravity. */
    double state_weights[NUM_STATES - 1] = {1, 1, 1, 0, 0, 50, 0, 0, 1, 1, 1, 1};
    /** \brief qpOASES working set recalculation budget per solve. */
    int max_working_set = 250;
  };

  typedef Eigen::Matrix<double, NUM_STATES, NUM_STATES> StateMatrix;
  typedef Eigen::Matrix<double, NUM_STATES, NUM_INPUTS> InputMatrix;
  /** \brief qpOASES takes C arrays it reads row by row, so anything handed to
      it has to be stored row-major rather than in Eigen's column-major
      default. */
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMatrix;

  MdlConvexMPC();
  ~MdlConvexMPC();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  void setParams(const params_t& params);
  const params_t& getParams() const { return _params; }

  /** \brief Validates the parameters, sizes the solver matrices and clears all
      warm-start and result state. Must be called after setParams().

    Returns false, and leaves the module unable to solve, when the parameters do
    not describe a physically usable robot. */
  bool reset();

  /** \brief Solves one horizon and returns the first interval's forces.

    Rejects non-finite input outright. On any rejection or solver failure the
    last successful result is copied into output with valid set false, so a
    caller holding the previous force keeps holding it rather than receiving a
    partially written one, and the stored result is left intact.

    @return true only when qpOASES converged and the extracted forces are finite. */
  bool solve(const input_t& input, output_t& output);

  /** \brief The most recent successful result, or a zeroed invalid one. */
  const output_t& getLastOutput() const { return _last; }

  /** \brief Wall time the last solve() spent inside qpOASES [s]. */
  double getSolveTime() const { return _solveTime; }

  /** \brief True once reset() has accepted a set of parameters. */
  bool isReady() const { return _ready; }

  // Solver internals, exposed for unit testing.
  const RowMatrix& getHessian() const { return _H; }
  const Eigen::VectorXd& getGradient() const { return _g; }
  /** \brief Per-variable force bounds from the last solve, in the stacked order
      (step, leg, xyz). A swing foot's three entries are pinned to zero. */
  const Eigen::VectorXd& getLowerBound() const { return _lb; }
  const Eigen::VectorXd& getUpperBound() const { return _ub; }

 private:
  params_t _params;
  bool _ready = false;

  // Per-step discretized dynamics, rebuilt every solve.
  StateMatrix _Ad[HORIZON];
  InputMatrix _Bd[HORIZON];

  // Condensed dynamics X = _Aqp * x0 + _Bqp * U, sized in reset().
  Eigen::MatrixXd _Aqp;  // (HORIZON*NUM_STATES) x NUM_STATES
  Eigen::MatrixXd _Bqp;  // (HORIZON*NUM_STATES) x NUM_VARS

  // Cost and constraint data handed to qpOASES.
  RowMatrix _H;                // NUM_VARS x NUM_VARS
  Eigen::VectorXd _g;          // NUM_VARS
  RowMatrix _Acon;             // NUM_CONSTRAINTS x NUM_VARS, constant after reset()
  Eigen::VectorXd _lbA, _ubA;  // constant after reset()
  Eigen::VectorXd _lb, _ub;    // per-foot force bounds, rebuilt every solve
  Eigen::VectorXd _solution;

  // Stacked weights and references, kept as members so nothing allocates in the
  // control path once reset() has run.
  Eigen::VectorXd _L;     // diagonal of the state cost, HORIZON*NUM_STATES
  Eigen::VectorXd _xref;  // stacked reference states
  Eigen::VectorXd _x0;    // current state, NUM_STATES

  // Cost assembly scratch. Members purely so that _buildCost() allocates
  // nothing: it runs inside the 1 kHz behavior's cycle whenever the MPC is due.
  Eigen::MatrixXd _LB;  // L * _Bqp
  Eigen::VectorXd _e;   // _Aqp*x0 - _xref
  Eigen::VectorXd _We;  // L .* _e

  qpOASES::SQProblem* _qp = nullptr;
  bool _qpInitialized = false;

  output_t _last;
  double _solveTime = 0.0;

  rtcore::LogServer* _logserver = nullptr;
  double _logForces[NUM_INPUTS] = {0};
  double _logSolver[2] = {0};  // solver status, solve time

  void _readConfig();

  /** \brief Continuous single-rigid-body dynamics at one horizon step,
      discretized over dt with a zero-order hold on the forces.

    @param yaw Mean reference yaw, shared by every step as in paper Section IV-C
    @param moment_arms COM-to-foot vectors in world coordinates, one per leg */
  bool _discretizeDynamics(double yaw, const Eigen::Matrix<double, 3, NUM_LEGS>& moment_arms,
                           StateMatrix& Ad, InputMatrix& Bd) const;

  /** \brief Stacks _Ad/_Bd into the condensed _Aqp/_Bqp. */
  void _buildCondensedDynamics();

  /** \brief Builds _H and _g from the condensed dynamics and the references. */
  void _buildCost();

  /** \brief Fills _lb/_ub from the contact schedule. Swing feet are pinned to
      exactly zero rather than being removed from the problem: at this horizon
      length the saving is not worth a decision-variable count that changes
      every time a foot lifts. */
  void _buildBounds(const input_t& input);

  /** \brief Packs a body_state_t and the gravity entry into the 13-vector. */
  void _packState(const body_state_t& state, Eigen::Ref<Eigen::VectorXd> x) const;

  static bool _stateFinite(const body_state_t& state);
};

#endif  // MDLCONVEXMPC_HH
