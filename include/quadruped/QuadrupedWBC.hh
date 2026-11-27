/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#ifndef QUADRUPEDWBC_HH
#define QUADRUPEDWBC_HH

#include <Eigen/Dense>

#include "quadruped/QuadrupedDynamics.hh"

/** Whole-body controller: prioritized task hierarchy followed by the WBIC
    quadratic program (thesis Sec. 6.2.1, Eq. 6.3-6.9; Kim et al. [16]).

  The caller supplies the dynamics terms, which feet are in rigid contact, the
  operational-space tasks in priority order and the nominal contact forces.
  solve() then

    1. builds the contact-consistent seed and null space (6.5) and inserts the
       tasks recursively with dynamically consistent inverses (6.4), (6.6),
       giving the hierarchical acceleration vdot_h; the analogous kinematic
       recursion gives joint position and velocity references;
    2. solves (6.8) for the floating-base acceleration correction and the
       contact-force correction, subject to the six unactuated rows of (6.1),
       the friction pyramid and the normal-force bounds;
    3. returns the joint torque of (6.9).

  Sign and layout conventions are QuadrupedDynamics'. Not a Module: it holds no
  robot state between calls beyond the QP solver's own storage. */
class QuadrupedWBC {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int NUM_LEGS = QuadrupedDynamics::NUM_LEGS;
  static constexpr int NV = QuadrupedDynamics::NV;
  /** Body orientation, body position and one task per swing foot (thesis
      Sec. 6.2.1 has two; a flight phase swings all four). */
  static constexpr int MAX_TASKS = 6;

  /** One three-dimensional operational-space task x = f(q). */
  struct task_t {
    QuadrupedDynamics::Jacobian J = QuadrupedDynamics::Jacobian::Zero();
    /** Task acceleration = J vdot + Jdv. */
    Eigen::Vector3d Jdv = Eigen::Vector3d::Zero();
    /** Feedback-stabilized acceleration command of (6.3), feedforward included. */
    Eigen::Vector3d acc_cmd = Eigen::Vector3d::Zero();
    /** Task-space position error, desired minus current, and desired task
        velocity. Used only by the kinematic recursion behind q_des, qd_des. */
    Eigen::Vector3d pos_err = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_des = Eigen::Vector3d::Zero();
  };

  struct params_t {
    /** W_v of (6.8): weight on the floating-base acceleration correction
        (Table 6.2, "Floating-base WBIC weight"). */
    double w_base = 1000.0;
    /** W_f of (6.8): weight on the contact-force correction. */
    double w_force = 1.0;
    /** Coulomb coefficient of the square friction pyramid |fx|,|fy| <= mu fz. */
    double friction = 0.6;
    /** Normal-force bounds on a contact foot [N]. */
    double fz_min = 0.0;
    double fz_max = 200.0;
  };

  struct input_t {
    /** M, h and the contact Jacobians; must outlive the call. */
    const QuadrupedDynamics::terms_t* dyn = nullptr;
    /** Measured joint angles, the linearization point of q_des. */
    Eigen::Vector3d q[NUM_LEGS];
    /** Feet held by the rigid contact constraint (6.2). */
    bool contact[NUM_LEGS] = {false, false, false, false};
    /** Nominal ground reaction F_c^d on the robot, world axes [N]. Read only
        for contact feet. */
    Eigen::Vector3d f_des[NUM_LEGS];
    /** Tasks in decreasing priority; the first num_tasks entries are used. */
    int num_tasks = 0;
    task_t task[MAX_TASKS];
  };

  struct output_t {
    /** Joint torque of (6.9), controller convention, leg-major. */
    Eigen::Matrix<double, 3 * NUM_LEGS, 1> tau;
    /** Joint references from the kinematic recursion. */
    Eigen::Vector3d q_des[NUM_LEGS], qd_des[NUM_LEGS];
    /** Generalized acceleration after the QP correction (6.7). */
    QuadrupedDynamics::VectorV vdot;
    /** Solved ground reaction, world axes; zero for a foot not in contact. */
    Eigen::Vector3d f[NUM_LEGS];
    /** qpOASES returnValue of the solve; zero is success. */
    int qp_status = 0;
  };

  QuadrupedWBC();
  ~QuadrupedWBC();

  void setParams(const params_t& params) { _params = params; }
  const params_t& getParams() const { return _params; }

  /** Hierarchical acceleration vdot_h of (6.5)-(6.6) and the joint references,
      without the QP. Exposed because the hierarchy's guarantees (contact
      consistency, strict priority) are properties of this step alone. */
  static bool hierarchy(const input_t& in, QuadrupedDynamics::VectorV& vdot_h,
                        Eigen::Vector3d q_des[NUM_LEGS], Eigen::Vector3d qd_des[NUM_LEGS]);

  /** False, with out not meaningful, on non-finite input, an M that is not
      positive definite or a failed QP. A task direction that a higher priority
      has taken is dropped, not reported: giving way is what the priority order
      is for. With no contact foot (flight) the QP has only the floating-base
      correction: the base falls freely whatever the body tasks ask, and the
      joint accelerations stay the hierarchy's. */
  bool solve(const input_t& in, output_t& out);

 private:
  params_t _params;
};

#endif  // QUADRUPEDWBC_HH
