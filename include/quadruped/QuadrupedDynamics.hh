/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#ifndef QUADRUPEDDYNAMICS_HH
#define QUADRUPEDDYNAMICS_HH

#include <Eigen/Dense>

#include "quadruped/QuadrupedLegDynamics.hh"

/** Floating-base rigid-body model of the whole quadruped (thesis 6.1, 6.2).

  QuadrupedLegDynamics models one leg on a fixed base; a whole-body controller
  needs the trunk and the coupling between the legs and the unactuated base.
  This class assembles both from the same link chain, so the two models cannot
  drift apart.

  Generalized velocity v (NV = 18):
    v[0:3]   base-origin linear velocity, world axes
    v[3:6]   base angular velocity, BODY axes
    v[6:18]  joint rates, controller convention, leg-major FL FR RL RR
  This is the layout of MuJoCo's free joint (qvel), chosen so the model can be
  checked against mj_fullM / qfrc_bias / mj_jac entry by entry (after the
  per-joint polarity).

  Equation of motion (thesis 6.1), with f_i the ground reaction ON the robot
  in world axes:
      M vdot + h = S^T tau + sum_i contact[i].J^T f_i,    h = C(v) v + g
  S selects the twelve joints (the last twelve rows).

  Every Cartesian output is in world axes. Positions are relative to the base
  origin: the model never needs the base position. */
class QuadrupedDynamics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int NUM_LEGS = QuadrupedLegDynamics::NUM_LEGS;
  static constexpr int NV = 6 + 3 * NUM_LEGS;

  typedef Eigen::Matrix<double, NV, NV> MatrixV;
  typedef Eigen::Matrix<double, NV, 1> VectorV;
  typedef Eigen::Matrix<double, 3, NV> Jacobian;

  struct params_t {
    /** The twelve leg links, joint axes, armature and torque limits. */
    QuadrupedLegDynamics::params_t legs;
    /** Trunk mass [kg], COM in body axes [m], inertia about that COM in body
        axes [kg m^2]. */
    double trunk_mass = 0.0;
    Eigen::Vector3d trunk_com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d trunk_inertia_com = Eigen::Matrix3d::Zero();
    /** Foot sphere radius [m]. The ground contact is under the sphere centre,
        not at it, and a planted foot rolls instead of pivoting. */
    double foot_radius = 0.0;
    /** Gravitational acceleration magnitude [m/s^2]. */
    double gravity = 9.81;
  };

  struct state_t {
    /** Body -> world rotation. */
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d v_base = Eigen::Vector3d::Zero();  // world axes
    Eigen::Vector3d w_body = Eigen::Vector3d::Zero();  // body axes
    Eigen::Vector3d q[NUM_LEGS], qd[NUM_LEGS];         // controller convention

    /** v in the layout documented on the class. */
    VectorV velocity() const {
      VectorV v;
      v.segment<3>(0) = v_base;
      v.segment<3>(3) = w_body;
      for (int leg = 0; leg < NUM_LEGS; leg++) v.segment<3>(6 + 3 * leg) = qd[leg];
      return v;
    }
  };

  /** A point (or constraint) moving with the robot: velocity = J v,
      acceleration = J vdot + Jdv. */
  struct point_t {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();  // world axes, from the base origin
    Jacobian J = Jacobian::Zero();
    Eigen::Vector3d Jdv = Eigen::Vector3d::Zero();
  };

  struct terms_t {
    MatrixV M = MatrixV::Zero();
    VectorV h = VectorV::Zero();
    /** Foot sphere centre. */
    point_t foot[NUM_LEGS];
    /** Rolling-without-slip constraint of the foot sphere on level ground.
        p is the ground contact point, centre - r e_z. J v is the velocity of
        the sphere's material point there, v_centre + r e_z x w_calf, which is
        zero for a planted foot; so is J vdot + Jdv. A force at the contact
        point enters the equation of motion through this J. */
    point_t contact[NUM_LEGS];
    /** Abduction joint anchor, fixed in the trunk. */
    point_t hip[NUM_LEGS];
    /** Whole-robot COM from the base origin, world axes, and total mass. */
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    double mass = 0.0;
  };

  explicit QuadrupedDynamics(const params_t& params);

  const params_t& getParams() const { return _params; }
  bool isValid() const { return _legs.isValid(); }

  /** False on non-finite input or a kinematics failure; out is then not
      meaningful. */
  bool compute(const state_t& state, terms_t& out) const;

 private:
  params_t _params;
  QuadrupedLegDynamics _legs;
};

#endif  // QUADRUPEDDYNAMICS_HH
