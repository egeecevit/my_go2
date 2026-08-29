/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#ifndef QUADRUPEDLEGDYNAMICS_HH
#define QUADRUPEDLEGDYNAMICS_HH

#include <array>
#include <string>

#include <Eigen/Dense>

/** Fixed-base inverse dynamics for one three-DoF quadruped leg.

  The controller's swing law only needs the dynamics between the body and one
  airborne foot. Keeping that model here, independent of MuJoCo or a hardware
  transport, makes the same feedforward available in simulation and on the
  robot. All public joint coordinates use the polarity-normalized convention
  used by QuadrupedKinematics and MotorHW. */
class QuadrupedLegDynamics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr int NUM_LEGS = 4;
  static constexpr int NUM_JOINTS = 3;

  struct link_t {
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    Eigen::Matrix3d inertia_com = Eigen::Matrix3d::Zero();
  };

  struct joint_t {
    /** Translation from the parent link frame to this joint, parent frame. */
    Eigen::Vector3d parent_translation = Eigen::Vector3d::Zero();
    /** Static rotation from this joint/link zero frame to its parent. */
    Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();
    /** Revolute axis in the joint/link zero frame. */
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    link_t link;
  };

  struct leg_t {
    std::array<joint_t, NUM_JOINTS> joint;
    /** Foot point relative to the final link frame. */
    Eigen::Vector3d foot = Eigen::Vector3d::Zero();
    /** Physical joint angle = direction .* controller joint angle. */
    Eigen::Vector3d joint_direction = Eigen::Vector3d::Ones();
  };

  struct params_t {
    std::string robot_name;
    std::array<leg_t, NUM_LEGS> leg;
    /** Reflected rotor inertia on each controller joint [kg m^2]. */
    Eigen::Vector3d armature = Eigen::Vector3d::Zero();
    /** Absolute motor torque limits [Nm], ordered abduction, hip, knee. */
    Eigen::Vector3d torque_limit = Eigen::Vector3d::Zero();
    double derivative_step = 1e-6;
    double min_operational_eigenvalue = 1e-7;
    double max_operational_condition = 1e8;

    bool validate() const;
  };

  struct terms_t {
    Eigen::Matrix3d mass_matrix = Eigen::Matrix3d::Zero();
    /** C(q,qdot)qdot + G(q), in controller coordinates [Nm]. */
    Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    Eigen::Matrix3d foot_jacobian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d foot_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d jdot_qdot = Eigen::Vector3d::Zero();
    Eigen::Matrix3d operational_inertia = Eigen::Matrix3d::Zero();
  };

  explicit QuadrupedLegDynamics(const params_t& params);

  const params_t& getParams() const { return _params; }
  bool isValid() const { return _valid; }

  bool footPosition(int leg, const Eigen::Vector3d& q, Eigen::Vector3d& position) const;
  bool footJacobian(int leg, const Eigen::Vector3d& q, Eigen::Matrix3d& jacobian) const;

  /** Compute all terms needed by paper equations (2) and (3).

    gravity_body is the inertial acceleration of gravity expressed in body
    axes, for example R_BW^T [0,0,-9.81]. */
  bool compute(int leg, const Eigen::Vector3d& q, const Eigen::Vector3d& qdot,
               const Eigen::Vector3d& gravity_body, terms_t& terms) const;

 private:
  struct kinematics_t {
    Eigen::Vector3d joint_position[NUM_JOINTS];
    Eigen::Vector3d joint_axis[NUM_JOINTS];
    Eigen::Matrix3d link_rotation[NUM_JOINTS];
    Eigen::Vector3d com_position[NUM_JOINTS];
    Eigen::Matrix3d com_linear_jacobian[NUM_JOINTS];
    Eigen::Matrix3d com_angular_jacobian[NUM_JOINTS];
    Eigen::Vector3d foot_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d foot_jacobian = Eigen::Matrix3d::Zero();
  };

  params_t _params;
  bool _valid = false;

  bool _kinematics(int leg, const Eigen::Vector3d& q, kinematics_t& out) const;
  bool _massMatrix(int leg, const Eigen::Vector3d& q, Eigen::Matrix3d& mass) const;
};

#endif  // QUADRUPEDLEGDYNAMICS_HH
