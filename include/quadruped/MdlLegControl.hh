/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef LEGMODULE_HH
#define LEGMODULE_HH

#include "rtcore/Module.hh"
#include "hardware/MotorHW.hh"
#include "Eigen/Dense"

#define LEGMODULE_NAME "MdlLegControl"

class QuadrupedKinematics;
class QuadrupedLegDynamics;

class MdlLegControl : public rtcore::Module {
public:
  struct command_result_t {
    Eigen::Vector3d kp_cartesian = Eigen::Vector3d::Zero();
    Eigen::Vector3d kd_cartesian = Eigen::Vector3d::Zero();
    Eigen::Vector3d feedback_torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d feedforward_torque = Eigen::Vector3d::Zero();
    /** Net torque including configured joint damping at the measured qdot. */
    Eigen::Vector3d requested_torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d applied_torque = Eigen::Vector3d::Zero();
    /** Value placed in MotorHW::cmd_t::tau; local kd supplies the remainder. */
    Eigen::Vector3d motor_feedforward_torque = Eigen::Vector3d::Zero();
    double torque_scale = 1.0;
  };

  MdlLegControl(int ind);
  ~MdlLegControl();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  // Full MIT-mode joint-space command. Returns false if joint limits violated.
  bool setJointCommand(const Eigen::Vector3d &a, const Eigen::Vector3d &adot,
                       const Eigen::Vector3d &kp, const Eigen::Vector3d &kd,
                       const Eigen::Vector3d &tau_ff);

  // Foot-space target with joint-space gains. IK and damped Jacobian handled internally.
  bool setFootCommand(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot,
                      const Eigen::Vector3d &kp, const Eigen::Vector3d &kd,
                      const Eigen::Vector3d &tau_ff);

  /** \brief Cartesian impedance about the *measured* foot state, emitted as
      joint torque. Equation (1) of the MIT Cheetah 3 convex MPC paper.

    The complement of setFootCommand(), which tracks a foot position through IK
    and joint-space PD. Here nothing is inverted: the foot state comes from
    forward kinematics on the measured joint angles, the Cartesian error is
    turned into a force, and J^T maps that force to torque. That is what lets
    one call serve both gait phases -- a swing leg gets Cartesian stiffness and
    no feedforward, a stance leg gets zero stiffness and the ground reaction
    force the MPC asked for -- and it is the only way to command a stance force
    at all, since a position controlled stance leg turns tracking error into leg
    deflection rather than body motion.

    Units differ from setFootCommand(): kp and kd here are N/m and N/(m/s), not
    Nm/rad and Nm/(rad/s).

    @param force_feedforward_body Force the *actuators apply at the foot*, body
           frame. For stance that is the negated, rotated ground reaction force;
           the caller has already flipped the sign.
    @param joint_damping Joint-space kd applied alongside the torque. Must be
           positive: MdlSimDriver substitutes 5.0 for any non-positive kd and
           does so by mutating the stored command, so a zero sent once sticks.
    @return false, with nothing emitted, if the motors are not ready or the
            resulting torque is not finite. */
  bool setCartesianForceCommand(const Eigen::Vector3d &position_ref_body,
                                const Eigen::Vector3d &velocity_ref_body,
                                const Eigen::Vector3d &kp_cartesian,
                                const Eigen::Vector3d &kd_cartesian,
                                const Eigen::Vector3d &force_feedforward_body,
                                double joint_damping,
                                command_result_t *result = nullptr);

  /** Paper equations (1)--(3), including Go2 inverse-dynamics feedforward. */
  bool computeSwingCommand(const Eigen::Vector3d& q, const Eigen::Vector3d& qdot,
                           const Eigen::Vector3d& position_ref_body,
                           const Eigen::Vector3d& velocity_ref_body,
                           const Eigen::Vector3d& acceleration_ref_body,
                           const Eigen::Vector3d& gravity_body,
                           const Eigen::Vector3d& natural_frequency,
                           const Eigen::Vector3d& damping_ratio,
                           double feedforward_scale, double joint_damping,
                           command_result_t& result) const;

  bool setSwingCommand(const Eigen::Vector3d& position_ref_body,
                       const Eigen::Vector3d& velocity_ref_body,
                       const Eigen::Vector3d& acceleration_ref_body,
                       const Eigen::Vector3d& gravity_body,
                       const Eigen::Vector3d& natural_frequency,
                       const Eigen::Vector3d& damping_ratio,
                       double feedforward_scale, double joint_damping,
                       command_result_t* result = nullptr);

  bool hasSwingDynamics() const;

  /** \brief The equation (1) computation on its own, with no hardware.

    Split out of setCartesianForceCommand() so the control law can be unit
    tested: this repository has no mocking infrastructure and a MotorHW cannot
    be faked, so the only testable form is one that takes the measured state as
    an argument. Uses nothing but the leg's own kinematics.

    @param q Measured joint angles [rad]
    @param qdot Measured joint velocities [rad/s]
    @param tau_ff Output feedforward joint torque [Nm]
    @return false if the kinematics fail or any input or result is not finite,
            in which case tau_ff is not meaningful. */
  bool computeCartesianForceCommand(const Eigen::Vector3d &q, const Eigen::Vector3d &qdot,
                                    const Eigen::Vector3d &position_ref_body,
                                    const Eigen::Vector3d &velocity_ref_body,
                                    const Eigen::Vector3d &kp_cartesian,
                                    const Eigen::Vector3d &kd_cartesian,
                                    const Eigen::Vector3d &force_feedforward_body,
                                    Eigen::Vector3d &tau_ff) const;

  // FK from current joint state. False if motors not STATUS_READY.
  bool getFootPosition(Eigen::Vector3d &pos) const;
  bool getFootState(Eigen::Vector3d& pos, Eigen::Vector3d& vel) const;

  // Legacy wrappers using default gains. Now return bool.
  bool setTargetAngles(const Eigen::Vector3d &a, const Eigen::Vector3d &adot);
  bool setTargetPosition(const Eigen::Vector3d &p, const Eigen::Vector3d &pdot);

private:
  MotorHW *_motorhw = nullptr;
  MotorHW::cmd_t _cmd[3];

  int _indices[3];
  QuadrupedKinematics *_kinematics = nullptr;
  QuadrupedLegDynamics *_dynamics = nullptr;

  Eigen::Vector3d _default_kp = Eigen::Vector3d(100.0, 100.0, 100.0);
  Eigen::Vector3d _default_kd = Eigen::Vector3d(5.0, 5.0, 5.0);
  // Targets without a populated dynamics model retain the legacy unbounded
  // command path. Go2 init replaces this with the real motor limits.
  Eigen::Vector3d _torqueLimit = Eigen::Vector3d::Constant(1e12);
  double _jacobian_damping = 0.01;

  void _emitCommands();
  bool _readJointState(Eigen::Vector3d& q, Eigen::Vector3d& qdot) const;
  bool _limitTorque(const Eigen::Vector3d& qdot, const Eigen::Vector3d& task_torque,
                    double joint_damping, command_result_t& result) const;
  bool _emitTorqueCommand(const Eigen::Vector3d& q, double joint_damping,
                          const command_result_t& result);
};

#endif
