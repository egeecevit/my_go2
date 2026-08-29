/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#include "quadruped/QuadrupedLegDynamics.hh"

#include <cmath>

namespace {

bool finiteRotation(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite()) return false;
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-8 &&
         std::fabs(rotation.determinant() - 1.0) < 1e-8;
}

}  // namespace

bool QuadrupedLegDynamics::params_t::validate() const {
  if (!armature.allFinite() || (armature.array() < 0.0).any()) return false;
  if (!torque_limit.allFinite() || (torque_limit.array() <= 0.0).any()) return false;
  if (!(derivative_step > 0.0) || !std::isfinite(derivative_step)) return false;
  if (!(min_operational_eigenvalue > 0.0) ||
      !std::isfinite(min_operational_eigenvalue))
    return false;
  if (!(max_operational_condition > 1.0) || !std::isfinite(max_operational_condition))
    return false;

  for (const leg_t& leg_params : leg) {
    if (!leg_params.foot.allFinite() || !leg_params.joint_direction.allFinite()) return false;
    for (int j = 0; j < NUM_JOINTS; ++j) {
      if (std::fabs(std::fabs(leg_params.joint_direction[j]) - 1.0) > 1e-12) return false;
      const joint_t& joint = leg_params.joint[j];
      if (!joint.parent_translation.allFinite() || !finiteRotation(joint.parent_rotation) ||
          !joint.axis.allFinite() || std::fabs(joint.axis.norm() - 1.0) > 1e-10)
        return false;
      if (!(joint.link.mass > 0.0) || !std::isfinite(joint.link.mass) ||
          !joint.link.com.allFinite() || !joint.link.inertia_com.allFinite())
        return false;
      if ((joint.link.inertia_com - joint.link.inertia_com.transpose()).norm() > 1e-10)
        return false;
      Eigen::LLT<Eigen::Matrix3d> inertia_llt(joint.link.inertia_com);
      if (inertia_llt.info() != Eigen::Success) return false;
    }
  }
  return true;
}

QuadrupedLegDynamics::QuadrupedLegDynamics(const params_t& params)
    : _params(params), _valid(params.validate()) {}

bool QuadrupedLegDynamics::_kinematics(int leg, const Eigen::Vector3d& q,
                                       kinematics_t& out) const {
  if (!_valid || leg < 0 || leg >= NUM_LEGS || !q.allFinite()) return false;

  const leg_t& lp = _params.leg[leg];
  Eigen::Vector3d parent_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d parent_rotation = Eigen::Matrix3d::Identity();

  for (int i = 0; i < NUM_JOINTS; ++i) {
    const joint_t& jp = lp.joint[i];
    out.joint_position[i] = parent_position + parent_rotation * jp.parent_translation;
    const Eigen::Matrix3d zero_rotation = parent_rotation * jp.parent_rotation;
    const Eigen::Vector3d physical_axis = zero_rotation * jp.axis;
    const double direction = lp.joint_direction[i];
    out.joint_axis[i] = direction * physical_axis;
    out.link_rotation[i] =
        zero_rotation * Eigen::AngleAxisd(direction * q[i], jp.axis).toRotationMatrix();
    out.com_position[i] = out.joint_position[i] + out.link_rotation[i] * jp.link.com;

    out.com_linear_jacobian[i].setZero();
    out.com_angular_jacobian[i].setZero();
    for (int j = 0; j <= i; ++j) {
      out.com_angular_jacobian[i].col(j) = out.joint_axis[j];
      out.com_linear_jacobian[i].col(j) =
          out.joint_axis[j].cross(out.com_position[i] - out.joint_position[j]);
    }

    parent_position = out.joint_position[i];
    parent_rotation = out.link_rotation[i];
  }

  out.foot_position = parent_position + parent_rotation * lp.foot;
  out.foot_jacobian.setZero();
  for (int j = 0; j < NUM_JOINTS; ++j)
    out.foot_jacobian.col(j) =
        out.joint_axis[j].cross(out.foot_position - out.joint_position[j]);

  return out.foot_position.allFinite() && out.foot_jacobian.allFinite();
}

bool QuadrupedLegDynamics::_massMatrix(int leg, const Eigen::Vector3d& q,
                                       Eigen::Matrix3d& mass) const {
  kinematics_t kin;
  if (!_kinematics(leg, q, kin)) return false;

  mass = _params.armature.asDiagonal();
  const leg_t& lp = _params.leg[leg];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    const link_t& link = lp.joint[i].link;
    const Eigen::Matrix3d inertia_body =
        kin.link_rotation[i] * link.inertia_com * kin.link_rotation[i].transpose();
    mass.noalias() += link.mass *
                      kin.com_linear_jacobian[i].transpose() *
                      kin.com_linear_jacobian[i];
    mass.noalias() += kin.com_angular_jacobian[i].transpose() * inertia_body *
                      kin.com_angular_jacobian[i];
  }
  mass = 0.5 * (mass + mass.transpose());
  return mass.allFinite();
}

bool QuadrupedLegDynamics::footPosition(int leg, const Eigen::Vector3d& q,
                                        Eigen::Vector3d& position) const {
  kinematics_t kin;
  if (!_kinematics(leg, q, kin)) return false;
  position = kin.foot_position;
  return true;
}

bool QuadrupedLegDynamics::footJacobian(int leg, const Eigen::Vector3d& q,
                                        Eigen::Matrix3d& jacobian) const {
  kinematics_t kin;
  if (!_kinematics(leg, q, kin)) return false;
  jacobian = kin.foot_jacobian;
  return true;
}

bool QuadrupedLegDynamics::compute(int leg, const Eigen::Vector3d& q,
                                   const Eigen::Vector3d& qdot,
                                   const Eigen::Vector3d& gravity_body,
                                   terms_t& terms) const {
  if (!_valid || leg < 0 || leg >= NUM_LEGS || !q.allFinite() || !qdot.allFinite() ||
      !gravity_body.allFinite())
    return false;

  kinematics_t kin;
  if (!_kinematics(leg, q, kin) || !_massMatrix(leg, q, terms.mass_matrix)) return false;
  Eigen::LDLT<Eigen::Matrix3d> mass_solver(terms.mass_matrix);
  if (mass_solver.info() != Eigen::Success || !mass_solver.isPositive()) return false;

  terms.foot_position = kin.foot_position;
  terms.foot_jacobian = kin.foot_jacobian;

  // Gravity generalized force. With gravity_body pointing down, the equation
  // M*qdd + C*qdot + G = tau has G = -sum(Jcom^T*m*g).
  terms.bias.setZero();
  const leg_t& lp = _params.leg[leg];
  for (int i = 0; i < NUM_JOINTS; ++i)
    terms.bias.noalias() -=
        kin.com_linear_jacobian[i].transpose() * lp.joint[i].link.mass * gravity_body;

  // For a three-joint leg, differentiating the analytic mass matrix is both
  // inexpensive and less convention-prone than duplicating a spatial-algebra
  // implementation. Central differences feed the exact Christoffel identity.
  Eigen::Matrix3d dmass[NUM_JOINTS];
  const double eps = _params.derivative_step;
  for (int k = 0; k < NUM_JOINTS; ++k) {
    Eigen::Vector3d qp = q;
    Eigen::Vector3d qm = q;
    qp[k] += eps;
    qm[k] -= eps;
    Eigen::Matrix3d mp, mm;
    if (!_massMatrix(leg, qp, mp) || !_massMatrix(leg, qm, mm)) return false;
    dmass[k] = (mp - mm) / (2.0 * eps);
  }
  for (int i = 0; i < NUM_JOINTS; ++i) {
    double coriolis = 0.0;
    for (int j = 0; j < NUM_JOINTS; ++j)
      for (int k = 0; k < NUM_JOINTS; ++k)
        coriolis += 0.5 *
                    (dmass[k](i, j) + dmass[j](i, k) - dmass[i](j, k)) * qdot[j] *
                    qdot[k];
    terms.bias[i] += coriolis;
  }

  if (qdot.squaredNorm() == 0.0) {
    terms.jdot_qdot.setZero();
  } else {
    Eigen::Matrix3d jp, jm;
    if (!footJacobian(leg, q + eps * qdot, jp) ||
        !footJacobian(leg, q - eps * qdot, jm))
      return false;
    terms.jdot_qdot = ((jp - jm) / (2.0 * eps)) * qdot;
  }

  Eigen::Matrix3d inverse_operational =
      terms.foot_jacobian * mass_solver.solve(terms.foot_jacobian.transpose());
  inverse_operational = 0.5 * (inverse_operational + inverse_operational.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(inverse_operational);
  if (eigen_solver.info() != Eigen::Success) return false;
  const Eigen::Vector3d eigenvalues = eigen_solver.eigenvalues();
  if (eigenvalues.minCoeff() < _params.min_operational_eigenvalue ||
      eigenvalues.maxCoeff() / eigenvalues.minCoeff() > _params.max_operational_condition)
    return false;
  terms.operational_inertia =
      eigen_solver.eigenvectors() * eigenvalues.cwiseInverse().asDiagonal() *
      eigen_solver.eigenvectors().transpose();
  terms.operational_inertia =
      0.5 * (terms.operational_inertia + terms.operational_inertia.transpose());

  return terms.mass_matrix.allFinite() && terms.bias.allFinite() &&
         terms.foot_jacobian.allFinite() && terms.foot_position.allFinite() &&
         terms.jdot_qdot.allFinite() && terms.operational_inertia.allFinite();
}
