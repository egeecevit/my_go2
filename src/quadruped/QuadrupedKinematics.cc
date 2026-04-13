/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include <algorithm>
#include <cmath>
#include <iostream>

#include "quadruped/QuadrupedKinematics.hh"

#define DBGPRINT(...) //printf(__VA_ARGS__)

bool QuadrupedKinematics::params_t::validate() const {
  // Check that all link lengths are positive
  for (int i = 0; i < NUM_LEGS; ++i) {
    if (link_lengths(i, 0) <= 0 || link_lengths(i, 1) <= 0) {
      DBGPRINT("QuadrupedKinematics: Invalid link lengths for leg %d\n", i);
      return false;
    }
  }

  // Check that joint limits are valid (min < max)
  for (int i = 0; i < NUM_LEGS; ++i) {
    if (hip_abduction_limits(i, 0) >= hip_abduction_limits(i, 1) ||
        hip_flexion_limits(i, 0) >= hip_flexion_limits(i, 1) ||
        knee_limits(i, 0) >= knee_limits(i, 1)) {
      DBGPRINT("QuadrupedKinematics: Invalid joint limits for leg %d\n", i);
      return false;
    }
  }

  // Check that joint directions are ±1
  for (int i = 0; i < NUM_LEGS; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (std::abs(joint_directions(i, j)) != 1.0) {
        DBGPRINT(
            "QuadrupedKinematics: Invalid joint direction for leg %d, joint "
            "%d\n",
            i, j);
        return false;
      }
    }
  }

  return true;
}

// ===== QuadrupedKinematics Implementation =====

QuadrupedKinematics::QuadrupedKinematics(const params_t& params) : params_(params) {
  if (!params_.validate()) {
    DBGPRINT("QuadrupedKinematics: Invalid kinematic parameters provided\n");
  }
}

void QuadrupedKinematics::computeFK(int leg_id,
                                    const Eigen::Vector3d& joint_angles,
                                    Eigen::Vector3d& footpos) const {
  Eigen::Vector3d q =
      joint_angles.cwiseProduct(params_.joint_directions.row(leg_id).transpose());

  double q1 = q(0), q2 = q(1), q3 = q(2);
  double l1 = params_.link_lengths(leg_id, 0);
  double l2 = params_.link_lengths(leg_id, 1);
  double offset = params_.hip_flexion_offset(leg_id);
  Eigen::Vector3d hip_pos = params_.hip_positions.row(leg_id).transpose();

  double flex_x = hip_pos(0);
  double flex_y = hip_pos(1) + offset * std::cos(q1);
  double flex_z = hip_pos(2) + offset * std::sin(q1);

  footpos(0) = flex_x - l1 * std::sin(q2) - l2 * std::sin(q2 + q3);
  footpos(1) = flex_y - (-l1 * std::cos(q2) - l2 * std::cos(q2 + q3)) * std::sin(q1);
  footpos(2) = flex_z + (-l1 * std::cos(q2) - l2 * std::cos(q2 + q3)) * std::cos(q1);
}

bool QuadrupedKinematics::forwardKinematics(int leg_id,
                                            const Eigen::Vector3d& joint_angles,
                                            Eigen::Vector3d& footpos) const {
  if (leg_id < 0 || leg_id >= NUM_LEGS || !checkJointLimits(leg_id, joint_angles)) {
    DBGPRINT(
        "QuadrupedKinematics: Invalid leg index %d or joint angles out of "
        "limits\n",
        leg_id);
    return false;
  }
  computeFK(leg_id, joint_angles, footpos);
  return true;
}

bool QuadrupedKinematics::forwardKinematicsUnchecked(int leg_id,
                                                      const Eigen::Vector3d& joint_angles,
                                                      Eigen::Vector3d& footpos) const {
  if (leg_id < 0 || leg_id >= NUM_LEGS)
    return false;
  computeFK(leg_id, joint_angles, footpos);
  return true;
}

bool QuadrupedKinematics::computeIK(int leg_idx,
                                     const Eigen::Vector3d& foot_position,
                                     Eigen::Vector3d& joint_angles) const {
  Eigen::Vector3d hip_pos = params_.hip_positions.row(leg_idx).transpose();

  double l1 = params_.link_lengths(leg_idx, 0);
  double l2 = params_.link_lengths(leg_idx, 1);
  double offset = params_.hip_flexion_offset(leg_idx);

  Eigen::Vector3d rel_pos = foot_position - hip_pos;

  double hip_dist = std::sqrt(rel_pos(1) * rel_pos(1) + rel_pos(2) * rel_pos(2));
  if (hip_dist < offset - 1e-6) {
    DBGPRINT("QuadrupedKinematics: Target foot position too close to hip for leg %d\n", leg_idx);
    return false;
  } else
    hip_dist = offset;

  double a = std::sqrt(rel_pos(1) * rel_pos(1) + rel_pos(2) * rel_pos(2));
  double alpha = std::atan2(rel_pos(2), rel_pos(1));
  double q1 = std::acos(offset / a);

  if (alpha < 0)
    q1 += alpha;
  else
    q1 = -q1 + alpha;

  Eigen::Vector3d flex_pos = hip_pos;
  flex_pos(1) += offset * std::cos(q1);
  flex_pos(2) += offset * std::sin(q1);

  Eigen::Vector3d rel_flex_pos = foot_position - flex_pos;

  double xbar = -rel_flex_pos(2) / std::cos(q1);
  double ybar = -rel_flex_pos(0);

  double q3 = -std::acos((xbar * xbar + ybar * ybar - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  if (std::isnan(q3) || std::isinf(q3)) {
    DBGPRINT("QuadrupedKinematics: Invalid q3 calculation for leg %d\n", leg_idx);
    return false;
  }
  double q2 =
      std::atan2(ybar, xbar) - std::atan2(l2 * std::sin(q3), (l1 + l2 * std::cos(q3)));
  if (std::isnan(q2) || std::isinf(q2)) {
    DBGPRINT("QuadrupedKinematics: Invalid q2 calculation for leg %d\n", leg_idx);
    return false;
  }

  joint_angles = Eigen::Vector3d(q1, q2, q3);

  for (int i = 0; i < 3; ++i) {
    joint_angles(i) /= params_.joint_directions(leg_idx, i);
  }

  return true;
}

bool QuadrupedKinematics::inverseKinematics(int leg_idx,
                                            const Eigen::Vector3d& foot_position,
                                            Eigen::Vector3d& joint_angles) const {
  if (leg_idx < 0 || leg_idx >= NUM_LEGS) {
    DBGPRINT("QuadrupedKinematics: Invalid leg index %d\n", leg_idx);
    return false;
  }
  if (!computeIK(leg_idx, foot_position, joint_angles))
    return false;
  return checkJointLimits(leg_idx, joint_angles);
}

bool QuadrupedKinematics::inverseKinematicsClamped(int leg_idx,
                                                    const Eigen::Vector3d& foot_position,
                                                    Eigen::Vector3d& joint_angles) const {
  if (leg_idx < 0 || leg_idx >= NUM_LEGS) {
    DBGPRINT("QuadrupedKinematics: Invalid leg index %d\n", leg_idx);
    return false;
  }
  if (!computeIK(leg_idx, foot_position, joint_angles))
    return false;
  clampToJointLimits(leg_idx, joint_angles);
  return true;
}

// Check if joint angles are within limits
bool QuadrupedKinematics::checkJointLimits(int leg_id,
                                           const Eigen::Vector3d& joint_angles) const {
  if (leg_id < 0 || leg_id >= NUM_LEGS) {
    return false;
  }

  // Check hip abduction limits
  if (joint_angles(0) < params_.hip_abduction_limits(leg_id, 0) ||
      joint_angles(0) > params_.hip_abduction_limits(leg_id, 1)) {
    DBGPRINT(
        "QuadrupedKinematics: Joint angle out of hip abduction limits for leg "
        "%d\n",
        leg_id);
    return false;
  }

  // Check hip flexion limits
  if (joint_angles(1) < params_.hip_flexion_limits(leg_id, 0) ||
      joint_angles(1) > params_.hip_flexion_limits(leg_id, 1)) {
    DBGPRINT(
        "QuadrupedKinematics: Joint angle out of hip flexion limits for leg "
        "%d\n",
        leg_id);
    return false;
  }

  // Check knee limits
  if (joint_angles(2) < params_.knee_limits(leg_id, 0) ||
      joint_angles(2) > params_.knee_limits(leg_id, 1)) {
    DBGPRINT("QuadrupedKinematics: Joint angle out of knee limits for leg %d\n", leg_id);
    return false;
  }

  return true;
}

bool QuadrupedKinematics::jacobian(int leg_idx, const Eigen::Vector3d& joint_angles, Eigen::Matrix3d &J) const {
  if (leg_idx < 0 || leg_idx >= NUM_LEGS) {
    DBGPRINT("QuadrupedKinematics: Invalid leg index %d\n", leg_idx);
    return false;
  }

  // Apply joint directions
  Eigen::Vector3d q =
      joint_angles.cwiseProduct(params_.joint_directions.row(leg_idx).transpose());

  double q1 = q(0);  // Hip abduction
  double q2 = q(1);  // Hip flexion
  double q3 = q(2);  // Knee

  double l1 = params_.link_lengths(leg_idx, 0);         // Thigh length
  double l2 = params_.link_lengths(leg_idx, 1);         // Calf length
  double offset = params_.hip_flexion_offset(leg_idx);  // Hip flexion offset

  // Initialize Jacobian matrix
  J = Eigen::Matrix3d::Zero();

  // Calculate common trigonometric terms
  double s1 = std::sin(q1);
  double c1 = std::cos(q1);
  double s2 = std::sin(q2);
  double c2 = std::cos(q2);
  double s23 = std::sin(q2 + q3);
  double c23 = std::cos(q2 + q3);

  double ly = -l1 * c2 - l2 * c23;

  // First column - partial derivatives with respect to hip abduction (q1)
  J(0, 0) = 0;  // dx/dq1 (no x-component affected by abduction)

  // Effect of abduction on y-coordinate
  J(1, 0) = -offset * s1 - ly * c1;

  // Effect of abduction on z-coordinate
  J(2, 0) = offset * c1 - ly * s1;

  // Second column - partial derivatives with respect to hip flexion (q2)
  J(0, 1) = -l1 * c2 - l2 * c23;         // dx/dq2
  J(1, 1) = -s1 * (l1 * s2 + l2 * s23);  // dy/dq2
  J(2, 1) = c1 * (l1 * s2 + l2 * s23);   // dz/dq2

  // Third column - partial derivatives with respect to knee (q3)
  J(0, 2) = -l2 * c23;       // dx/dq3
  J(1, 2) = -l2 * s1 * s23;  // dy/dq3
  J(2, 2) = l2 * c1 * s23;   // dz/dq3

  // Apply joint directions to convert from internal representation to external
  for (int col = 0; col < 3; ++col) {
    J.col(col) *= params_.joint_directions(leg_idx, col);
  }
  return true;
}

// Get current parameters
const QuadrupedKinematics::params_t& QuadrupedKinematics::getKinematicParams() const {
  return params_;
}

// Set new parameters
bool QuadrupedKinematics::setKinematicParams(const params_t& params) {
  if (!params.validate()) return false;
  params_ = params;
  return true;
}

// Clamp joint angles to their respective limits
void QuadrupedKinematics::clampToJointLimits(int leg_id,
                                             Eigen::Vector3d& joint_angles) const {
  if (leg_id < 0 || leg_id >= NUM_LEGS) return;

  // Clamp hip abduction
  joint_angles(0) =
      std::max(params_.hip_abduction_limits(leg_id, 0),
               std::min(params_.hip_abduction_limits(leg_id, 1), joint_angles(0)));

  // Clamp hip flexion
  joint_angles(1) =
      std::max(params_.hip_flexion_limits(leg_id, 0),
               std::min(params_.hip_flexion_limits(leg_id, 1), joint_angles(1)));

  // Clamp knee angle
  joint_angles(2) = std::max(params_.knee_limits(leg_id, 0),
                             std::min(params_.knee_limits(leg_id, 1), joint_angles(2)));
}
