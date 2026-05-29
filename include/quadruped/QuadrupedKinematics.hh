/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef QUADRUPEDKINEMATICS_HH
#define QUADRUPEDKINEMATICS_HH

#include <Eigen/Dense>
#include <string>
#include <vector>

/**
 * @brief Configurable kinematics class for quadruped robots
 *
 * This class provides forward and inverse kinematics for legs on a quadruped
 * robots with configurable kinematic parameters. It supports different robot
 * platforms by allowing customization of link lengths, joint limits, and hip
 * positions. The origin frame is coincident with the body frame.
 *
 * Coordinate system:
 * - +X: forward (from rear to front of robot)
 * - +Y: left (from right to left of robot)
 * - +Z: up (from ground to top of robot)
 *
 * Leg numbering:
 * - 0: Front Left (FL)
 * - 1: Front Right (FR)
 * - 2: Rear Left (RL)
 * - 3: Rear Right (RR)
 *
 * Joint angle conventions:
 * - Hip abduction: positive is outward (away from body).
 * - Hip flexion: positive is backward (towards body)
 * - Knee: positive is extension (straightening)
 */
class QuadrupedKinematics {
 public:
  /** \brief Enum type for leg indices */
  enum LegIndex {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    NUM_LEGS = 4
  };

  /** \brief Enum type for joint indices for each leg */
  enum JointIndex {
    HIP_ABDUCTION = 0,  // Hip abduction/adduction joint
    HIP_FLEXION = 1,    // Hip flexion/extension joint (thigh)
    KNEE = 2,           // Knee joint (calf)
    NUM_JOINTS_PER_LEG = 3
  };

  /** \brief Structure to encapsulate relevant kinematic parameters for quadrupeds */
  struct params_t {
    /** \brief Name of the robot platform. Only used for informational purposes */
    std::string robot_name;

    /** \brief Positions of all hip joints in the body fram e (origin of the
        the topmost abduction joint) (meters)*/
    Eigen::Matrix<double, NUM_LEGS, 3> hip_positions;

    /** \brief Link lengths for each leg (meters) */
    Eigen::Matrix<double, NUM_LEGS, 2>
        link_lengths;  // [thigh_length, calf_length] per leg

    /** \brief Offset between hip abduction and hip flexion joint axes along
      the y axis of the abduction joint output (meters) */
    Eigen::Matrix<double, NUM_LEGS, 1> hip_flexion_offset;

    /**\brief Joint limits for each leg [min, max] (radians) */
    Eigen::Matrix<double, NUM_LEGS, 2>
        hip_abduction_limits;  // Hip abduction limits per leg
    Eigen::Matrix<double, NUM_LEGS, 2> hip_flexion_limits;  // Hip flexion limits per leg
    Eigen::Matrix<double, NUM_LEGS, 2> knee_limits;         // Knee limits per leg

    /**\brief Joint directions (1.0 or -1.0) for sign conventions. 1.0
        chooses the Right-Hand-Rule direction, -1.0 the opposite */
    Eigen::Matrix<double, NUM_LEGS, 3>
        joint_directions;  // [abd, flex, knee] directions per leg

    /** \brief Returns true when the parameter selection is consistent and valid */
    bool validate() const;
  };

  /** \brief Constructs a kinematic object with the supplied parameters */
  explicit QuadrupedKinematics(const params_t& params);

  /** \brief Sets parameters for kinematic computations. Returns false if validation fails  */
  bool setKinematicParams(const params_t& params);

  /** \brief Returns the current set of parameters  */
  const params_t& getKinematicParams() const;

  /** @brief Computes forward kinematics for a single leg. Returns true if successful,
   * false if problem
   * @param leg_idx Leg index (0-3)
   * @param angles Joint angles [hip_abduction, hip_flexion, knee] in radians
   * @param footpos Foot position in body frame [x, y, z] in meters
   */
  bool forwardKinematics(int leg_idx, const Eigen::Vector3d& angles,
                         Eigen::Vector3d& footpos) const;

  /** @brief Unchecked FK — computes foot position without joint limit validation.
   *  Use for state estimation where actual angles may be outside configured limits. */
  bool forwardKinematicsUnchecked(int leg_idx, const Eigen::Vector3d& angles,
                                  Eigen::Vector3d& footpos) const;

  /** @brief Computes inverse kinematics for a single leg. Returns true if solution found
   * within joint limits, false otherwise
   * @param leg_idx Leg index (0-3)
   * @param footpos Desired foot position in body frame [x, y, z] in meters
   * @param angles Output joint angles [hip_abduction, hip_flexion, knee] in radians
   */
  bool inverseKinematics(int leg_idx, const Eigen::Vector3d& footpos,
                         Eigen::Vector3d& angles) const;

  /** @brief Computes the Jacobian matrix from joint angles to foot position for a single
   * leg, Returns true if computation was successful
   *
   * @param leg_idx Leg index (0-3)
   * @param angles Joint angles [hip_abduction, hip_flexion, knee] in radians
   * @param J Computed 3x3 Jacobian matrix (foot velocity = J * joint velocities)
  (modified in place)
   */
  bool jacobian(int leg_idx, const Eigen::Vector3d& angles, Eigen::Matrix3d& J) const;

  /** @brief Returns true if all angles are within limits
   * @param leg_idx Leg index (0-3)
   * @param angles Joint angles to check
   */
  bool checkJointLimits(int leg_idx, const Eigen::Vector3d& angles) const;

  /** @brief Clamps joint angles to their respective limits for a specific leg
   * @param leg_idx Leg index (0-3)
   * @param angles Joint angles to clamp (modified in place)
   */
  void clampToJointLimits(int leg_idx, Eigen::Vector3d& angles) const;

 private:
  params_t params_;

  // Core FK geometry — no validation
  void computeFK(int leg_idx, const Eigen::Vector3d& angles,
                 Eigen::Vector3d& footpos) const;
};

#endif  // QUADRUPEDKINEMATICS_HH