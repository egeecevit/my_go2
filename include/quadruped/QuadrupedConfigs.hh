#ifndef QUADRUPEDCONFIGS_HH
#define QUADRUPEDCONFIGS_HH

#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"

// Per-hardware-target kinematic parameters. Used by MdlLegControl, MdlDrawSquare,
// MdlSit, and anything else that needs FK/IK with the right dimensions.
QuadrupedKinematics::params_t createGo1Config();
QuadrupedKinematics::params_t createGo2Config();

// Go2 fixed-base leg inertial model used by swing inverse dynamics.
QuadrupedLegDynamics::params_t createGo2DynamicsConfig();

#endif
