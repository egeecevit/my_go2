#ifndef TRAJECTORYUTILS_HH
#define TRAJECTORYUTILS_HH

namespace TrajectoryUtils {

// Quintic time-scaling with zero velocity and acceleration at both ends.
// tau in [0,1], T is the total duration (for velocity scaling).
// sigma goes 0 -> 1, sigma_dot goes 0 -> 0 with a peak at tau=0.5.
inline void sampleQuintic(double tau, double T, double &sigma, double &sigma_dot,
                          double &sigma_ddot) {
  if (tau <= 0.0) {
    sigma = 0.0;
    sigma_dot = 0.0;
    sigma_ddot = 0.0;
    return;
  }
  if (tau >= 1.0) {
    sigma = 1.0;
    sigma_dot = 0.0;
    sigma_ddot = 0.0;
    return;
  }
  double t2 = tau * tau;
  double t3 = t2 * tau;
  sigma = 10.0 * t3 - 15.0 * t3 * tau + 6.0 * t3 * tau * tau;
  sigma_dot = (30.0 * t2 - 60.0 * t2 * tau + 30.0 * t2 * tau * tau) / T;
  sigma_ddot = (60.0 * tau - 180.0 * t2 + 120.0 * t3) / (T * T);
}

inline void sampleQuintic(double tau, double T, double &sigma, double &sigma_dot) {
  double sigma_ddot;
  sampleQuintic(tau, T, sigma, sigma_dot, sigma_ddot);
}

}  // namespace TrajectoryUtils

#endif
