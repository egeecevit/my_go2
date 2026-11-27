/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#ifndef MDLAMB_HH
#define MDLAMB_HH

#include "rtcore/Module.hh"
#include "rtcore/LogServer.hh"
#include "hardware/IMUHW.hh"
#include <Eigen/Dense>
#include <vector>

#define AMBMODULE_NAME "MdlAMB"

class MdlConvexMPC;
class MdlLegControl;
class MdlOrientationEstimator;
class QuadrupedDynamics;
class QuadrupedKinematics;
class QuadrupedLegDynamics;
class QuadrupedWBC;

/** \brief Angular-momentum balancing on the FL/RR diagonal stance.

  Lifts FR and RL and balances about the FL-RR support line by swinging the two
  free legs, following the Ozden thesis template controller. Two realizations
  of the same reduced-order law, selected by [amb] wbc: joint space (hip-roll
  computed torque, lever-rule stance forces), or the thesis Ch. 6 whole-body
  one (AMB as a principal-frame swing-foot task of QuadrupedWBC, stance forces
  from MdlConvexMPC). The pure-math pieces are static so they can be unit
  tested without hardware (pattern: MdlLegControl::computeCartesianForceCommand). */
class MdlAMB : public rtcore::Module {
public:
  // Balancing alone never settles. SETTLED is reached only in WBC mode, after
  // stopBalancing() has placed the swing feet, transferred the load and held
  // four-foot support (thesis Sec. 6.6.7); the module keeps holding that
  // stance until it is released.
  enum Status { IDLE, ACTIVE, SETTLED, ERROR };

  struct input_t {
    Eigen::Vector3d q[4], qd[4];  // controller-convention joint state
    Eigen::Matrix3d R;            // body -> world
    Eigen::Vector3d w_body;       // body-frame angular rate
    double t;
  };

  /** Per-leg MIT-mode command (see MdlLegControl::setJointCommand). */
  struct cmd_t { Eigen::Vector3d a, adot, kp, kd, tau; };

  /** Every [amb] key; the defaults are the amb.toml values. */
  struct params_t {
    bool enabled = true;
    Eigen::Vector3d template_m = Eigen::Vector3d(14.954, 0.2296, 0.0594);
    Eigen::Vector3d template_l = Eigen::Vector3d(0.30605, 0.31773, 0.56403);
    double template_d = 0.093;
    double x3_ref_deg = -304.0;
    Eigen::Vector3d gains = Eigen::Vector3d(950.0, 360.0, 41.0);
    double kp3 = 100.0;
    double kd3 = 40.0;
    double observer_tc = 0.05;
    double kp_stance = 60.0;
    double kd_stance = 2.5;
    double kp_swing = 50.0;
    double kd_swing = 1.0;
    double kp_height = 400.0;
    double kd_height = 10.0;
    double qdd_max = 300.0;
    double barrier_margin_deg = 8.0;
    double barrier_kp = 60.0;
    double barrier_kd = 2.0;
    double joint_damping = 0.2;
    double joint_friction = 0.2;
    double lift_duration = 0.4;
    double stance_x = 0.15;
    double base_z = 0.30;
    Eigen::Vector3d swing_rel = Eigen::Vector3d(0.07, 0.0955, 0.18);
    double fd_step = 1e-4;
    double theta1_max_deg = 45.0;
    double z_min = 0.16;
    // Whole-body realization (thesis Ch. 6); false keeps the joint-space one.
    // Gains and limits are Table 6.2's, except the body orientation (100, 10
    // there): see amb.toml.
    bool wbc = false;
    double kp_body_ori = 400.0;
    double kd_body_ori = 40.0;
    double kp_body_pos = 100.0;
    double kd_body_pos = 10.0;
    double kp_foot = 150.0;
    double kd_foot = 10.0;
    double kp_joint = 30.0;
    double kd_joint = 3.0;
    double foot_acc_max = 50.0;
    double force_ratio = 0.10;
    // Experiment inputs of thesis Sec. 6.6 (Table 6.3); the defaults leave
    // them off. x3_schedule: (time since activation [s], x3 reference [deg])
    // pairs in increasing time; once the lift is over, each replaces
    // x3_ref_deg from its time on (Sec. 6.6.3, both realizations).
    std::vector<Eigen::Vector2d> x3_schedule;
    // Amplitudes [deg] of the 0.1 Hz pitch and yaw sinusoids added to the
    // latched body orientation (Sec. 6.6.5.1, WBC mode only).
    double pitch_amp_deg = 0.0;
    double yaw_amp_deg = 0.0;
    // Swing-foot placement after a stop request (Sec. 6.6.7, Fig. 6.3) [s]:
    // the normal reference back to its hand-off value and the radial one out
    // to the ground (not to rho0: x2 drifts), more slowly than the lift, with
    // AMB still acting.
    double place_duration = 2.5;
  };

  /** Cross-bar template kinematics in the x = (theta1, theta2+theta3,
      theta2-theta3) coordinates. */
  struct kin_t {
    Eigen::Matrix3d Mt;
    Eigen::Vector3d Ct, Gt, dcx, dcy;
    double cx, cy, cx_bias, cy_bias, m_tot;
  };

  /** Robot state projected onto the support line: template angles, support
      frame and the CoM relative to the support midpoint. */
  struct proj_t {
    Eigen::Vector3d theta, s, o, tv, th;
    double c_h, c_v, l_c, lever_a, lever_b;
    double mass;  // whole robot, summed from the same model as the points
    // Rates by finite difference along (qd, w_body), and the swing hip-roll
    // Jacobians d theta2 / d q_sw2,0, d theta3 / d q_sw3,0 measured the same way.
    Eigen::Vector3d thetad;
    double ch_dot, lc_dot, j_c, j_d;
  };

  /** An ordered support pair: the support line s runs from foot a to foot b,
      and sw2, sw3 are the swing legs that carry theta2 and theta3. FL_RR is
      the diagonal MdlAMB balances on. The other diagonal is its mirror image,
      and a mirror flips the sign of every angle and momentum about the line;
      running the line from RL to FR flips them back, so t_h = s x t_v, the
      theta2/theta3 branches, L = s . H and e_t = s x e_r hold for RL_FR
      unchanged (test_trot_amb checks this against the mirrored state). */
  struct pair_t { int a, b, sw2, sw3; };
  static constexpr pair_t FL_RR{0, 3, 1, 2};
  static constexpr pair_t RL_FR{2, 1, 0, 3};

  MdlAMB();
  ~MdlAMB();

  void init();
  void uninit();
  void activate();
  void deactivate();
  void update();

  Status getStatus() const { return _status; }

  /** Ends balancing with the thesis Sec. 6.6.7 sequence (WBC mode): once the
      lift is over the swing feet are placed, each becomes a contact when the
      ground has stopped it, the normal load moves to four feet over 3 s, and
      SETTLED follows a 3 s four-foot hold. False, and nothing
      changes, in the joint-space realization, which has no such sequence: the
      caller sits right away, as before. Repeated calls are harmless. */
  bool stopBalancing();

  /** Support frame, template angles and CoM offset at one configuration, all
      relative to the base origin in world axes. False on a kinematics failure. */
  static bool geometry(const QuadrupedLegDynamics& dyn, const Eigen::Vector3d q[4],
                       const Eigen::Matrix3d& R, const pair_t& pair, proj_t& out);
  /** Angular momentum [kg m^2/s] of the whole robot about the line (o, s), the
      base velocity taken from the pair's two planted feet. */
  static double robotMomentum(const QuadrupedLegDynamics& dyn, const Eigen::Vector3d q[4],
                              const Eigen::Vector3d qd[4], const Eigen::Matrix3d& R,
                              const Eigen::Vector3d& w, const Eigen::Vector3d& o,
                              const Eigen::Vector3d& s, const pair_t& pair);
  /** geometry() plus its rates and the swing hip-roll Jacobians j_c, j_d.
      False on a kinematics failure. */
  static bool project(const params_t& p, const input_t& in, const QuadrupedLegDynamics& dyn,
                      proj_t& out, const pair_t& pair = FL_RR);
  static void templateKin(const params_t& p, const Eigen::Vector3d& x,
                          const Eigen::Vector3d& xd, kin_t& k);
  /** Template x1 whose CoM offset m_tot c_x equals the robot's M c_h, seeded
      from the geometric theta1 in x(0). */
  static double comConsistentX1(const params_t& p, const Eigen::Vector3d& x,
                                const Eigen::Vector3d& xd, double M_ch);
  /** False when the momentum manifold is singular (|det| < 1e-9). */
  static bool ambLaw(const params_t& p, const kin_t& k, const Eigen::Vector3d& x,
                     const Eigen::Vector3d& xd, double L, double Ld, double Ldd,
                     double tau_hat, double x3d, Eigen::Vector3d& xdd);
  /** Split the stance wrench F into FL and RR contact forces f[0], f[1]. */
  static void stanceForces(const Eigen::Vector3d& F, const proj_t& pr, Eigen::Vector3d f[2]);
  /** Hip-centred principal frame of a swing leg (thesis 6.13-6.14): R_WP =
      [e_n e_r e_t], e_r the hip->foot vector d_f projected into the plane
      normal to the unit e_n, e_t = e_n x e_r, rho the projected radius. False,
      outputs untouched, when rho is too small to define e_r: the reference is
      then rejected instead of normalizing a degenerate vector (Sec. 6.3.2). */
  static bool principalFrame(const Eigen::Vector3d& e_n, const Eigen::Vector3d& d_f,
                             Eigen::Matrix3d& R_WP, double& rho);
  /** Tangential arc coordinate (6.16) of d_f about the stored radial axis
      e_r0 reprojected into the plane normal to e_n, at the radius rho_d. */
  static double tangentialArc(const Eigen::Vector3d& e_n, const Eigen::Vector3d& e_r0,
                              double rho_d, const Eigen::Vector3d& d_f);
  /** Desired hip->foot vector (6.20)-(6.22), (6.24): normal offset xi_n,
      radius rho_d, arc xi_t about the reprojected e_r0; and e_t^d (6.22). */
  static void swingReference(const Eigen::Vector3d& e_n, const Eigen::Vector3d& e_r0,
                             double xi_n, double rho_d, double xi_t, Eigen::Vector3d& d_des,
                             Eigen::Vector3d& e_t_des);
  /** AMB foot feedforward rho thdd e_t (6.18), (6.26), scaled down to the
      Euclidean norm a_max; true when it had to be. */
  static bool tangentialFeedforward(double rho, double thdd, const Eigen::Vector3d& e_t,
                                    double a_max, Eigen::Vector3d& a_f);
  /** Whole tick; mutates the observer, lift and latch state. The first call
      after activate() latches t0, l_nom, x3(t0), the swing feet and the
      nominal targets (in WBC mode also the body and principal references),
      so a test can drive it without a ModuleManager. After stopBalancing()
      it runs the placement sequence and sets SETTLED at its end. */
  bool computeTick(const input_t& in, cmd_t out[4]);

  /** Tests run without a ModuleManager, so the [amb] table is never read. */
  void setParams(const params_t& p) { _p = p; }
  /** The stance-force planner of WBC mode; init() finds it when there is a
      ModuleManager, a bare instance must be handed one before computeTick(). */
  void setSolver(MdlConvexMPC* mpc) { _mpc = mpc; }
  /** The MdlAMB_state log of the last tick (layout documented at _log). */
  const double* logState() const { return _log; }
  /** The MdlAMB_wbc log of the last WBC tick (layout documented at _logWbc). */
  const double* logWbc() const { return _logWbc; }

private:
  MdlLegControl *_legs[4] = {};
  // Set only for [amb] attitude_source = "estimator": R and the body rate then
  // come from MdlOrientationEstimator (filtered, with its synthetic noise in
  // simulation) instead of the raw IMU sample.
  MdlOrientationEstimator *_orientation = nullptr;
  QuadrupedLegDynamics *_dyn = nullptr;
  QuadrupedKinematics *_kin = nullptr;
  params_t _p;
  Status _status = IDLE;
  // Kept across ticks: getLastReading() leaves it untouched when the IMU has
  // nothing newer, so a slow IMU reuses its last sample instead of garbage.
  // Consecutive ticks without a newer sample; a stalled IMU is an ERROR.
  IMUHW::imudata_t _imu;
  int _imuMiss = 0;

  double _t0 = 0.0;
  // Observer and latch state, reset in activate().
  double _l_nom = 0.0;
  double _tau_hat = 0.0;
  double _L_prev = 0.0;
  double _ch_prev = 0.0;
  double _t_prev = 0.0;
  bool _first = true;

  Eigen::Vector3d _q_nom[4];
  Eigen::Vector3d _p_sw0[2], _p_sw1[2];
  double _x3d0 = 0.0;

  // Reported by deactivate(): tip excursion once the lift is over, and how
  // often the MIT-law torque of any joint exceeded its motor limit.
  double _maxTheta1 = 0.0;
  int _ticks = 0, _satTicks = 0;
  // Wall time of computeTick() in update(), measured and reported in WBC mode
  // only (its MPC re-solve has to fit the control period).
  double _tickSum = 0.0, _tickMax = 0.0;
  int _tickCount = 0;

  // WBC mode (thesis Ch. 6). Latched by the first tick: body orientation and
  // ZYX angles, base position from the support midpoint (world axes), and per
  // swing leg (FR, RL) the principal reference xi_n0, rho0, e_r0 and the
  // normal, radial and angular (about e_r0) coordinates of the lift target.
  MdlConvexMPC *_mpc = nullptr;
  QuadrupedDynamics *_wb = nullptr;
  QuadrupedWBC *_wbc = nullptr;
  Eigen::Matrix3d _R_ref;
  Eigen::Vector3d _rpy_ref, _p_ref;
  Eigen::Vector3d _e_r0[2];
  double _xi_n0[2], _rho0[2], _xi_n1[2], _rho1[2], _alpha1[2];
  // Next MPC solve [s] and its tangentially limited forces, per leg (zero on a
  // foot outside the MPC's contact schedule).
  double _tMpc = 0.0;
  Eigen::Vector3d _f_des[4];
  // Stop sequence (Sec. 6.6.7), reset in activate(). Times are since
  // activation, negative until reached: placement start (with x3d there),
  // load-transfer start (both swing feet promoted to contacts, FR then RL in
  // _promoted), and the two-foot desired forces frozen at that instant.
  bool _stopReq = false;
  double _tPlace = -1.0, _x3dStop = 0.0, _tTransfer = -1.0;
  bool _promoted[2] = {false, false};
  Eigen::Vector3d _f_frz[4];
  // sigma: the lift ramp, place: the placement ramp (zero before it starts),
  // each with its first and second time derivatives.
  bool _wbcTick(const input_t& in, const proj_t& pr, const double thdd[2], double sigma,
                double sigma_dot, double sigma_ddot, double place, double place_dot,
                double place_ddot, cmd_t out[4]);
  // Body-orientation reference at t [s] since activation: the latched one
  // with the Sec. 6.6.5.1 sinusoids on its ZYX pitch and yaw, as a rotation,
  // ZYX angles, and world-axes rate.
  void _oriReference(double t, Eigen::Matrix3d& R_d, Eigen::Vector3d& rpy_d,
                     Eigen::Vector3d& w_d) const;

  rtcore::LogServer *_logserver = nullptr;
  // theta(3), x1, xd(3), c_h, c_h dot, L, Ld, Ldd, tau_hat, u0, v3, xdd1,
  // xdd2, cy ddot, j_c, j_d, phase (0 lift, 1 hold), l_c, l_c dot, x3d.
  double _log[24] = {0};
  // MIT-law torque per joint (kp (a - q) + kd (adot - qd) + tau_ff), motor order.
  double _logTau[12] = {0};
  // WBC diagnostics (thesis Sec. 6.5, 6.6.2, 6.6.7), world axes, SI units:
  //  0      phase: 0 lift, 1 balance, 2 placement, 3 load transfer, 4 final
  //         four-foot hold (SETTLED once it has lasted 3 s)
  //  1      qpOASES status of the WBIC QP (0 = solved)
  //  2-7    desired contact force F_c^d handed to the WBIC on FL, RR (MPC,
  //         tangentially limited; in transfer the ramp blend of 3 and 4)
  //  8-13   solved contact force on FL, RR
  //  14-20  FR: measured xi_n, rho, xi_t; desired xi_n, rho, xi_t; AMB
  //         feedforward status (0 applied, 1 norm-limited, 2 rejected or off,
  //         3 promoted to a contact: no swing task, desired values not used)
  //  21-27  RL, same layout
  //  28-30  body position error (desired - measured), base origin
  //  31-33  body orientation error, rotation vector of R_d R^T (R_d: R_ref
  //         with the pitch/yaw sinusoids, see _oriReference)
  //  34     smallest distance of any FR or RL joint to its limit [rad]
  //  35-37  base velocity from the planted stance feet
  //  38-43  desired contact force on FR, RL (zero while not contacts)
  //  44-49  solved contact force on FR, RL (zero while not contacts)
  //  50-51  height of the FR, RL contact point above the ground plane
  //         (horizontal, through the FL-RR support line), from kinematics
  double _logWbc[52] = {0};
};

#endif
