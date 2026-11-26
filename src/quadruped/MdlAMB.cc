/*
 * Copyright (C) 2005-2026 Uluç Saranlı. All Rights Reserved.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include "rtcore/ModuleManager.hh"
#include "rtcore/ConfigTable.hh"
#include "quadruped/MdlAMB.hh"
#include "quadruped/MdlConvexMPC.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedDynamics.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "quadruped/QuadrupedLegDynamics.hh"
#include "quadruped/QuadrupedWBC.hh"
#include "quadruped/TrajectoryUtils.hh"
#include "hardware/MotorHW.hh"
#include "hardware/IMUHW.hh"

namespace {

// Missing or short arrays keep the compiled-in default rather than zeroing a
// template parameter, which would make the law singular.
void readVec3(const rtcore::ConfigTable& config, const char* key, Eigen::Vector3d& v) {
  rtcore::ConfigArray arr;
  if (!config.getArray(key, arr) || arr.size() < 3) return;
  for (int i = 0; i < 3; i++) v(i) = arr.getDoubleAt(i, v(i));
}

const double GRAVITY = 9.81;
const double DEG = M_PI / 180.0;
// Trunk inertial from models/go2/go2.xml (<body name="base"> <inertial ...>):
// QuadrupedLegDynamics only carries the twelve leg links, so the whole-body and
// support-set CoMs need the trunk added here.
const double BASE_MASS = 6.921;
const Eigen::Vector3d BASE_COM(0.021112, 0.0, -0.005366);
// Trunk inertia about its CoM in body axes (go2.xml inertial quat + diaginertia).
const Eigen::Matrix3d BASE_INERTIA = [] {
  const Eigen::Matrix3d Rq =
      Eigen::Quaterniond(-0.000543471, 0.713435, -0.00173769, 0.700719).toRotationMatrix();
  return Eigen::Matrix3d(Rq * Eigen::Vector3d(0.107027, 0.0980771, 0.0244531).asDiagonal() *
                         Rq.transpose());
}();
// Foot sphere radius (go2.xml foot geom). The support line runs through the
// ground contact points, not the sphere centres: the ground reaction then has no
// moment about the line and L_dot = M g c_h holds exactly (thesis Sec. 6.3.1).
const double FOOT_RADIUS = 0.022;
// Projected swing radius below which e_r is not defined (thesis Sec. 6.3.2).
// The Go2 swing legs work at 0.2-0.3 m; at 1 cm a millimetre of foot error
// already turns e_r by 6 degrees.
const double RHO_MIN = 0.01;
// Orientation sinusoids of thesis Sec. 6.6.5.1: 0.1 Hz, after a 2 s delay and
// a 2 s envelope ramp. Fixed by the protocol; only the amplitudes are keys.
const double ORI_FREQ = 0.1;
const double ORI_DELAY = 2.0;
const double ORI_RAMP = 2.0;
// Stop sequence of thesis Sec. 6.6.7: the 3 s load transfer of Table 6.3, and
// a final four-foot hold as long again, the "Final" interval of Table 6.6.
const double TRANSFER_DURATION = 3.0;
const double FINAL_HOLD = 3.0;
// Touchdown without a contact sensor. The placement aims the contact point
// PLACE_DEPTH below the ground plane, so a foot meets the ground before the
// ramp ends however the reference lags. A foot counts as down once its
// kinematic contact point is within TOUCHDOWN_HEIGHT of that plane and sinks
// no faster than TOUCHDOWN_SPEED: the ground has stopped it (it descends at
// about 6 cm/s before). Height alone cannot time it: a touching foot reads
// +0.6 to +1.4 mm in the full stack (the loaded stance feet sink further into
// the soft ground than a just-touching one), and promoting at 2.5 or 4 mm
// left the feet hovering for 0.12-0.2 s, the QP asking them for up to 2 N the
// ground was not giving. Stopped by the ground, they are promoted a few tens
// of ms after first contact and already bear load.
const double PLACE_DEPTH = 0.01;
const double TOUCHDOWN_HEIGHT = 0.005;
const double TOUCHDOWN_SPEED = 0.01;

// Offset [rad] and its rate of an amplitude-a sinusoid at t [s] since
// activation. The quintic envelope starts it with zero rate and acceleration,
// so the reference does not kick the torso at either end of the ramp.
void oriSinusoid(double a, double t, double& x, double& xd) {
  double e, ed;
  TrajectoryUtils::sampleQuintic((t - ORI_DELAY) / ORI_RAMP, ORI_RAMP, e, ed);
  const double w = 2.0 * M_PI * ORI_FREQ, ph = w * (t - ORI_DELAY);
  x = a * e * std::sin(ph);
  xd = a * (ed * std::sin(ph) + e * w * std::cos(ph));
}

// Template plane directions: y is up, so u(0) points straight up (thesis Fig. 5.2)
// and du/da is the bar direction e_perp at a = theta1.
Eigen::Vector2d dirUp(double a) { return Eigen::Vector2d(std::sin(a), std::cos(a)); }
Eigen::Vector2d dirTan(double a) { return Eigen::Vector2d(std::cos(a), -std::sin(a)); }

// In-plane angle of w from t_v toward t_h (thesis 5.46, 5.49), written so the
// normalisation of t_v cancels between the two atan2 arguments.
double planeAngle(const Eigen::Vector3d& s, const Eigen::Vector3d& w) {
  return std::atan2(s(1) * w(0) - s(0) * w(1), w(2) - w.dot(s) * s(2));
}

// Python's np.mod takes the divisor's sign, std::fmod the dividend's.
double mod2pi(double a) { return a - 2.0 * M_PI * std::floor(a / (2.0 * M_PI)); }
double wrapPi(double a) { return a - 2.0 * M_PI * std::floor((a + M_PI) / (2.0 * M_PI)); }

// Plan steps 1-2 at one configuration; project() differences it for the rates.
bool geometry(const QuadrupedLegDynamics& dyn, const Eigen::Vector3d q[4],
              const Eigen::Matrix3d& R, MdlAMB::proj_t& out) {
  const QuadrupedLegDynamics::params_t& prm = dyn.getParams();
  Eigen::Vector3d foot[4];
  Eigen::Vector3d c_B = BASE_MASS * BASE_COM, b_B = c_B;
  double m_all = BASE_MASS, m_sup = BASE_MASS;
  for (int leg = 0; leg < 4; leg++) {
    QuadrupedLegDynamics::kinematics_t kin;
    if (!dyn.linkKinematics(leg, q[leg], kin)) return false;
    foot[leg] = kin.foot_position;
    for (int j = 0; j < 3; j++) {
      const double m = prm.leg[leg].joint[j].link.mass;
      c_B += m * kin.com_position[j];
      m_all += m;
      // theta1 follows the CoM of {trunk, FL, RR} (the template's link 1), not
      // the whole body: the robot balances at theta1 = 0 whatever the swing legs
      // do, while the template's CoM moves with x2 (estimator.py body_point).
      if (leg == 0 || leg == 3) {
        b_B += m * kin.com_position[j];
        m_sup += m;
      }
    }
  }

  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d p_a = R * foot[0] - FOOT_RADIUS * ez;
  const Eigen::Vector3d p_b = R * foot[3] - FOOT_RADIUS * ez;
  const Eigen::Vector3d u = p_b - p_a;
  const double len = u.norm();
  out.s = u / len;
  out.o = 0.5 * (p_a + p_b);
  const Eigen::Vector3d e = ez - ez.dot(out.s) * out.s;
  out.tv = e / e.norm();
  out.th = out.s.cross(out.tv);

  // Swing anchors are the abduction joints: the hip roll then rotates f - h
  // rigidly in the plane, which is what makes j_c, j_d unit-sized.
  const double th1 = planeAngle(out.s, R * (b_B / m_sup) - out.o);
  const double a_c = planeAngle(out.s, R * (foot[1] - prm.leg[1].joint[0].parent_translation));
  const double a_d = planeAngle(out.s, R * (foot[2] - prm.leg[2].joint[0].parent_translation));
  // A leg hanging straight down sits on the atan2 cut, so the plain (-pi, pi]
  // wrap would jump there; theta2 in (-2pi, 0] and theta3 in (0, 2pi] put
  // legs-down at (-pi, +pi), i.e. x3 = -2pi (projection.py wrap_branches).
  double th3 = mod2pi(a_d - th1);
  if (th3 == 0.0) th3 = 2.0 * M_PI;
  out.theta = Eigen::Vector3d(th1, -mod2pi(-(a_c - th1)), th3);

  const Eigen::Vector3d r = R * (c_B / m_all) - out.o;
  out.c_h = out.th.dot(r);
  out.c_v = out.tv.dot(r);
  out.l_c = std::hypot(out.c_h, out.c_v);
  out.lever_a = 0.5 * len + r.dot(out.s);
  out.lever_b = 0.5 * len - r.dot(out.s);
  out.mass = m_all;
  return true;
}

// Angular momentum of the whole robot about the support line (o, s), as the
// rigid-body sum over the trunk and the twelve leg links of the same model
// geometry() uses, with the base velocity taken from the planted stance feet.
// The template's own momentum row is not a usable proxy on this robot: its
// swing coupling Mt(0,1) = +0.0078 has the wrong sign and ~8x the magnitude of
// the measured one (-0.001 at the nominal posture, sign-changing with the
// swing), so Mt(0,:) xd fed into k0 and the observer grows a 5 Hz swing until
// the rolls hit their range (reproduced in the planar Python with
// L_source = "template"; its default is the robot's L, plant.py).
double robotMomentum(const QuadrupedLegDynamics& dyn, const Eigen::Vector3d q[4],
                     const Eigen::Vector3d qd[4], const Eigen::Matrix3d& R,
                     const Eigen::Vector3d& w, const Eigen::Vector3d& o,
                     const Eigen::Vector3d& s) {
  const QuadrupedLegDynamics::params_t& prm = dyn.getParams();
  // project() has already run geometry() on this q, so the chain cannot fail here.
  QuadrupedLegDynamics::kinematics_t kin[4];
  for (int leg = 0; leg < 4; leg++) dyn.linkKinematics(leg, q[leg], kin[leg]);
  // The stance feet roll without slipping: their ground CONTACT points are at
  // rest while each sphere centre moves at r (omega_foot x e_z), omega_foot
  // being the calf's rate. The base origin's velocity follows, averaged over
  // FL and RR. Taking the centres as fixed instead undercounts a pure roll by
  // M r c_v omega (~7% of Mt(0,0) omega), which the observer would then read
  // as an external torque, and it would contradict geometry(), whose contact
  // points sit under the centres (d theta1 / d roll = 1 - r / l_b).
  Eigen::Vector3d v_base = Eigen::Vector3d::Zero();
  for (int leg : {0, 3}) {
    const Eigen::Vector3d w_foot = w + kin[leg].com_angular_jacobian[2] * qd[leg];
    v_base += 0.5 * (FOOT_RADIUS * (R * w_foot).cross(Eigen::Vector3d::UnitZ()) -
                     R * (w.cross(kin[leg].foot_position) + kin[leg].foot_jacobian * qd[leg]));
  }
  // Positions are relative to the base origin (as o is), velocities absolute.
  Eigen::Vector3d H = BASE_MASS * (R * BASE_COM - o).cross(v_base + R * w.cross(BASE_COM)) +
                      R * (BASE_INERTIA * w);
  for (int leg = 0; leg < 4; leg++) {
    for (int i = 0; i < 3; i++) {
      const QuadrupedLegDynamics::link_t& link = prm.leg[leg].joint[i].link;
      const Eigen::Vector3d& c = kin[leg].com_position[i];
      const Eigen::Matrix3d& Rl = kin[leg].link_rotation[i];
      const Eigen::Vector3d v_com = w.cross(c) + kin[leg].com_linear_jacobian[i] * qd[leg];
      const Eigen::Vector3d w_link = w + kin[leg].com_angular_jacobian[i] * qd[leg];
      H += link.mass * (R * c - o).cross(v_base + R * v_com) +
           R * (Rl * link.inertia_com * Rl.transpose() * w_link);
    }
  }
  return s.dot(H);
}

}  // namespace

MdlAMB::MdlAMB() : Module(AMBMODULE_NAME, 0, SINGLE_USER) {}
MdlAMB::~MdlAMB() {}

void MdlAMB::init() {
  // The balance law needs the link inertials, which only the Go2 model carries.
  _dyn = new QuadrupedLegDynamics(createGo2DynamicsConfig());
  _kin = new QuadrupedKinematics(createGo2Config());
  // Built whatever [amb] wbc says: a bare instance only gets its params after
  // init(). QuadrupedWBC's defaults are Table 6.2's (floating-base weight 1000).
  _wb = new QuadrupedDynamics(createGo2WholeBodyConfig());
  _wbc = new QuadrupedWBC();
  // A bare instance (test_amb_sim) has no manager: no legs to find, no [amb]
  // table and no log registration; it keeps the built-in defaults and drives
  // computeTick() itself (MdlLegControl::init follows the same rule).
  if (!_mgr) return;
  if (!_dyn->isValid())
    _mgr->warning(AMBMODULE_NAME, "Go2 leg dynamics parameters are invalid");

  for (int i = 0; i < 4; i++)
    _legs[i] = (MdlLegControl *)_mgr->findModule(LEGMODULE_NAME, i);

  rtcore::ConfigTable config;
  if (_mgr->getConfigTable("amb", config)) {
    _p.enabled = config.getBool("enabled", _p.enabled);
    readVec3(config, "template_m", _p.template_m);
    readVec3(config, "template_l", _p.template_l);
    _p.template_d = config.getDouble("template_d", _p.template_d);
    _p.x3_ref_deg = config.getDouble("x3_ref_deg", _p.x3_ref_deg);
    readVec3(config, "gains", _p.gains);
    _p.kp3 = config.getDouble("kp3", _p.kp3);
    _p.kd3 = config.getDouble("kd3", _p.kd3);
    _p.observer_tc = config.getDouble("observer_tc", _p.observer_tc);
    _p.kp_stance = config.getDouble("kp_stance", _p.kp_stance);
    _p.kd_stance = config.getDouble("kd_stance", _p.kd_stance);
    _p.kp_swing = config.getDouble("kp_swing", _p.kp_swing);
    _p.kd_swing = config.getDouble("kd_swing", _p.kd_swing);
    _p.kp_height = config.getDouble("kp_height", _p.kp_height);
    _p.kd_height = config.getDouble("kd_height", _p.kd_height);
    _p.qdd_max = config.getDouble("qdd_max", _p.qdd_max);
    _p.barrier_margin_deg = config.getDouble("barrier_margin_deg", _p.barrier_margin_deg);
    _p.barrier_kp = config.getDouble("barrier_kp", _p.barrier_kp);
    _p.barrier_kd = config.getDouble("barrier_kd", _p.barrier_kd);
    _p.joint_damping = config.getDouble("joint_damping", _p.joint_damping);
    _p.joint_friction = config.getDouble("joint_friction", _p.joint_friction);
    _p.lift_duration = config.getDouble("lift_duration", _p.lift_duration);
    _p.stance_x = config.getDouble("stance_x", _p.stance_x);
    _p.base_z = config.getDouble("base_z", _p.base_z);
    readVec3(config, "swing_rel", _p.swing_rel);
    _p.fd_step = config.getDouble("fd_step", _p.fd_step);
    _p.theta1_max_deg = config.getDouble("theta1_max_deg", _p.theta1_max_deg);
    _p.z_min = config.getDouble("z_min", _p.z_min);
    _p.wbc = config.getBool("wbc", _p.wbc);
    _p.kp_body_ori = config.getDouble("kp_body_ori", _p.kp_body_ori);
    _p.kd_body_ori = config.getDouble("kd_body_ori", _p.kd_body_ori);
    _p.kp_body_pos = config.getDouble("kp_body_pos", _p.kp_body_pos);
    _p.kd_body_pos = config.getDouble("kd_body_pos", _p.kd_body_pos);
    _p.kp_foot = config.getDouble("kp_foot", _p.kp_foot);
    _p.kd_foot = config.getDouble("kd_foot", _p.kd_foot);
    _p.kp_joint = config.getDouble("kp_joint", _p.kp_joint);
    _p.kd_joint = config.getDouble("kd_joint", _p.kd_joint);
    _p.foot_acc_max = config.getDouble("foot_acc_max", _p.foot_acc_max);
    _p.force_ratio = config.getDouble("force_ratio", _p.force_ratio);
    rtcore::ConfigArray sched;
    if (config.getArray("x3_schedule", sched)) {
      for (int i = 0; i < sched.size(); i++) {
        // A dropped or zeroed step would silently change the experiment.
        rtcore::ConfigArray e;
        const bool ok = sched.getArrayAt(i, e) && e.size() == 2;
        const Eigen::Vector2d v(e.getDoubleAt(0, NAN), e.getDoubleAt(1, NAN));
        if (!ok || !v.allFinite())
          _mgr->fatalError(AMBMODULE_NAME, "[amb] x3_schedule entry %d is not a [time, x3_deg] pair", i);
        _p.x3_schedule.push_back(v);
      }
    }
    _p.pitch_amp_deg = config.getDouble("pitch_amp_deg", _p.pitch_amp_deg);
    _p.yaw_amp_deg = config.getDouble("yaw_amp_deg", _p.yaw_amp_deg);
    _p.place_duration = config.getDouble("place_duration", _p.place_duration);
    if (config.getString("attitude_source", "imu") == "estimator") {
      _orientation =
          (MdlOrientationEstimator *)_mgr->findModule(ORIENTATIONMODULE_NAME, 0);
      if (!_orientation)
        _mgr->warning(AMBMODULE_NAME, "attitude_source = \"estimator\" but %s not found; using the IMU",
                      ORIENTATIONMODULE_NAME);
    }
  } else {
    _mgr->warning(AMBMODULE_NAME, "No [amb] config table; using built-in defaults");
  }
  // The stance-force planner is grabbed in activate(), as MdlTrot does.
  if (_p.wbc) {
    _mpc = (MdlConvexMPC *)_mgr->findModule(MPCMODULE_NAME, 0);
    if (!_mpc)
      _mgr->fatalError(AMBMODULE_NAME, "[amb] wbc = true needs %s, created before %s",
                       MPCMODULE_NAME, AMBMODULE_NAME);
  }

  _logserver = (rtcore::LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
  if (_logserver) {
    _logserver->registerVar(rtcore::LOG_DOUBLE, 24, AMBMODULE_NAME, "state",
                            (unsigned char*)_log);
    _logserver->registerVar(rtcore::LOG_DOUBLE, 12, AMBMODULE_NAME, "tau",
                            (unsigned char*)_logTau);
    _logserver->registerVar(rtcore::LOG_DOUBLE, 52, AMBMODULE_NAME, "wbc",
                            (unsigned char*)_logWbc);
  }
}

void MdlAMB::uninit() {
  if (_logserver) {
    _logserver->deleteVar(AMBMODULE_NAME, "state");
    _logserver->deleteVar(AMBMODULE_NAME, "tau");
    _logserver->deleteVar(AMBMODULE_NAME, "wbc");
    _logserver = nullptr;
  }
  delete _dyn;
  _dyn = nullptr;
  delete _kin;
  _kin = nullptr;
  delete _wb;
  _wb = nullptr;
  delete _wbc;
  _wbc = nullptr;
}

void MdlAMB::activate() {
  _status = IDLE;
  // The lift start, the latched height, x3(t0) and the swing feet are taken by
  // the first computeTick() from the very state it then commands, so the
  // hand-off tick cannot see a snapshot/command mismatch.
  _first = true;
  _stopReq = false;
  _tPlace = _tTransfer = -1.0;
  _promoted[0] = _promoted[1] = false;
  _tau_hat = 0.0;
  _maxTheta1 = 0.0;
  _ticks = _satTicks = 0;
  _tickSum = _tickMax = 0.0;
  _tickCount = 0;
  // A sample left over from an earlier activation must not pass the first
  // tick's validity check when the IMU has since stopped.
  _imu.t = -1.0;
  _imuMiss = 0;

  // Grab all legs first. If any motor is not ready, release all legs before
  // returning ERROR. WBC mode also owns the MPC, whose warm start belongs to
  // one behavior at a time (MdlTrot does the same).
  for (int i = 0; i < 4; i++)
    _mgr->grabModule(_legs[i], this);
  if (_p.wbc) _mgr->grabModule(_mpc, this);

  MotorHW *hw = MotorHW::instance();
  for (int idx = 0; idx < 12; idx++) {
    if (hw->getStatus(idx) != MotorHW::STATUS_READY) {
      _mgr->warning(AMBMODULE_NAME, "Motor %d not ready at activation", idx);
      for (int k = 0; k < 4; k++)
        _mgr->releaseModule(_legs[k], this);
      if (_p.wbc) _mgr->releaseModule(_mpc, this);
      _status = ERROR;
      return;
    }
  }

  _status = ACTIVE;
}

void MdlAMB::deactivate() {
  if (_ticks > 0)
    _mgr->message("MdlAMB: max|theta1| after the lift %.2f deg, torque saturated on %.1f%% of ticks",
                  _maxTheta1 / DEG, 100.0 * _satTicks / _ticks);
  if (_p.wbc && _tickCount > 0)
    _mgr->message("MdlAMB: computeTick mean %.1f us, max %.1f us over %d ticks (control period %.1f ms)",
                  1e6 * _tickSum / _tickCount, 1e6 * _tickMax, _tickCount,
                  1e3 * CLOCK_TO_SEC(_mgr->getStepPeriod()));
  for (int i = 0; i < 4; i++)
    _mgr->releaseModule(_legs[i], this);
  if (_p.wbc) _mgr->releaseModule(_mpc, this);
}

bool MdlAMB::stopBalancing() {
  if (!_p.wbc) return false;
  _stopReq = true;
  return true;
}

bool MdlAMB::project(const params_t& p, const input_t& in, const QuadrupedLegDynamics& dyn,
                     proj_t& out) {
  if (!geometry(dyn, in.q, in.R, out)) return false;

  // Rates as the wrapped difference of the geometry advanced by fd_step along
  // (qd, w_body) (thesis Sec. 5.5.5): one more geometry() call instead of an
  // analytic Jacobian of the whole support-frame construction.
  const double h = p.fd_step;
  Eigen::Vector3d q1[4];
  for (int i = 0; i < 4; i++) q1[i] = in.q[i] + h * in.qd[i];
  Eigen::Matrix3d R1 = in.R;
  const double wn = in.w_body.norm();
  if (wn > 0.0) R1 = in.R * Eigen::AngleAxisd(wn * h, in.w_body / wn).toRotationMatrix();
  proj_t adv;
  if (!geometry(dyn, q1, R1, adv)) return false;
  // theta2 and theta3 live on branches 2pi wide, so a step across a cut must
  // not read as a 2pi/h rate.
  for (int i = 0; i < 3; i++) out.thetad(i) = wrapPi(adv.theta(i) - out.theta(i)) / h;
  out.ch_dot = (adv.c_h - out.c_h) / h;
  out.lc_dot = (adv.l_c - out.l_c) / h;

  // FR roll drives theta2, RL roll drives theta3. Measured at runtime so a
  // wrong abduction sign convention fails loudly (|j| far from 1) instead of
  // silently inverting the balance loop.
  for (int k = 0; k < 2; k++) {
    const int leg = (k == 0) ? 1 : 2;
    for (int i = 0; i < 4; i++) q1[i] = in.q[i];
    q1[leg](0) += h;
    proj_t pert;
    if (!geometry(dyn, q1, in.R, pert)) return false;
    const double j = wrapPi(pert.theta(1 + k) - out.theta(1 + k)) / h;
    if (std::abs(j) < 0.5) return false;
    (k == 0 ? out.j_c : out.j_d) = j;
  }
  return true;
}

void MdlAMB::templateKin(const params_t& p, const Eigen::Vector3d& x,
                         const Eigen::Vector3d& xd, kin_t& k) {
  // theta = T x (thesis 2.7 / 5.54): x2 is the symmetric swing, x3 the separation.
  Eigen::Matrix3d T;
  T << 1, 0, 0, 0, 0.5, 0.5, 0, 0.5, -0.5;
  const Eigen::Vector3d th = T * x, thd = T * xd;
  const double l1 = p.template_l(0), l2 = p.template_l(1), l3 = p.template_l(2);
  const double d = p.template_d;

  // Cross-bar point masses (thesis 5.1-5.6, errata E3/E4): body at l1 u(theta1),
  // appendage 2 at the LEFT bar end p1 - (d/2) e_perp, appendage 3 at the right.
  const Eigen::Vector2d u1 = dirUp(th(0)), e1 = dirTan(th(0));
  const Eigen::Vector2d u2 = dirUp(th(0) + th(1)), e2 = dirTan(th(0) + th(1));
  const Eigen::Vector2d u3 = dirUp(th(0) + th(2)), e3 = dirTan(th(0) + th(2));
  Eigen::Vector2d pt[3], a[3];
  Eigen::Matrix<double, 2, 3> J[3];
  pt[0] = l1 * u1;
  pt[1] = pt[0] - 0.5 * d * e1 + l2 * u2;
  pt[2] = pt[0] + 0.5 * d * e1 + l3 * u3;
  // d e_perp / d theta1 = -u1, which is why the bar half-width enters the
  // theta1 column with the opposite sign to its position term.
  J[0].setZero();
  J[0].col(0) = l1 * e1;
  J[1].col(0) = l1 * e1 + 0.5 * d * u1 + l2 * e2;
  J[1].col(1) = l2 * e2;
  J[1].col(2).setZero();
  J[2].col(0) = l1 * e1 - 0.5 * d * u1 + l3 * e3;
  J[2].col(1).setZero();
  J[2].col(2) = l3 * e3;
  // Bias accelerations Jdot thetadot (u'' = -u): the Coriolis vector of the
  // Lagrangian in crossbar.py reduces to sum m_k J_k^T a_k for point masses.
  const double w1 = thd(0), w2 = thd(0) + thd(1), w3 = thd(0) + thd(2);
  a[0] = -l1 * u1 * w1 * w1;
  a[1] = a[0] + 0.5 * d * e1 * w1 * w1 - l2 * u2 * w2 * w2;
  a[2] = a[0] - 0.5 * d * e1 * w1 * w1 - l3 * u3 * w3 * w3;

  Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
  Eigen::Vector3d Cth = Eigen::Vector3d::Zero(), G = Eigen::Vector3d::Zero();
  Eigen::RowVector3d dcx = Eigen::RowVector3d::Zero(), dcy = Eigen::RowVector3d::Zero();
  double cx = 0.0, cy = 0.0, bx = 0.0, by = 0.0, m_tot = 0.0;
  for (int i = 0; i < 3; i++) {
    const double m = p.template_m(i);
    M += m * J[i].transpose() * J[i];
    Cth += m * J[i].transpose() * a[i];
    G += GRAVITY * m * J[i].row(1).transpose();
    dcx += m * J[i].row(0);
    dcy += m * J[i].row(1);
    cx += m * pt[i](0);
    cy += m * pt[i](1);
    bx += m * a[i](0);
    by += m * a[i](1);
    m_tot += m;
  }
  k.Mt = T.transpose() * M * T;
  k.Ct = T.transpose() * Cth;
  k.Gt = T.transpose() * G;
  k.cx = cx / m_tot;
  k.cy = cy / m_tot;
  k.dcx = (dcx * T).transpose() / m_tot;
  k.dcy = (dcy * T).transpose() / m_tot;
  k.cx_bias = bx / m_tot;
  k.cy_bias = by / m_tot;
  k.m_tot = m_tot;
}

double MdlAMB::comConsistentX1(const params_t& p, const Eigen::Vector3d& x,
                               const Eigen::Vector3d& xd, double M_ch) {
  // With m_tot c_x = M c_h the template's L_dot = m g c_x is the robot's true
  // momentum rate and both share one equilibrium (matching.py com_consistent_x1).
  // c_x is monotonic in x1 near upright, so Newton from the geometric theta1
  // converges in a few steps.
  Eigen::Vector3d xc = x;
  kin_t k;
  for (int i = 0; i < 3; i++) {
    templateKin(p, xc, xd, k);
    xc(0) -= (k.m_tot * k.cx - M_ch) / (k.m_tot * k.dcx(0));
  }
  return xc(0);
}

bool MdlAMB::ambLaw(const params_t& p, const kin_t& k, const Eigen::Vector3d& x,
                    const Eigen::Vector3d& xd, double L, double Ld, double Ldd,
                    double tau_hat, double x3d, Eigen::Vector3d& xdd) {
  // The observed external torque acts on the passive row exactly like -Gt_1, so
  // it is folded into tau_1 and into L_dot together: the commanded accelerations
  // are then the ones the plant realises (controller.py line 111).
  const double tau1 = -(k.Ct(0) + k.Gt(0) - tau_hat);
  Ld += tau_hat;
  // AMB base reaction (thesis 4.19 with erratum E2): gains are positive magnitudes.
  const double u0 = -(p.gains(0) * L + p.gains(1) * Ld + p.gains(2) * Ldd) / GRAVITY;
  const double v3 = p.kp3 * (x3d - x(2)) - p.kd3 * xd(2);  // (4.27), constant reference

  // Momentum manifold (4.28) + (2.41) solved as the 2x2 system
  // [m11 m12; M01 M02] xdd12 = [tau1 - m13 v3; u0 - C0 - M03 v3] rather than
  // through gamma = m1k / m12: m12 changes sign inside the operating band while
  // this determinant stays bounded away from zero (momentum.py amb_manifold).
  const double m11 = k.Mt(0, 0), m12 = k.Mt(0, 1), m13 = k.Mt(0, 2);
  const Eigen::Vector3d M0b = k.m_tot * k.dcx;  // fictitious base row (4.10)-(4.14)
  const double C0 = k.m_tot * k.cx_bias;
  const double det = m11 * M0b(1) - m12 * M0b(0);
  if (std::abs(det) < 1e-9) return false;
  const double r1 = tau1, r2 = u0 - C0, s1 = -m13, s2 = -M0b(2);
  const double a1 = (M0b(1) * r1 - m12 * r2) / det, b1 = (M0b(1) * s1 - m12 * s2) / det;
  const double a2 = (m11 * r2 - M0b(0) * r1) / det, b2 = (m11 * s2 - M0b(0) * s1) / det;
  xdd = Eigen::Vector3d(a1 + b1 * v3, a2 + b2 * v3, v3);
  return true;
}

void MdlAMB::stanceForces(const Eigen::Vector3d& F, const proj_t& pr, Eigen::Vector3d f[2]) {
  // Closed form of the two-foot wrench distribution (thesis 5.35-5.36) once the
  // moment about the support line, which two point feet cannot realise, is
  // dropped: a lever rule on F plus the couple that F's along-line component
  // makes with the CoM offset r from the line.
  const Eigen::Vector3d r = pr.c_h * pr.th + pr.c_v * pr.tv;
  const double fs = F.dot(pr.s), span = pr.lever_a + pr.lever_b;
  f[0] = (pr.lever_b * F + fs * r) / span;  // FL
  f[1] = (pr.lever_a * F - fs * r) / span;  // RR
}

bool MdlAMB::principalFrame(const Eigen::Vector3d& e_n, const Eigen::Vector3d& d_f,
                            Eigen::Matrix3d& R_WP, double& rho) {
  const Eigen::Vector3d r_p = d_f - d_f.dot(e_n) * e_n;  // (6.13)
  if (r_p.norm() < RHO_MIN) return false;
  rho = r_p.norm();
  // (6.14): with e_n = s and e_r = dirUp(alpha) in the (t_v, t_h) plane basis,
  // e_t = de_r / dalpha, so a positive rate along e_t is a positive rate of the
  // plane angle geometry() measures, i.e. of theta2 (FR) or theta3 (RL).
  R_WP.col(0) = e_n;
  R_WP.col(1) = r_p / rho;
  R_WP.col(2) = e_n.cross(R_WP.col(1));
  return true;
}

double MdlAMB::tangentialArc(const Eigen::Vector3d& e_n, const Eigen::Vector3d& e_r0,
                             double rho_d, const Eigen::Vector3d& d_f) {
  // e_r0 was stored perpendicular to the support line, which only moves by the
  // stance feet's rolling, so its reprojection never degenerates.
  const Eigen::Vector3d er = (e_r0 - e_r0.dot(e_n) * e_n).normalized();
  return rho_d * std::atan2(d_f.dot(e_n.cross(er)), d_f.dot(er));  // (6.16)
}

void MdlAMB::swingReference(const Eigen::Vector3d& e_n, const Eigen::Vector3d& e_r0,
                            double xi_n, double rho_d, double xi_t, Eigen::Vector3d& d_des,
                            Eigen::Vector3d& e_t_des) {
  // (6.20) divides by the radius that (6.16) multiplied with, so the two are
  // exact inverses also while the lift ramps rho_d away from rho0.
  const double alpha = xi_t / rho_d;
  const Eigen::Vector3d er = (e_r0 - e_r0.dot(e_n) * e_n).normalized();
  const Eigen::Vector3d et = e_n.cross(er);
  const Eigen::Vector3d e_r_des = std::cos(alpha) * er + std::sin(alpha) * et;  // (6.21)
  e_t_des = -std::sin(alpha) * er + std::cos(alpha) * et;                       // (6.22)
  d_des = xi_n * e_n + rho_d * e_r_des;                                          // (6.24)
}

bool MdlAMB::tangentialFeedforward(double rho, double thdd, const Eigen::Vector3d& e_t,
                                   double a_max, Eigen::Vector3d& a_f) {
  a_f = rho * thdd * e_t;  // (6.18), along the current axis (6.26)
  const double n = a_f.norm();
  if (n <= a_max) return false;
  a_f *= a_max / n;  // Sec. 6.4.3: bounded before it reaches the WBC
  return true;
}

bool MdlAMB::computeTick(const input_t& in, cmd_t out[4]) {
  proj_t pr;
  if (!project(_p, in, *_dyn, pr)) return false;
  const QuadrupedKinematics::params_t& geo = _kin->getKinematicParams();

  if (_first) {
    _t0 = in.t;
    // Height is regulated about where the robot stands at hand-off, not base_z:
    // MdlDiagonalStance leaves it a little lower under load, and pulling it up
    // during the lift would unload a stance foot.
    _l_nom = pr.l_c;
    // The lift geometry itself moves x3 from the four-feet value (~ -322 deg)
    // to the nominal -304 deg; a reference ramped from where x3 starts does
    // not kick the swing rolls at lift-off.
    _x3d0 = pr.theta(1) - pr.theta(2);
    for (int leg = 0; leg < 4; leg++) {
      const double fore = (leg < 2) ? 1.0 : -1.0, left = (leg % 2 == 0) ? 1.0 : -1.0;
      if (leg == 1 || leg == 2) {
        const int k = (leg == 1) ? 0 : 1;
        if (!_dyn->footPosition(leg, in.q[leg], _p_sw0[k])) return false;
        // swing_rel is fore / outward / down from the abduction anchor, mirrored
        // per leg (posture.py Posture.swing_rel).
        _p_sw1[k] = geo.hip_positions.row(leg).transpose() +
                    Eigen::Vector3d(fore * _p.swing_rel(0), left * _p.swing_rel(1), -_p.swing_rel(2));
      } else {
        // The posture MdlDiagonalStance hands over: stance feet on the body
        // centreline, foot centre one radius above the ground at base_z.
        const Eigen::Vector3d p(fore * _p.stance_x, 0.0, -(_p.base_z - FOOT_RADIUS));
        if (!_kin->inverseKinematics(leg, p, _q_nom[leg])) return false;
      }
    }
  }
  const double elapsed = in.t - _t0;
  // Minimum-jerk profile: zero rate and acceleration at both ends, so neither
  // the swing feet nor the x3 reference kick the rolls at lift-off or at the
  // end of the lift.
  double sigma, sigma_dot, sigma_ddot;
  TrajectoryUtils::sampleQuintic(elapsed / _p.lift_duration, _p.lift_duration, sigma, sigma_dot,
                                 sigma_ddot);
  const bool lifting = elapsed < _p.lift_duration;

  // Template coordinates (thesis 5.53): x2 the symmetric swing, x3 the split.
  Eigen::Vector3d x(pr.theta(0), pr.theta(1) + pr.theta(2), pr.theta(1) - pr.theta(2));
  Eigen::Vector3d xd(pr.thetad(0), pr.thetad(1) + pr.thetad(2), pr.thetad(1) - pr.thetad(2));
  const double M = pr.mass;
  x(0) = comConsistentX1(_p, x, xd, M * pr.c_h);

  // The momentum triple is the robot's (plant.py L_source "robot", thesis
  // Ch. 6): L measured, L_dot = M g c_h exact about the contact line (Sec.
  // 6.3.1), and x1_dot replaced by the rate that makes the template carry the
  // measured L, so the law and the observer see one consistent momentum.
  kin_t k;
  templateKin(_p, x, xd, k);  // Mt depends on x only
  const double L = robotMomentum(*_dyn, in.q, in.qd, in.R, in.w_body, pr.o, pr.s);
  xd(0) = (L - k.Mt(0, 1) * xd(1) - k.Mt(0, 2) * xd(2)) / k.Mt(0, 0);
  templateKin(_p, x, xd, k);
  const double Ld = M * GRAVITY * pr.c_h;
  const double Ldd = M * GRAVITY * pr.ch_dot;
  // External-torque observer (runner.py MomentumObserver): first-order low-pass
  // of the momentum rate the model cannot explain, trapezoidal in c_h so a
  // one-sample lag does not read as a torque.
  if (!_first) {
    const double dt = in.t - _t_prev;
    if (dt > 0.0) {
      const double r = (L - _L_prev) / dt - 0.5 * M * GRAVITY * (pr.c_h + _ch_prev);
      _tau_hat += dt / (_p.observer_tc + dt) * (r - _tau_hat);
    }
  }
  _L_prev = L;
  _ch_prev = pr.c_h;
  _t_prev = in.t;

  // Sec. 6.6.3 schedule, after the lift: a raw step, as drawn in Fig. 6.4.
  // The law's x3 channel (4.27) treats its reference as constant and is
  // itself the filter: kp3, kd3 put two real poles at 2.7 and 37 rad/s, and
  // the largest thesis step (120 deg) asks the swing feet for at most about
  // 30 m/s^2 on its first tick (11 measured for 54 deg), within foot_acc_max.
  // A 0.5 s quintic instead cut that to 1 m/s^2 but settled 0.26 s later,
  // with the same theta1, L and steady error.
  double x3_ref = _p.x3_ref_deg;
  if (!lifting)
    for (const Eigen::Vector2d& e : _p.x3_schedule)
      if (elapsed >= e(0)) x3_ref = e(1);
  double x3d = _x3d0 + (x3_ref * DEG - _x3d0) * sigma;
  // Sec. 6.6.7 placement, from the first tick after a stop request once the
  // lift is over (a stop during the lift waits for it rather than reversing
  // it): the lift run backwards, so x3d returns from where it is to x3 at
  // activation along the same quintic, and the schedule no longer applies
  // (Fig. 6.3, "foot placement" ramp). The swing references follow in
  // _wbcTick on the same ramp.
  double place = 0.0, place_dot = 0.0, place_ddot = 0.0;
  if (_stopReq && !lifting) {
    if (_tPlace < 0.0) {
      _tPlace = elapsed;
      _x3dStop = x3d;
      if (_mgr) _mgr->message("MdlAMB: t=%.3f placement started", in.t);
    }
    TrajectoryUtils::sampleQuintic((elapsed - _tPlace) / _p.place_duration, _p.place_duration,
                                   place, place_dot, place_ddot);
    x3d = _x3dStop + (_x3d0 - _x3dStop) * place;
  }
  Eigen::Vector3d xdd;
  if (!ambLaw(_p, k, x, xd, L, Ld, Ldd, _tau_hat, x3d, xdd)) return false;
  // Log-only mirror of ambLaw's u0 (tau_hat folded into L_dot there too).
  const double u0 =
      -(_p.gains(0) * L + _p.gains(1) * (Ld + _tau_hat) + _p.gains(2) * Ldd) / GRAVITY;
  const double cyy = k.dcy.dot(xdd) + k.cy_bias;
  const double thdd[2] = {0.5 * (xdd(1) + xdd(2)), 0.5 * (xdd(1) - xdd(2))};

  const Eigen::Vector3d g_body = in.R.transpose() * Eigen::Vector3d(0.0, 0.0, -GRAVITY);
  const Eigen::Vector3d& tau_lim = _dyn->getParams().torque_limit;

  if (_p.wbc) {
    // Thesis Ch. 6: the same thdd handed to the whole-body controller as a
    // principal-frame swing-foot task (signal path 6.10).
    if (!_wbcTick(in, pr, thdd, sigma, sigma_dot, sigma_ddot, place, place_dot, place_ddot, out))
      return false;
  } else {
    // Stance: weight plus height regulation along t_v, nothing lateral. The
    // lateral and roll errors ARE the balance state; a stance PD on them would be
    // a second, unmodelled actuator fighting AMB (lowlevel.py "balance" mode).
    const Eigen::Vector3d F =
        M * GRAVITY * Eigen::Vector3d::UnitZ() +
        (_p.kp_height * (_l_nom - pr.l_c) - _p.kd_height * pr.lc_dot) * pr.tv;
    Eigen::Vector3d f[2];
    stanceForces(F, pr, f);
    for (int i = 0; i < 2; i++) {
      const int leg = (i == 0) ? 0 : 3;
      QuadrupedLegDynamics::terms_t terms;
      if (!_dyn->compute(leg, in.q[leg], in.qd[leg], g_body, terms)) return false;
      out[leg].a = _q_nom[leg];
      out[leg].adot.setZero();
      out[leg].kp.setConstant(_p.kp_stance);
      out[leg].kd.setConstant(_p.kd_stance);
      // f is the ground reaction ON the robot (world); the joints realise it with
      // -J^T f, since M qdd + h = tau + J^T f (lowlevel.py note on thesis 5.38).
      out[leg].tau = terms.bias - terms.foot_jacobian.transpose() * (in.R.transpose() * f[i]);
    }

    // Swing legs: computed torque on the roll, PD along the lift on thigh/knee.
    for (int i = 0; i < 2; i++) {
      const int leg = (i == 0) ? 1 : 2;
      QuadrupedLegDynamics::terms_t terms;
      if (!_dyn->compute(leg, in.q[leg], in.qd[leg], g_body, terms)) return false;
      const double q_r = in.q[leg](0), qd_r = in.qd[leg](0);
      const double qdd = std::max(-_p.qdd_max, std::min(_p.qdd_max, thdd[i] / ((i == 0) ? pr.j_c : pr.j_d)));
      // No PD on the roll (thesis 6.23/6.28): it would fight the inertial action
      // AMB asks for. The sim model's joint damping and dry friction are
      // cancelled explicitly; the friction sign is previewed 10 ms ahead and,
      // at rest, taken from the commanded acceleration, otherwise small
      // commands stick (lifting.py, rung d).
      const double sv = std::tanh((qd_r + 0.01 * qdd) / 0.02);
      double tau_r = terms.mass_matrix(0, 0) * qdd + terms.bias(0) + _p.joint_damping * qd_r +
                     _p.joint_friction * (sv + (1.0 - std::abs(sv)) * std::tanh(qdd / 0.1));
      // One-sided soft barrier just inside the hip roll range.
      const double lo = geo.hip_abduction_limits(leg, 0), hi = geo.hip_abduction_limits(leg, 1);
      const double margin = _p.barrier_margin_deg * DEG;
      if (q_r > hi - margin)
        tau_r += -_p.barrier_kp * (q_r - (hi - margin)) - _p.barrier_kd * std::max(qd_r, 0.0);
      else if (q_r < lo + margin)
        tau_r += -_p.barrier_kp * (q_r - (lo + margin)) - _p.barrier_kd * std::min(qd_r, 0.0);
      tau_r = std::max(-tau_lim(0), std::min(tau_lim(0), tau_r));

      const Eigen::Vector3d p_sw = _p_sw0[i] + (_p_sw1[i] - _p_sw0[i]) * sigma;
      Eigen::Vector3d q_t;
      if (!_kin->inverseKinematics(leg, p_sw, q_t)) return false;
      out[leg].adot.setZero();
      if (_p.enabled) {
        // The roll target is the measured angle, clamped: kp is zero so it is
        // inert, but setJointCommand rejects a target outside the range.
        out[leg].a = Eigen::Vector3d(std::max(lo, std::min(hi, q_r)), q_t(1), q_t(2));
        out[leg].kp = Eigen::Vector3d(0.0, _p.kp_swing, _p.kp_swing);
        // The roll kd must stay positive: MdlSimDriver.cc:781 rewrites any
        // kd <= 0 to 5.0 inside the stored command, permanently, which would put
        // a 5 Nm s/rad brake on the joint AMB is trying to accelerate.
        out[leg].kd = Eigen::Vector3d(1e-3, _p.kd_swing, _p.kd_swing);
        out[leg].tau = Eigen::Vector3d(tau_r, terms.bias(1), terms.bias(2));
      } else {
        // AMB-off ablation: the roll is PD-held on the lift target, same lift.
        out[leg].a = q_t;
        out[leg].kp.setConstant(_p.kp_swing);
        out[leg].kd.setConstant(_p.kd_swing);
        out[leg].tau = terms.bias;
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    _log[i] = pr.theta(i);
    _log[4 + i] = xd(i);
  }
  _log[3] = x(0);
  _log[7] = pr.c_h;
  _log[8] = pr.ch_dot;
  _log[9] = L;
  _log[10] = Ld;
  _log[11] = Ldd;
  _log[12] = _tau_hat;
  _log[13] = u0;
  _log[14] = xdd(2);
  _log[15] = xdd(0);
  _log[16] = xdd(1);
  _log[17] = cyy;
  _log[18] = pr.j_c;
  _log[19] = pr.j_d;
  _log[20] = lifting ? 0.0 : 1.0;
  _log[21] = pr.l_c;
  _log[22] = pr.lc_dot;
  _log[23] = x3d;
  bool saturated = false;
  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      const double tau = out[leg].kp(j) * (out[leg].a(j) - in.q[leg](j)) +
                         out[leg].kd(j) * (out[leg].adot(j) - in.qd[leg](j)) + out[leg].tau(j);
      _logTau[3 * leg + j] = tau;
      if (std::abs(tau) > tau_lim(j)) saturated = true;
    }
  }
  _ticks++;
  if (saturated) _satTicks++;
  if (!lifting) _maxTheta1 = std::max(_maxTheta1, std::abs(pr.theta(0)));
  _first = false;

  // NaN passes MdlLegControl's joint-limit check (every comparison is false),
  // so a non-finite command has to be refused here, before it reaches a motor.
  for (int leg = 0; leg < 4; leg++)
    if (!out[leg].a.allFinite() || !out[leg].tau.allFinite()) return false;
  return std::abs(pr.theta(0)) <= _p.theta1_max_deg * DEG && pr.l_c >= _p.z_min;
}

bool MdlAMB::_wbcTick(const input_t& in, const proj_t& pr, const double thdd[2], double sigma,
                      double sigma_dot, double sigma_ddot, double place, double place_dot,
                      double place_ddot, cmd_t out[4]) {
  typedef QuadrupedDynamics::VectorV VectorV;
  const QuadrupedKinematics::params_t& geo = _kin->getKinematicParams();
  const Eigen::Vector3d& s = pr.s;  // e_n = e_s, ordered FL -> RR (6.11)

  QuadrupedDynamics::state_t st;
  st.R = in.R;
  st.w_body = in.w_body;
  for (int leg = 0; leg < 4; leg++) {
    st.q[leg] = in.q[leg];
    st.qd[leg] = in.qd[leg];
  }
  QuadrupedDynamics::terms_t dyn;
  if (!_wb->compute(st, dyn)) return false;
  // No base position or velocity estimate: compute() needs neither, the base
  // velocity follows from the planted stance feet (contact J v = 0, whose
  // base-linear block is the identity, averaged over FL and RR as
  // robotMomentum does), and positions are only needed from the support
  // midpoint, which geometry() already gives.
  const VectorV v0 = st.velocity();  // v_base still zero
  st.v_base = -0.5 * (dyn.contact[0].J * v0 + dyn.contact[3].J * v0);
  const VectorV v = st.velocity();
  const Eigen::Vector3d p_base = -pr.o;
  const Eigen::Vector3d w_world = in.R * in.w_body;

  if (_first) {
    // The measured attitude, not a levelled one: MdlDiagonalStance hands over
    // nearly level, and a levelled reference would add a standing error that
    // no contact force can remove about the support line (the thesis'
    // references in Fig. 6.5 are the non-zero hand-off angles too).
    _R_ref = in.R;
    _rpy_ref = MdlOrientationEstimator::rpyFromQuat(Eigen::Quaterniond(in.R));
    _p_ref = p_base;
    // Principal references (Sec. 6.4.2) from the hand-off geometry, and the
    // joint-space lift target swing_rel in the same plane: the lift ramps the
    // normal and radial references to it, while the tangential channel stays
    // with AMB, whose x3 ramp produces the separation. The target's angle
    // about e_r0 is only used when AMB is off (the ablation's same lift).
    for (int k = 0; k < 2; k++) {
      const int leg = (k == 0) ? 1 : 2;
      const Eigen::Vector3d d_f = dyn.foot[leg].p - dyn.hip[leg].p;
      Eigen::Matrix3d R_WP;
      if (!principalFrame(s, d_f, R_WP, _rho0[k])) return false;
      _e_r0[k] = R_WP.col(1);
      _xi_n0[k] = d_f.dot(s);
      const Eigen::Vector3d d_1 = in.R * _p_sw1[k] - dyn.hip[leg].p;
      _xi_n1[k] = d_1.dot(s);
      _rho1[k] = (d_1 - _xi_n1[k] * s).norm();
      _alpha1[k] = tangentialArc(s, _e_r0[k], 1.0, d_1);  // (6.16) at unit radius
    }
    _tMpc = in.t;
  }

  // Sec. 6.6.7 contact promotion. There is no contact sensor, but the ground
  // is the horizontal plane through the support line, so a swing foot's
  // height above it is kinematic. A foot that the ground has stopped there
  // becomes a contact of the WBC; one not down when the placement ramp, aimed
  // PLACE_DEPTH into the ground, has ended is not where the controller
  // thinks, and the tick fails rather than waiting for it. With both down
  // ("Balance stop", Fig. 6.3) the load transfer starts from the last
  // two-foot forces, and the MPC re-plans on four feet on this very tick
  // instead of up to a solve period later. Its plan for the 3-contact
  // interval in between would carry no weight in the blend (s = 0), so it
  // keeps the two-foot schedule until then.
  const double elapsed = in.t - _t0;
  double height[2];
  for (int k = 0; k < 2; k++) height[k] = dyn.contact[(k == 0) ? 1 : 2].p.z() - pr.o.z();
  if (_tPlace >= 0.0 && _tTransfer < 0.0) {
    for (int k = 0; k < 2; k++) {
      const double vz = (dyn.contact[(k == 0) ? 1 : 2].J * v).z();
      if (_promoted[k] || height[k] > TOUCHDOWN_HEIGHT || vz < -TOUCHDOWN_SPEED) continue;
      _promoted[k] = true;
      if (_mgr)
        _mgr->message("MdlAMB: t=%.3f %s promoted to contact (height %.1f mm, %.1f mm/s)", in.t,
                      (k == 0) ? "FR" : "RL", 1e3 * height[k], 1e3 * vz);
    }
    if (_promoted[0] && _promoted[1]) {
      _tTransfer = elapsed;
      for (int leg = 0; leg < 4; leg++) _f_frz[leg] = _f_des[leg];
      _tMpc = in.t;
      if (_mgr) _mgr->message("MdlAMB: t=%.3f balance stop, load transfer started", in.t);
    } else if (place >= 1.0) {
      if (_mgr)
        _mgr->warning(AMBMODULE_NAME, "Placement ended at t=%.3f with FR %.1f mm, RL %.1f mm "
                      "above the ground", in.t, 1e3 * height[0], 1e3 * height[1]);
      return false;
    }
  }

  // F_c^d of (6.7) from the MPC at its solve period, as MdlTrot does: FL and
  // RR over the whole horizon (all four feet from the load transfer on),
  // holding the latched pose. Times come from an integer-microsecond clock,
  // hence the half-microsecond tolerance.
  if (in.t >= _tMpc - 0.5e-6) {
    const MdlConvexMPC::params_t& mp = _mpc->getParams();
    MdlConvexMPC::input_t mi;
    const Eigen::Vector3d rpy = MdlOrientationEstimator::rpyFromQuat(Eigen::Quaterniond(in.R));
    // Yaw kept next to its reference: a heading near +-pi is not a 2 pi error.
    mi.current.rpy = Eigen::Vector3d(rpy(0), rpy(1), _rpy_ref(2) + wrapPi(rpy(2) - _rpy_ref(2)));
    mi.current.com_position_world = p_base + dyn.com;
    mi.current.angular_velocity_world = w_world;
    // With v_base in world axes, the base-linear rows of M v are the total
    // linear momentum.
    mi.current.com_velocity_world = dyn.M.topRows<3>() * v / dyn.mass;
    for (int k = 0; k < mp.horizon; k++) {
      // The orientation task's reference, sampled where reference[k] lives
      // (now + (k+1) dt), so the planned forces already turn the body with it.
      Eigen::Matrix3d R_k;
      _oriReference(in.t - _t0 + (k + 1) * mp.dt, R_k, mi.reference[k].rpy,
                    mi.reference[k].angular_velocity_world);
      // The COM reference moves with the COM offset, so the lifting swing legs
      // do not read as a height error the body position task does not see.
      mi.reference[k].com_position_world = _p_ref + dyn.com;
      for (int leg = 0; leg < 4; leg++) {
        mi.contact[k][leg] = (leg == 0 || leg == 3 || _tTransfer >= 0.0);
        mi.moment_arm_world[k].col(leg) = dyn.contact[leg].p - dyn.com;
      }
    }
    MdlConvexMPC::output_t mo;
    if (!_mpc->solve(mi, mo)) {
      if (_mgr)
        _mgr->warning(AMBMODULE_NAME, "MPC solve failed (status %d) at t=%.3f", mo.solver_status,
                      in.t);
      return false;
    }
    // Next point of the solve lattice after now, so one late tick does not
    // become a solve on every tick that follows (MdlTrot's
    // PeriodicSolveSchedule). A solver that solved has been reset(), which
    // makes solve_period positive.
    while (_tMpc <= in.t + 0.5e-6) _tMpc += mp.solve_period;
    for (int leg = 0; leg < 4; leg++) {
      // Table 6.2, Sec. 6.4.3: tangential at most force_ratio of the normal
      // load before the WBC sees it (its own pyramid is the wider friction one).
      // The MPC pins an unscheduled foot to zero, which this leaves at zero.
      Eigen::Vector3d f = mo.force_world[leg];
      const double ft = f.head<2>().norm(), lim = _p.force_ratio * std::max(f.z(), 0.0);
      if (ft > lim) f.head<2>() *= lim / ft;
      _f_des[leg] = f;
    }
  }

  QuadrupedWBC::input_t wi;
  wi.dyn = &dyn;
  // A promoted foot is a contact from its touchdown; until the transfer (the
  // other foot's touchdown, 10-75 ms later in the runs here) its desired force
  // is the MPC's zero, so the QP loads it only as far as the body tasks need
  // it. In the transfer the desired forces go linearly from the frozen
  // two-foot split to the live four-foot plan, the way Table 6.6's desired
  // values do (stance feet down from their two-foot load, the former swing
  // feet up from zero); both ends obey the 0.10 limit, so the blend does.
  const double s_tr =
      (_tTransfer < 0.0) ? 0.0 : std::min(1.0, (elapsed - _tTransfer) / TRANSFER_DURATION);
  for (int leg = 0; leg < 4; leg++) {
    wi.q[leg] = in.q[leg];
    wi.contact[leg] = (leg == 0 || leg == 3 || _promoted[(leg == 1) ? 0 : 1]);
    wi.f_des[leg] = _f_des[leg];
    if (_tTransfer >= 0.0) wi.f_des[leg] = (1.0 - s_tr) * _f_frz[leg] + s_tr * _f_des[leg];
  }
  // Thesis 6.2.1 order: body orientation, body position, then the swing feet
  // that are not contacts.
  wi.num_tasks = 2;
  // Orientation: J v = R w, whose rate is exactly J vdot (R (w x w) = 0), so
  // Jdv = 0. Error: rotation vector of R_d R^T, world axes. No feedforward
  // acceleration: the sinusoids' peak, 5 deg (2 pi 0.1 Hz)^2 = 0.035 rad/s^2,
  // is what kp_body_ori makes of a 0.02 deg error, and adding it moved neither
  // the tracked amplitude nor the lag (test_amb_wbc_sim rig).
  QuadrupedWBC::task_t& ori = wi.task[0];
  ori.J.block<3, 3>(0, 3) = in.R;
  Eigen::Matrix3d R_d;
  Eigen::Vector3d rpy_d;
  _oriReference(in.t - _t0, R_d, rpy_d, ori.vel_des);
  const Eigen::AngleAxisd e_ori(R_d * in.R.transpose());
  ori.pos_err = e_ori.angle() * e_ori.axis();
  ori.acc_cmd = _p.kp_body_ori * ori.pos_err + _p.kd_body_ori * (ori.vel_des - w_world);
  // Position: the base origin, whose world-axes velocity is v[0:3].
  QuadrupedWBC::task_t& pos = wi.task[1];
  pos.J.block<3, 3>(0, 0).setIdentity();
  pos.pos_err = _p_ref - p_base;
  pos.acc_cmd = _p.kp_body_pos * pos.pos_err - _p.kd_body_pos * st.v_base;

  for (int k = 0; k < 2; k++) {
    const int leg = (k == 0) ? 1 : 2;
    const QuadrupedDynamics::point_t &foot = dyn.foot[leg], &hip = dyn.hip[leg];
    const Eigen::Vector3d d_f = foot.p - hip.p;
    const Eigen::Vector3d v_rel = foot.J * v - hip.J * v;
    // Lift: normal and radial references from the hand-off values (a) to the
    // lift target (b). Placement (Fig. 6.3) runs it backwards on the slower
    // ramp: the normal reference returns to its hand-off value and the radial
    // one rises to where the contact point meets the ground, PLACE_DEPTH
    // under it, along the current radial axis. Not to rho0: rho0 reaches the
    // ground only at the hand-off tangential angle, and the AMB-owned
    // tangential channel does not return there, because x2 drifts. In the
    // full stack x2 is -3 deg at hand-off, +26 deg at a stop 5 s later and
    // still +9 deg when a ramp back to rho0 ended, with FR 19.5 mm above the
    // ground and RL's reference 12 mm below it.
    double xa = _xi_n0[k], xb = _xi_n1[k], ra = _rho0[k], rb = _rho1[k];
    double sg = sigma, sg_dot = sigma_dot, sg_ddot = sigma_ddot;
    if (_tPlace >= 0.0) {
      const Eigen::Vector3d e_r = (d_f - d_f.dot(s) * s).normalized();
      xa = _xi_n1[k];
      xb = _xi_n0[k];
      ra = _rho1[k];
      rb = (pr.o.z() + FOOT_RADIUS - PLACE_DEPTH - hip.p.z() - xb * s.z()) / e_r.z();
      sg = place;
      sg_dot = place_dot;
      sg_ddot = place_ddot;
    }
    const double xi_n_d = xa + (xb - xa) * sg;
    const double rho_d = ra + (rb - ra) * sg;
    Eigen::Matrix3d R_WP;
    double rho = 0.0;
    const bool valid = principalFrame(s, d_f, R_WP, rho);
    const double xi_t = tangentialArc(s, _e_r0[k], rho_d, d_f);
    // (6.23): while AMB acts the tangential references are the measured ones,
    // so Cartesian feedback acts on the normal and radial channels only. With
    // AMB off (the Sec. 6.6.4 ablation) or a rejected frame (Sec. 6.4.3) there
    // is no balance acceleration, and the ordinary Cartesian task takes the
    // tangential channel through the same lift as the other two, to the lift
    // target's angle about e_r0, and holds it there: the posture the
    // joint-space ablation PD-holds. (xi_t_d_dot is rho_d d(alpha^d)/dt.)
    double xi_t_d = rho_d * _alpha1[k] * sigma, xi_t_d_dot = rho_d * _alpha1[k] * sigma_dot;
    double xi_t_d_ddot = rho_d * _alpha1[k] * sigma_ddot;
    double ff_status = 2.0;
    Eigen::Vector3d a_f = Eigen::Vector3d::Zero();
    if (_p.enabled && valid) {
      xi_t_d = xi_t;
      xi_t_d_dot = rho_d * R_WP.col(2).dot(v_rel) / rho;  // rho_d d(alpha)/dt
      xi_t_d_ddot = 0.0;  // the tangential acceleration is AMB's a_f
      ff_status =
          tangentialFeedforward(rho, thdd[k], R_WP.col(2), _p.foot_acc_max, a_f) ? 1.0 : 0.0;
    }
    Eigen::Vector3d d_des, e_t_des;
    swingReference(s, _e_r0[k], xi_n_d, rho_d, xi_t_d, d_des, e_t_des);
    const Eigen::Vector3d e_r_des = (d_des - xi_n_d * s) / rho_d;
    // A promoted foot is a contact: no swing task and no AMB feedforward. Its
    // principal coordinates are still logged; the desired ones are unused.
    if (_promoted[k]) {
      ff_status = 3.0;
    } else {
      QuadrupedWBC::task_t& sw = wi.task[wi.num_tasks++];
      sw.J = foot.J;
      sw.Jdv = foot.Jdv;
      sw.pos_err = d_des - d_f;  // p_f^d - p_f of (6.27): the hip position cancels
      // (6.25) with the ramp's normal rate, plus its radial rate, which (6.25)
      // lacks only because the thesis' experiments hold rho^d.
      sw.vel_des = hip.J * v + (xb - xa) * sg_dot * s + (rb - ra) * sg_dot * e_r_des +
                   xi_t_d_dot * e_t_des;
      // (6.3)'s reference acceleration of the same ramps, which (6.27) lacks
      // only because the thesis holds its references. Without it the 0.4 s
      // lift lags by its acceleration over kp_foot: RL's normal error was 57
      // mm just after the lift (5.8 * 0.21 m / 0.4^2 s^2 / 150 = 51 mm).
      sw.acc_cmd = _p.kp_foot * sw.pos_err + _p.kd_foot * (sw.vel_des - foot.J * v) + a_f +
                   (xb - xa) * sg_ddot * s + (rb - ra) * sg_ddot * e_r_des + xi_t_d_ddot * e_t_des;
    }

    double* lg = _logWbc + 14 + 7 * k;
    lg[0] = d_f.dot(s);
    lg[1] = (d_f - d_f.dot(s) * s).norm();
    lg[2] = xi_t;
    lg[3] = xi_n_d;
    lg[4] = rho_d;
    lg[5] = xi_t_d;
    lg[6] = ff_status;
  }

  QuadrupedWBC::output_t wo;
  const bool solved = _wbc->solve(wi, wo);
  _logWbc[1] = wo.qp_status;
  if (!solved) {
    if (_mgr)
      _mgr->warning(AMBMODULE_NAME, "WBC solve failed (qpOASES status %d) at t=%.3f",
                    wo.qp_status, in.t);
    return false;
  }

  // (6.28): stance legs, promoted swing legs included, follow the kinematic
  // stage under the joint PD with the WBIC torque as feedforward; the swing
  // legs get the WBIC torque alone.
  double lim_dist = INFINITY;
  for (int leg = 0; leg < 4; leg++) {
    out[leg].tau = wo.tau.segment<3>(3 * leg);
    if (leg == 1 || leg == 2) {
      const Eigen::Vector3d lo(geo.hip_abduction_limits(leg, 0), geo.hip_flexion_limits(leg, 0),
                               geo.knee_limits(leg, 0));
      const Eigen::Vector3d hi(geo.hip_abduction_limits(leg, 1), geo.hip_flexion_limits(leg, 1),
                               geo.knee_limits(leg, 1));
      lim_dist = std::min({lim_dist, (in.q[leg] - lo).minCoeff(), (hi - in.q[leg]).minCoeff()});
    }
    if (wi.contact[leg]) {
      out[leg].a = wo.q_des[leg];
      out[leg].adot = wo.qd_des[leg];
      out[leg].kp.setConstant(_p.kp_joint);
      out[leg].kd.setConstant(_p.kd_joint);
    } else {
      out[leg].a = in.q[leg];
      out[leg].adot.setZero();
      out[leg].kp.setZero();
      // Not zero: MdlSimDriver rewrites kd <= 0 to a 5 Nm s/rad brake on
      // exactly the motion AMB asks for (as on the joint-space swing roll).
      out[leg].kd.setConstant(1e-3);
      // The joints' viscous and dry friction are not in the model's h, and
      // with kp = 0 nothing else absorbs them. Uncancelled, the 0.2 Nm of dry
      // friction stalls the small accelerations AMB asks for and the swing
      // legs creep into the hip-roll stop: the full-stack autorun trips 4.6 s
      // after the hand-off, test_amb_wbc_sim falls at 3.2 s, and balances 10 s
      // when the swing joints' friction is instead removed from the model.
      // Same cancellation as the joint-space swing roll, the sign at rest
      // taken from the WBC's commanded acceleration.
      const Eigen::Vector3d qdd = wo.vdot.segment<3>(6 + 3 * leg);
      for (int j = 0; j < 3; j++) {
        const double qd = in.qd[leg](j);
        const double sv = std::tanh((qd + 0.01 * qdd(j)) / 0.02);
        out[leg].tau(j) += _p.joint_damping * qd +
                           _p.joint_friction * (sv + (1.0 - std::abs(sv)) * std::tanh(qdd(j) / 0.1));
      }
    }
    // setJointCommand rejects a target outside the range (inert at kp = 0).
    _kin->clampToJointLimits(leg, out[leg].a);
  }

  // Phase for the log; SETTLED once the final hold has lasted FINAL_HOLD.
  double phase = sigma < 1.0 ? 0.0 : 1.0;
  if (_tTransfer >= 0.0) {
    phase = (s_tr < 1.0) ? 3.0 : 4.0;
    if (_status != SETTLED && elapsed - _tTransfer >= TRANSFER_DURATION + FINAL_HOLD) {
      _status = SETTLED;
      if (_mgr) _mgr->message("MdlAMB: t=%.3f SETTLED in four-foot support", in.t);
    }
  } else if (_tPlace >= 0.0) {
    phase = 2.0;
  }
  _logWbc[0] = phase;
  // Desired and solved forces: FL, RR at 2-7 and 8-13, FR, RL at 38-43 and 44-49.
  const int des[4] = {2, 38, 41, 5}, sol[4] = {8, 44, 47, 11};
  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      _logWbc[des[leg] + j] = wi.f_des[leg](j);
      _logWbc[sol[leg] + j] = wo.f[leg](j);
    }
  }
  for (int j = 0; j < 3; j++) {
    _logWbc[28 + j] = pos.pos_err(j);
    _logWbc[31 + j] = ori.pos_err(j);
    _logWbc[35 + j] = st.v_base(j);
  }
  _logWbc[34] = lim_dist;
  _logWbc[50] = height[0];
  _logWbc[51] = height[1];
  return true;
}

void MdlAMB::_oriReference(double t, Eigen::Matrix3d& R_d, Eigen::Vector3d& rpy_d,
                           Eigen::Vector3d& w_d) const {
  double dp, dp_dot, dy, dy_dot;
  oriSinusoid(_p.pitch_amp_deg * DEG, t, dp, dp_dot);
  oriSinusoid(_p.yaw_amp_deg * DEG, t, dy, dy_dot);
  // A stop fades them out on the placement ramp, as it returns x3 to its
  // hand-off value: the feet are placed, and four-foot support held, at the
  // latched attitude they were lifted at.
  if (_tPlace >= 0.0) {
    double g, g_dot;
    TrajectoryUtils::sampleQuintic((t - _tPlace) / _p.place_duration, _p.place_duration, g, g_dot);
    dp_dot = (1.0 - g) * dp_dot - g_dot * dp;
    dy_dot = (1.0 - g) * dy_dot - g_dot * dy;
    dp *= 1.0 - g;
    dy *= 1.0 - g;
  }
  // ZYX: Rz(yaw + dy) Ry(pitch + dp) Rx(roll) = Rz(dy) R_ref Rot(a, dp), a the
  // intermediate y axis in body axes, Rx(roll)^T e_y. Composed onto R_ref
  // rather than rebuilt from the angles, so zero amplitudes give R_ref bit for
  // bit; R_d a is the world pitch axis Rz(yaw + dy) e_y.
  const Eigen::Vector3d a(0.0, std::cos(_rpy_ref(0)), -std::sin(_rpy_ref(0)));
  R_d = Eigen::AngleAxisd(dy, Eigen::Vector3d::UnitZ()).toRotationMatrix() * _R_ref *
        Eigen::AngleAxisd(dp, a).toRotationMatrix();
  rpy_d = _rpy_ref + Eigen::Vector3d(0.0, dp, dy);
  w_d = dy_dot * Eigen::Vector3d::UnitZ() + dp_dot * (R_d * a);
}

void MdlAMB::update() {
  // SETTLED keeps commanding: it holds four-foot support until released.
  if (_status != ACTIVE && _status != SETTLED) return;

  input_t in;
  MotorHW *hw = MotorHW::instance();
  for (int leg = 0; leg < 4; leg++) {
    for (int j = 0; j < 3; j++) {
      const int idx = 3 * leg + j;
      if (hw->getStatus(idx) != MotorHW::STATUS_READY) {
        _status = ERROR;
        _mgr->warning(AMBMODULE_NAME, "Motor %d not ready at t=%.3f", idx, _mgr->readTime());
        return;
      }
      MotorHW::state_t st;
      hw->getState(idx, st);
      in.q[leg](j) = st.pos;
      in.qd[leg](j) = st.vel;
    }
  }
  // Freshness comes from the flag, not from _imu.t (the sim stamps it in
  // microseconds, Go1 in seconds). With a frozen R the stance geometry barely
  // moves, so theta1 would sit near its last value, never trip theta1_max, and
  // the law would keep commanding the rolls while the robot falls.
  if (!IMUHW::instance() || !IMUHW::instance()->getLastReading(0, _imu)) {
    if (++_imuMiss > 5) {
      _status = ERROR;
      _mgr->warning(AMBMODULE_NAME, "IMU stale at t=%.3f", _mgr->readTime());
      return;
    }
  } else {
    _imuMiss = 0;
  }
  Eigen::Quaterniond q_BW(_imu.q.v[0], _imu.q.v[1], _imu.q.v[2], _imu.q.v[3]);
  in.w_body = Eigen::Vector3d(_imu.gyro.v[0], _imu.gyro.v[1], _imu.gyro.v[2]);
  in.t = _mgr->readTime();

  bool finite = _imu.t >= 0.0 && std::isfinite(q_BW.norm()) && q_BW.norm() > 0.5 &&
                in.w_body.allFinite();
  for (int leg = 0; leg < 4; leg++)
    finite = finite && in.q[leg].allFinite() && in.qd[leg].allFinite();
  if (!finite) {
    _status = ERROR;
    _mgr->warning(AMBMODULE_NAME, "Non-finite joint or IMU state at t=%.3f", in.t);
    return;
  }
  in.R = q_BW.normalized().toRotationMatrix();
  // attitude_source = "estimator": the IMU sample above only guards freshness;
  // the attitude and body rate are MdlOrientationEstimator's, which runs in
  // the sensing band so this tick's estimate is already published.
  if (_orientation) {
    if (!_orientation->isReady()) {
      _status = ERROR;
      _mgr->warning(AMBMODULE_NAME, "Orientation estimate not ready at t=%.3f", in.t);
      return;
    }
    in.R = _orientation->getRotation();
    in.w_body = _orientation->getAngularVelocity();
  }

  cmd_t out[4];
  bool ok;
  if (_p.wbc) {
    // Timed for deactivate()'s report: the MPC re-solve has to fit the period.
    const auto t_tick = std::chrono::steady_clock::now();
    ok = computeTick(in, out);
    const double dt_tick =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_tick).count();
    _tickSum += dt_tick;
    _tickMax = std::max(_tickMax, dt_tick);
    _tickCount++;
  } else {
    ok = computeTick(in, out);
  }
  if (!ok) {
    _status = ERROR;
    _mgr->warning(AMBMODULE_NAME,
                  "Tick failed at t=%.3f: theta1=%.1f deg l_c=%.3f m j_c=%.2f j_d=%.2f", in.t,
                  _log[0] / DEG, _log[21], _log[18], _log[19]);
    return;
  }
  for (int i = 0; i < 4; i++) {
    if (!_legs[i]->setJointCommand(out[i].a, out[i].adot, out[i].kp, out[i].kd, out[i].tau)) {
      _status = ERROR;
      _mgr->warning(AMBMODULE_NAME, "Leg %d command rejected at t=%.3f", i, in.t);
      return;
    }
  }
}
