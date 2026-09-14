/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "quadruped/MdlPosVelEstimator.hh"

#include <cmath>
#include <cstdio>

#include "hardware/MotorHW.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/QuadrupedConfigs.hh"
#include "quadruped/QuadrupedKinematics.hh"
#include "rtcore/ConfigTable.hh"
#include "rtcore/LogServer.hh"
#include "rtcore/ModuleManager.hh"

#define DBGPRINT(...)  // printf(__VA_ARGS__)

using namespace rtcore;

// Optional simulation hooks; all uses are null-safe on non-simulation targets.
extern bool simGetGroundTruth(double pos[3], double quat[4], double linvel[3],
                              double angvel[3]) __attribute__((weak));
extern bool simGetFootPositions(double pos[4][3]) __attribute__((weak));
extern void simDebugClear() __attribute__((weak));
extern void simDebugSphere(const double pos[3], double radius,
                           const double rgba[4]) __attribute__((weak));
extern void simDebugLine(const double from[3], const double to[3], double width,
                         const double rgba[4]) __attribute__((weak));

MdlPosVelEstimator::MdlPosVelEstimator()
    : Module(POSVELMODULE_NAME, 0, MULTI_USER) {
  DBGPRINT("MdlPosVelEstimator::MdlPosVelEstimator\n");
  for (int i = 0; i < NUM_LEGS; i++) {
    _jointAngles[i].setZero();
    _jointVel[i].setZero();
    _footPosBody[i].setZero();
    _footVelBody[i].setZero();
    _footJacobian[i].setIdentity();
    _footTruePos[i].setZero();
  }
}

MdlPosVelEstimator::~MdlPosVelEstimator() {
  DBGPRINT("MdlPosVelEstimator::~MdlPosVelEstimator\n");
}

// ===========================================================================
//  The filter
// ===========================================================================

double MdlPosVelEstimator::trustFromPhase(double phase, double window) {
  if (phase > 1.0) phase = 1.0;
  if (phase < 0.0) phase = 0.0;
  // A nonpositive window disables the ramp and gives full trust.
  if (!(window > 0.0)) return 1.0;

  if (phase < window) return phase / window;
  if (phase > 1.0 - window) return (1.0 - phase) / window;
  return 1.0;
}

void MdlPosVelEstimator::setParams(const params_t& params) {
  _params = params;
  if (!(_params.dt > 0.0)) _params.dt = 0.001;
  if (!(_params.trust_window >= 0.0)) _params.trust_window = 0.0;
  if (_params.trust_window > 0.5) _params.trust_window = 0.5;
}

void MdlPosVelEstimator::reset() {
  const double dt = _params.dt;

  _xhat.setZero();

  // Preserve the horizontal world-frame datum across activations.
  _xhat.head<2>() = _originCarry;

  // Model body position and velocity as a double integrator and foot positions as constant.
  _A.setZero();
  _A.block<3, 3>(0, 0).setIdentity();
  _A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
  _A.block<3, 3>(3, 3).setIdentity();
  _A.block<12, 12>(6, 6).setIdentity();

  _B.setZero();
  _B.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();

  // C1 selects the body position out of (r, v); C2 selects the body velocity.
  Eigen::Matrix<double, 3, 6> C1, C2;
  C1 << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
  C2 << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

  _C.setZero();
  for (int i = 0; i < NUM_LEGS; i++) {
    _C.block<3, 6>(3 * i, 0) = C1;       // rows  0-11: r - p_i
    _C.block<3, 6>(12 + 3 * i, 0) = C2;  // rows 12-23: v
    _C(24 + i, 6 + 3 * i + 2) = 1.0;     // rows 24-27: world z of foot i
  }
  _C.block<12, 12>(0, 6) = -Eigen::Matrix<double, 12, 12>::Identity();

  // Start with a large diagonal covariance because measurements initialize the state.
  _P.setIdentity();
  _P *= 100.0;

  // Build fixed covariance shapes that configuration values scale.
  _Q0.setIdentity();
  _Q0.block<3, 3>(0, 0) = (dt / 20.0) * Eigen::Matrix3d::Identity();
  // The process-noise shape uses a fixed position-to-velocity ratio; gravity enters in step().
  _Q0.block<3, 3>(3, 3) = (dt * 9.8 / 20.0) * Eigen::Matrix3d::Identity();
  _Q0.block<12, 12>(6, 6) = dt * Eigen::Matrix<double, 12, 12>::Identity();

  _R0.setIdentity();

  _Rbw.setIdentity();
  _vBody.setZero();
  for (int i = 0; i < NUM_LEGS; i++) _trust[i] = 0.0;
  _needSeed = true;
  _ready = false;
}

void MdlPosVelEstimator::_seedFromKinematics(const Eigen::Matrix3d& Rbw,
                                             const Eigen::Vector3d footPos[NUM_LEGS],
                                             const double phase[NUM_LEGS]) {
  // Wait until at least one foot has positive contact phase before seeding.
  double sum = 0.0;
  int n = 0;
  for (int i = 0; i < NUM_LEGS; i++) {
    if (phase[i] <= 0.0) continue;
    sum += (Rbw * footPos[i]).z();
    n++;
  }
  if (n == 0) return;

  // Place body and foot states on the ground datum while preserving the horizontal origin.
  _xhat.setZero();
  _xhat.head<2>() = _originCarry;
  _xhat(2) = _params.foot_ground_height - sum / (double)n;
  for (int i = 0; i < NUM_LEGS; i++)
    _xhat.segment<3>(6 + 3 * i) = _xhat.head<3>() + Rbw * footPos[i];

  // Keep the initial covariance large so measurements can correct the seeded mean.
  _needSeed = false;
}

void MdlPosVelEstimator::step(const Eigen::Matrix3d& Rbw, const Eigen::Vector3d& aWorld,
                              const Eigen::Vector3d& omegaBody,
                              const Eigen::Vector3d footPos[NUM_LEGS],
                              const Eigen::Vector3d footVel[NUM_LEGS],
                              const double phase[NUM_LEGS]) {
  _Rbw = Rbw;

  Eigen::Matrix<double, DIM, DIM> Q = Eigen::Matrix<double, DIM, DIM>::Identity();
  Q.block<3, 3>(0, 0) = _Q0.block<3, 3>(0, 0) * _params.imu_process_noise_position;
  Q.block<3, 3>(3, 3) = _Q0.block<3, 3>(3, 3) * _params.imu_process_noise_velocity;
  Q.block<12, 12>(6, 6) = _Q0.block<12, 12>(6, 6) * _params.foot_process_noise_position;

  Eigen::Matrix<double, MEAS, MEAS> R = Eigen::Matrix<double, MEAS, MEAS>::Identity();
  R.block<12, 12>(0, 0) = _R0.block<12, 12>(0, 0) * _params.foot_sensor_noise_position;
  R.block<12, 12>(12, 12) = _R0.block<12, 12>(12, 12) * _params.foot_sensor_noise_velocity;
  R.block<4, 4>(24, 24) = _R0.block<4, 4>(24, 24) * _params.foot_height_sensor_noise;

  // Convert world-frame specific force to inertial acceleration by adding gravity.
  const Eigen::Vector3d a = aWorld + _params.gravity;

  if (_needSeed) {
    if (_params.warm_start)
      _seedFromKinematics(Rbw, footPos, phase);
    else
      _needSeed = false;
  }

  // Snapshot the pre-propagation state for untrusted-foot measurement blending.
  const Eigen::Vector3d r0 = _xhat.head<3>();
  const Eigen::Vector3d v0 = _xhat.segment<3>(3);

  Eigen::Matrix<double, 12, 1> ps, vs;
  Eigen::Vector4d pzs = Eigen::Vector4d::Zero();

  for (int i = 0; i < NUM_LEGS; i++) {
    const int i1 = 3 * i;

    // Kinematics already returns the foot position in the body frame.
    const Eigen::Vector3d& p_rel = footPos[i];
    const Eigen::Vector3d& dp_rel = footVel[i];

    // Rotate positions and include angular transport in world-frame foot velocity.
    const Eigen::Vector3d p_f = Rbw * p_rel;
    const Eigen::Vector3d dp_f = Rbw * (omegaBody.cross(p_rel) + dp_rel);

    const int qindex = 6 + i1, rindex2 = 12 + i1, rindex3 = 24 + i;

    const double trust = trustFromPhase(phase[i], _params.trust_window);
    _trust[i] = trust;

    // Inflate foot process and velocity/height measurement noise as trust decreases; retain position anchoring.
    const double suspect = 1.0 + (1.0 - trust) * _params.high_suspect_number;
    Q.block<3, 3>(qindex, qindex) *= suspect;
    R.block<3, 3>(rindex2, rindex2) *= suspect;
    R(rindex3, rindex3) *= suspect;

    // Match the body-to-foot measurement sign used by _C.
    ps.segment<3>(i1) = -p_f;

    // Blend stationary-foot velocity with the current estimate according to trust.
    vs.segment<3>(i1) = (1.0 - trust) * v0 + trust * (-dp_f);

    // Blend foot height toward the configured ground datum according to trust.
    pzs(i) = (1.0 - trust) * (r0.z() + p_f.z()) + trust * _params.foot_ground_height;
  }

  Eigen::Matrix<double, MEAS, 1> y;
  y << ps, vs, pzs;

  _xhat = _A * _xhat + _B * a;
  const Eigen::Matrix<double, DIM, DIM> Pm = _A * _P * _A.transpose() + Q;
  const Eigen::Matrix<double, DIM, MEAS> Ct = _C.transpose();
  const Eigen::Matrix<double, MEAS, 1> ey = y - _C * _xhat;
  const Eigen::Matrix<double, MEAS, MEAS> S = _C * Pm * Ct + R;

  // Reuse one factorization for the state and covariance updates.
  const Eigen::PartialPivLU<Eigen::Matrix<double, MEAS, MEAS> > lu(S);
  _xhat += Pm * Ct * lu.solve(ey);
  _P = (Eigen::Matrix<double, DIM, DIM>::Identity() - Pm * Ct * lu.solve(_C)) * Pm;

  // Force evaluation before assigning to the matrix being read.
  _P = 0.5 * (_P + _P.transpose()).eval();

  // Limit the unobservable horizontal covariance block to maintain conditioning.
  if (_P.block<2, 2>(0, 0).determinant() > 1e-6) {
    _P.block<2, 16>(0, 2).setZero();
    _P.block<16, 2>(2, 0).setZero();
    _P.block<2, 2>(0, 0) /= 10.0;
  }

  _vBody = Rbw.transpose() * _xhat.segment<3>(3);
  _ready = true;
}

// ===========================================================================
//  Module interface
// ===========================================================================

void MdlPosVelEstimator::init() {
  DBGPRINT("MdlPosVelEstimator::init\n");

  // Own kinematics so the estimator does not contend with behavior modules.
  ConfigTable hwConfig;
  std::string hwlib;
  if (_mgr->getConfigTable("hardware", hwConfig))
    hwlib = hwConfig.getString("library", "");

  if (hwlib == "go1hw")
    _kinematics = new QuadrupedKinematics(createGo1Config());
  else
    _kinematics = new QuadrupedKinematics(createGo2Config());

  _dt = CLOCK_TO_SEC(_mgr->getStepPeriod());
  _readConfig();
  reset();

  // Require the attitude stage, which must be created earlier in the cycle.
  _orientation = (MdlOrientationEstimator*)_mgr->findModule(ORIENTATIONMODULE_NAME, 0);
  if (!_orientation)
    _mgr->fatalError(POSVELMODULE_NAME,
                     "%s must be created before %s in AddCoreModules()",
                     ORIENTATIONMODULE_NAME, POSVELMODULE_NAME);

  _logserver = (LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
  if (_logserver) {
    _logserver->registerVar(LOG_DOUBLE, 9, POSVELMODULE_NAME, "state",
                            (unsigned char*)_logState);
    _logserver->registerVar(LOG_DOUBLE, 12, POSVELMODULE_NAME, "footholds",
                            (unsigned char*)_logFootholds);
    _logserver->registerVar(LOG_DOUBLE, 9, POSVELMODULE_NAME, "error",
                            (unsigned char*)_logError);
    _logserver->registerVar(LOG_DOUBLE, DIM, POSVELMODULE_NAME, "cov",
                            (unsigned char*)_logCov);
    _logserver->registerVar(LOG_DOUBLE, 4, POSVELMODULE_NAME, "contacts",
                            (unsigned char*)_logContacts);
    _logserver->registerVar(LOG_DOUBLE, 4, POSVELMODULE_NAME, "footerr",
                            (unsigned char*)_logFootErr);
    _logserver->registerVar(LOG_DOUBLE, 12, POSVELMODULE_NAME, "foottruth",
                            (unsigned char*)_logFootTruth);
  }
}

void MdlPosVelEstimator::uninit() {
  DBGPRINT("MdlPosVelEstimator::uninit\n");

  if (_logserver) {
    _logserver->deleteVar(POSVELMODULE_NAME, "state");
    _logserver->deleteVar(POSVELMODULE_NAME, "footholds");
    _logserver->deleteVar(POSVELMODULE_NAME, "error");
    _logserver->deleteVar(POSVELMODULE_NAME, "cov");
    _logserver->deleteVar(POSVELMODULE_NAME, "contacts");
    _logserver->deleteVar(POSVELMODULE_NAME, "footerr");
    _logserver->deleteVar(POSVELMODULE_NAME, "foottruth");
    _logserver = nullptr;
  }

  if (_kinematics) delete _kinematics;
  _kinematics = nullptr;
}

void MdlPosVelEstimator::_readConfig() {
  ConfigTable config;
  if (!_mgr->getConfigTable("posvelestimator", config)) {
    _mgr->warning(POSVELMODULE_NAME,
                  "No [posvelestimator] config table; using built-in defaults");
    return;
  }

  _enable = config.getBool("enable", true);

  params_t p;
  p.dt = _dt;
  p.imu_process_noise_position =
      config.getDouble("imu_process_noise_position", p.imu_process_noise_position);
  p.imu_process_noise_velocity =
      config.getDouble("imu_process_noise_velocity", p.imu_process_noise_velocity);
  p.foot_process_noise_position =
      config.getDouble("foot_process_noise_position", p.foot_process_noise_position);
  p.foot_sensor_noise_position =
      config.getDouble("foot_sensor_noise_position", p.foot_sensor_noise_position);
  p.foot_sensor_noise_velocity =
      config.getDouble("foot_sensor_noise_velocity", p.foot_sensor_noise_velocity);
  p.foot_height_sensor_noise =
      config.getDouble("foot_height_sensor_noise", p.foot_height_sensor_noise);
  p.trust_window = config.getDouble("trust_window", p.trust_window);
  p.high_suspect_number = config.getDouble("high_suspect_number", p.high_suspect_number);
  p.foot_ground_height = config.getDouble("foot_ground_height", p.foot_ground_height);
  p.warm_start = config.getBool("warm_start", p.warm_start);
  p.use_ground_truth = config.getBool("use_ground_truth", p.use_ground_truth);

  ConfigArray gravity;
  if (config.getArray("gravity", gravity) && gravity.size() >= 3)
    p.gravity = Eigen::Vector3d(gravity.getDoubleAt(0, 0.0), gravity.getDoubleAt(1, 0.0),
                                gravity.getDoubleAt(2, -9.81));

  if (p.use_ground_truth && !simGetGroundTruth) {
    _mgr->warning(POSVELMODULE_NAME,
                  "use_ground_truth needs the simulation target; running the filter");
    p.use_ground_truth = false;
  }

  setParams(p);

  // Warn when obsolete configuration keys are still present.
  if (config.getDouble("foot_radius", -1.0) >= 0.0)
    _mgr->warning(POSVELMODULE_NAME,
                  "foot_radius is obsolete; it is now foot_ground_height and is an "
                  "active height measurement rather than a fixed datum");
  if (config.getDouble("swing_foot_noise", -1.0) >= 0.0)
    _mgr->warning(POSVELMODULE_NAME,
                  "swing_foot_noise is obsolete; swing feet are now handled by the "
                  "contact trust ramp (trust_window, high_suspect_number)");
  if (!config.getString("contact_source", "").empty() ||
      !config.getString("contact_phase_source", "").empty())
    _mgr->warning(POSVELMODULE_NAME,
                  "contact_source and contact_phase_source are obsolete, along with "
                  "the contact_* thresholds, jacobian_damping and contact_force_sign. "
                  "Contact phase now comes from the gait schedule alone, as in the MIT "
                  "sources; there is no contact detector");

  ConfigTable viz;
  if (config.getTable("viz", viz)) {
    _vizEnable = viz.getBool("enable", true);
    _vizTrail = viz.getBool("show_trail", true);
    _vizTrailLength = (int)viz.getInt("trail_length", 200);
    if (_vizTrailLength > MAX_TRAIL) _vizTrailLength = MAX_TRAIL;
    if (_vizTrailLength < 0) _vizTrailLength = 0;
    _vizTrailDecimation = (int)viz.getInt("trail_decimation", 20);
    if (_vizTrailDecimation < 1) _vizTrailDecimation = 1;
    _vizMarkerSize = viz.getDouble("marker_size", 0.03);
  }

  ConfigTable truth;
  if (config.getTable("groundtruth", truth)) {
    _truthEnable = truth.getBool("enable", true);
    _truthReportPeriod = truth.getDouble("report_period", 2.0);
    _pathSamplePeriod = truth.getDouble("path_sample_period", _pathSamplePeriod);
    if (_pathSamplePeriod < _dt) _pathSamplePeriod = _dt;
    _pathFilterTau = truth.getDouble("path_filter_tau", _pathFilterTau);
    if (_pathFilterTau < 0.0) _pathFilterTau = 0.0;
  }
}

void MdlPosVelEstimator::activate() {
  DBGPRINT("MdlPosVelEstimator::activate\n");

  reset();

  // Resolve MdlTrot lazily because it is created after this module.
  if (!_trotSearched) {
    _trot = (MdlTrot*)_mgr->findModule(TROTMODULE_NAME, 0);
    _trotSearched = true;
  }

  _startTime = _mgr->readTime();
  _lastReportTime = _startTime;

  // Reset statistics and visualization state for this activation.
  _sumSqPos.setZero();
  _sumSqVel.setZero();
  _errorSamples = 0;
  _distanceTravelled = 0.0;
  _havePathSample = false;
  _haveFootTruth = false;
  for (int i = 0; i < NUM_LEGS; i++) {
    _contactPhase[i] = 0.0;
    _trustSum[i] = 0.0;
    _footholdErrSum[i] = 0.0;
    _footholdErrCount[i] = 0;
  }
  _trustSamples = 0;
  _trailCount = 0;
  _trailHead = 0;
  _trailTick = 0;
  _comparison = comparison_t();
}

void MdlPosVelEstimator::deactivate() {
  DBGPRINT("MdlPosVelEstimator::deactivate\n");

  // Carry the last horizontal datum into the next activation.
  if (_ready) _originCarry = _xhat.head<2>();

  if (simDebugClear && _vizEnable) simDebugClear();
}

void MdlPosVelEstimator::update() {
  if (!_enable) return;
  if (!_orientation || !_orientation->isReady()) return;

  _readSensors();
  _readContactPhase();
  _readFootTruth();

  const Eigen::Matrix3d& Rbw = _orientation->getRotation();

  if (_params.use_ground_truth && simGetGroundTruth) {
    // Substitute simulator state when the diagnostic bypass is enabled.
    double pos[3], quat[4], linvel[3], angvel[3];
    if (simGetGroundTruth(pos, quat, linvel, angvel)) {
      _xhat.head<3>() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
      _xhat.segment<3>(3) = Eigen::Vector3d(linvel[0], linvel[1], linvel[2]);
      for (int i = 0; i < NUM_LEGS; i++)
        _xhat.segment<3>(6 + 3 * i) = _xhat.head<3>() + Rbw * _footPosBody[i];
      _Rbw = Rbw;
      _vBody = Rbw.transpose() * _xhat.segment<3>(3);
      for (int i = 0; i < NUM_LEGS; i++)
        _trust[i] = trustFromPhase(_contactPhase[i], _params.trust_window);
      _ready = true;
    }
  } else {
    step(Rbw, _orientation->getAccelerationWorld(), _orientation->getAngularVelocity(),
         _footPosBody, _footVelBody, _contactPhase);
  }

  for (int i = 0; i < NUM_LEGS; i++) _trustSum[i] += _trust[i];
  _trustSamples++;

  // Compute the comparison once for statistics, logging, and visualization.
  compareWithGroundTruth(_comparison);

  _updateGroundTruthStats();
  _refreshLogBuffers();
  _updateVisualization();
}

// ---------------------------------------------------------------------------
//  Sensing
// ---------------------------------------------------------------------------

void MdlPosVelEstimator::_readSensors() {
  MotorHW* motorhw = MotorHW::instance();
  if (!motorhw) return;

  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int j = 0; j < 3; j++) {
      // Motors are indexed by 3*leg + joint.
      const int idx = 3 * leg + j;
      MotorHW::state_t state;
      motorhw->getState(idx, state);
      _jointAngles[leg][j] = state.pos;
      _jointVel[leg][j] = state.vel;
    }

    // Use unchecked FK so sensor samples outside limits remain usable.
    _kinematics->forwardKinematicsUnchecked(leg, _jointAngles[leg], _footPosBody[leg]);
    _kinematics->jacobian(leg, _jointAngles[leg], _footJacobian[leg]);

    // Compute body-frame foot velocity from the joint Jacobian.
    _footVelBody[leg] = _footJacobian[leg] * _jointVel[leg];
  }
}

void MdlPosVelEstimator::_readContactPhase() {
  // Use the active gait's contact phase as the trust input.
  double gaitPhase[NUM_LEGS];
  if (_trot && _trot->getState() == Module::ACTIVE && _trot->getStancePhase(gaitPhase)) {
    for (int i = 0; i < NUM_LEGS; i++) _contactPhase[i] = gaitPhase[i];
    return;
  }

  // Treat unscheduled behavior as planted at the center of the trust plateau.
  for (int i = 0; i < NUM_LEGS; i++) _contactPhase[i] = PLANTED_PHASE;
}

void MdlPosVelEstimator::_readFootTruth() {
  _haveFootTruth = false;
  if (!_truthEnable || !simGetFootPositions) return;

  double truth[NUM_LEGS][3];
  if (!simGetFootPositions(truth)) return;
  for (int i = 0; i < NUM_LEGS; i++)
    _footTruePos[i] = Eigen::Vector3d(truth[i][0], truth[i][1], truth[i][2]);
  _haveFootTruth = true;
}

// ---------------------------------------------------------------------------
//  Ground truth comparison
// ---------------------------------------------------------------------------

bool MdlPosVelEstimator::compareWithGroundTruth(comparison_t& result) const {
  result = comparison_t();

  if (!simGetGroundTruth || !_ready) return false;

  double pos[3], quat[4], linvel[3], angvel[3];
  if (!simGetGroundTruth(pos, quat, linvel, angvel)) return false;

  result.true_position = Eigen::Vector3d(pos[0], pos[1], pos[2]);
  result.true_velocity = Eigen::Vector3d(linvel[0], linvel[1], linvel[2]);
  result.true_orientation = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
  result.true_orientation.normalize();

  // Compare the estimated body-frame origin with the simulator's free-joint origin.
  result.position_error = _xhat.head<3>() - result.true_position;
  result.velocity_error = _xhat.segment<3>(3) - result.true_velocity;

  result.position_error_norm = result.position_error.norm();
  result.distance_travelled = _distanceTravelled;
  // Use the filtered path endpoint for net horizontal displacement.
  result.net_displacement =
      _havePathSample ? (_pathFilt2 - _startTruePos).head<2>().norm() : 0.0;
  // Report drift only after the minimum travel distance is exceeded.
  result.drift_percent = _distanceTravelled > MIN_DRIFT_DISTANCE
                             ? 100.0 * result.position_error_norm / _distanceTravelled
                             : 0.0;
  result.elapsed = _mgr ? _mgr->readTime() - _startTime : 0.0;

  // Report foothold error only for legs currently trusted; negative means unavailable.
  if (_haveFootTruth)
    for (int i = 0; i < NUM_LEGS; i++)
      result.foothold_error[i] =
          _trust[i] > 0.5 ? (_xhat.segment<3>(6 + 3 * i) - _footTruePos[i]).norm() : -1.0;

  if (_errorSamples > 0) {
    const double n = (double)_errorSamples;
    result.position_rms = (_sumSqPos / n).cwiseSqrt();
    result.velocity_rms = (_sumSqVel / n).cwiseSqrt();
  }

  result.valid = true;
  return true;
}

void MdlPosVelEstimator::_updateGroundTruthStats() {
  if (!_truthEnable || !_comparison.valid) return;

  const comparison_t& s = _comparison;

  _sumSqPos += s.position_error.cwiseProduct(s.position_error);
  _sumSqVel += s.velocity_error.cwiseProduct(s.velocity_error);
  _errorSamples++;

  for (int i = 0; i < NUM_LEGS; i++)
    if (s.foothold_error[i] >= 0.0) {
      _footholdErrSum[i] += s.foothold_error[i];
      _footholdErrCount[i]++;
    }

  const double now = _mgr->readTime();

  // Accumulate a low-pass-filtered horizontal path at the configured sample period.
  if (!_havePathSample) {
    _startTruePos = s.true_position;
    _pathFilt1 = s.true_position;
    _pathFilt2 = s.true_position;
    _lastPathSample = s.true_position;
    _lastPathTime = now;
    _havePathSample = true;
  } else {
    const double a = _dt / (_pathFilterTau + _dt);
    _pathFilt1 += a * (s.true_position - _pathFilt1);
    _pathFilt2 += a * (_pathFilt1 - _pathFilt2);

    if (now - _lastPathTime >= _pathSamplePeriod) {
      _distanceTravelled += (_pathFilt2 - _lastPathSample).head<2>().norm();
      _lastPathSample = _pathFilt2;
      _lastPathTime = now;
    }
  }

  if (_truthReportPeriod > 0.0 && now - _lastReportTime >= _truthReportPeriod) {
    _lastReportTime = now;

    // Recompute so this report includes the sample just added.
    comparison_t c;
    compareWithGroundTruth(c);

    // Aggregate trust and foothold error over the report interval before printing.
    double meanTrust[NUM_LEGS] = {0, 0, 0, 0};
    if (_trustSamples > 0)
      for (int i = 0; i < NUM_LEGS; i++)
        meanTrust[i] = _trustSum[i] / (double)_trustSamples;

    // Average foothold error over samples where each foot was trusted.
    double footErr[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; i++)
      footErr[i] = _footholdErrCount[i] > 0
                       ? _footholdErrSum[i] / (double)_footholdErrCount[i]
                       : -1.0;

    char footErrStr[128];
    int n = snprintf(footErrStr, sizeof(footErrStr), "foot pos err=[");
    for (int i = 0; i < NUM_LEGS; i++)
      n += footErr[i] >= 0.0
               ? snprintf(footErrStr + n, sizeof(footErrStr) - n, "%.4f%s", footErr[i],
                          i < NUM_LEGS - 1 ? " " : "")
               : snprintf(footErrStr + n, sizeof(footErrStr) - n, "  --  %s",
                          i < NUM_LEGS - 1 ? " " : "");
    snprintf(footErrStr + n, sizeof(footErrStr) - n, "]m");

    _mgr->message(
        "%s t=%.1fs trust=[%.2f %.2f %.2f %.2f] |pos err|=%.4fm (drift %.1f%% of "
        "%.2fm path, net %.2fm) vel rms=[%.4f %.4f %.4f]m/s %s\n",
        POSVELMODULE_NAME, c.elapsed, meanTrust[0], meanTrust[1], meanTrust[2],
        meanTrust[3], c.position_error_norm, c.drift_percent, c.distance_travelled,
        c.net_displacement, c.velocity_rms.x(), c.velocity_rms.y(), c.velocity_rms.z(),
        footErrStr);

    for (int i = 0; i < NUM_LEGS; i++) {
      _trustSum[i] = 0.0;
      _footholdErrSum[i] = 0.0;
      _footholdErrCount[i] = 0;
    }
    _trustSamples = 0;
  }
}

void MdlPosVelEstimator::_refreshLogBuffers() {
  const Eigen::Vector3d r = _xhat.head<3>();
  const Eigen::Vector3d v = _xhat.segment<3>(3);

  for (int i = 0; i < 3; i++) {
    _logState[i] = r[i];
    _logState[3 + i] = v[i];
    _logState[6 + i] = _vBody[i];
  }

  for (int i = 0; i < NUM_LEGS; i++) {
    const Eigen::Vector3d p = _xhat.segment<3>(6 + 3 * i);
    _logFootholds[3 * i + 0] = p.x();
    _logFootholds[3 * i + 1] = p.y();
    _logFootholds[3 * i + 2] = p.z();
    // Log the trust weight applied to this foot.
    _logContacts[i] = _trust[i];

    for (int j = 0; j < 3; j++)
      _logFootTruth[3 * i + j] = _haveFootTruth ? _footTruePos[i][j] : 0.0;
  }

  for (int i = 0; i < DIM; i++) _logCov[i] = _P(i, i);

  if (_comparison.valid) {
    const comparison_t& c = _comparison;
    _logError[0] = c.position_error.x();
    _logError[1] = c.position_error.y();
    _logError[2] = c.position_error.z();
    _logError[3] = c.velocity_error.x();
    _logError[4] = c.velocity_error.y();
    _logError[5] = c.velocity_error.z();
    _logError[6] = c.drift_percent;
    _logError[7] = c.distance_travelled;
    _logError[8] = c.net_displacement;

    // Preserve negative unavailable values so plots show gaps for untrusted feet.
    for (int i = 0; i < NUM_LEGS; i++) _logFootErr[i] = c.foothold_error[i];
  }
}

// ---------------------------------------------------------------------------
//  Visualization
// ---------------------------------------------------------------------------

void MdlPosVelEstimator::_updateVisualization() {
  if (!_vizEnable || !simDebugClear || !simDebugSphere) return;

  // Per-leg colours, in FL, FR, RL, RR order.
  static const double legColor[4][4] = {{1.0, 0.55, 0.1, 1.0},
                                        {0.2, 0.8, 1.0, 1.0},
                                        {0.9, 0.3, 0.9, 1.0},
                                        {0.5, 1.0, 0.3, 1.0}};
  static const double estColor[4] = {1.0, 0.15, 0.15, 1.0};
  static const double truthColor[4] = {0.15, 0.4, 1.0, 1.0};
  static const double errColor[4] = {1.0, 1.0, 0.2, 1.0};

  simDebugClear();

  const Eigen::Vector3d r = _xhat.head<3>();
  const double est[3] = {r.x(), r.y(), r.z()};
  simDebugSphere(est, _vizMarkerSize, estColor);

  // Draw ground truth and the position-error vector when available.
  const comparison_t& c = _comparison;
  const bool haveTruth = c.valid;
  if (haveTruth) {
    const double truth[3] = {c.true_position.x(), c.true_position.y(),
                             c.true_position.z()};
    simDebugSphere(truth, _vizMarkerSize * 0.8, truthColor);
    if (simDebugLine && c.position_error_norm > 1e-4)
      simDebugLine(est, truth, _vizMarkerSize * 0.25, errColor);
  }

  // Size and fade estimated foot markers by their applied contact trust.
  for (int i = 0; i < NUM_LEGS; i++) {
    const Eigen::Vector3d p = _xhat.segment<3>(6 + 3 * i);
    const double pos[3] = {p.x(), p.y(), p.z()};
    double rgba[4] = {legColor[i][0], legColor[i][1], legColor[i][2],
                      0.3 + 0.7 * _trust[i]};
    simDebugSphere(pos, _vizMarkerSize * (0.45 + 0.3 * _trust[i]), rgba);
  }

  // Draw decimated estimated and true path trails.
  if (_vizTrail && _vizTrailLength > 0) {
    if (++_trailTick >= _vizTrailDecimation) {
      _trailTick = 0;
      _trailEst[_trailHead] = r;
      _trailTrue[_trailHead] = haveTruth ? c.true_position : r;
      _trailHead = (_trailHead + 1) % _vizTrailLength;
      if (_trailCount < _vizTrailLength) _trailCount++;
    }

    for (int i = 0; i < _trailCount; i++) {
      const int slot = (_trailHead - 1 - i + 2 * _vizTrailLength) % _vizTrailLength;
      // Fade older trail samples.
      const double alpha = 0.9 * (1.0 - (double)i / (double)_trailCount);

      const double pe[3] = {_trailEst[slot].x(), _trailEst[slot].y(),
                            _trailEst[slot].z()};
      double ce[4] = {estColor[0], estColor[1], estColor[2], alpha};
      simDebugSphere(pe, _vizMarkerSize * 0.25, ce);

      if (haveTruth) {
        const double pt[3] = {_trailTrue[slot].x(), _trailTrue[slot].y(),
                              _trailTrue[slot].z()};
        double ct[4] = {truthColor[0], truthColor[1], truthColor[2], alpha};
        simDebugSphere(pt, _vizMarkerSize * 0.25, ct);
      }
    }
  }
}

// ---------------------------------------------------------------------------
//  Estimate accessors
// ---------------------------------------------------------------------------

bool MdlPosVelEstimator::getBodyPosition(Eigen::Vector3d& pos) const {
  if (!_ready) return false;
  pos = _xhat.head<3>();
  return true;
}

bool MdlPosVelEstimator::getBodyVelocity(Eigen::Vector3d& vel) const {
  if (!_ready) return false;
  vel = _xhat.segment<3>(3);
  return true;
}

bool MdlPosVelEstimator::getBodyVelocityInBody(Eigen::Vector3d& vel) const {
  if (!_ready) return false;
  vel = _vBody;
  return true;
}

bool MdlPosVelEstimator::getFootPosition(int leg, Eigen::Vector3d& pos) const {
  if (!_ready || leg < 0 || leg >= NUM_LEGS) return false;
  pos = _xhat.segment<3>(6 + 3 * leg);
  return true;
}

double MdlPosVelEstimator::getContactTrust(int leg) const {
  if (leg < 0 || leg >= NUM_LEGS) return 0.0;
  return _trust[leg];
}
