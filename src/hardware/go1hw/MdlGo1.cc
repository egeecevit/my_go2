#include "MdlGo1.hh"
#include "rtcore/ModuleManager.hh"
#include "quadruped/MdlLegControl.hh"
#include <cmath>

using namespace rtcore;
using namespace UNITREE_LEGGED_SDK;

#define DBGPRINT(...) printf(__VA_ARGS__)

MdlGo1::MdlGo1() : Module(GO1MODULE_NAME, 0, SINGLE_USER), _safe(LeggedType::Go1), _udp(LOWLEVEL, 8090, "192.168.123.10", 8007) {
  DBGPRINT("MdlGo1::MdlGo1\n");
}

MdlGo1::~MdlGo1() {
  DBGPRINT("MdlGo1::~MdlGo1\n");
}

void MdlGo1::init() {
  DBGPRINT("MdlGo1::init\n");
  _udp.InitCmdData(_cmd);
  _leg = (MdlLegControl*)_mgr->findModule(LEGMODULE_NAME, 0);
}
void MdlGo1::uninit() {
  DBGPRINT("MdlGo1::uninit\n");
}
void MdlGo1::activate() {
  DBGPRINT("MdlGo1::activate\n");
  _mgr->grabModule(_leg, this);
}
void MdlGo1::deactivate() {
  DBGPRINT("MdlGo1::deactivate\n");
  _mgr->releaseModule(_leg, this);
}

void MdlGo1::PDControl(double t) {
  _udp.Recv();
  _udp.GetRecv(_state);

  _cmd.motorCmd[FR_0].tau = -0.65f;
  _cmd.motorCmd[FL_0].tau = +0.65f;
  _cmd.motorCmd[RR_0].tau = -0.65f;
  _cmd.motorCmd[RL_0].tau = +0.65f;

  const double A     = 0.4;   // amplitude [rad]
  const double f     = 0.2;   // frequency [Hz]
  const double q0    = 0.0;  // center position offset [rad]

  double q_ref  = q0 + A * sin(2.0 * M_PI * f * t);
  double dq_ref =      A * 2.0 * M_PI * f * cos(2.0 * M_PI * f * t);

  // TODO: tune Kp and Kd for your test conditions
  _cmd.motorCmd[FR_1].q   = (float)q_ref;
  _cmd.motorCmd[FR_1].dq  = (float)dq_ref;
  _cmd.motorCmd[FR_1].Kp  = 20.0f;
  _cmd.motorCmd[FR_1].Kd  = 0.5f;
  _cmd.motorCmd[FR_1].tau = 0.0f;

  Eigen::Vector3d pos(0.0, q_ref, 0.0);
  Eigen::Vector3d vel(0.0, dq_ref, 0.0);

  _leg->setTargetAngles(pos, vel);
  
  int res = _safe.PowerProtect(_cmd, _state, 1);
  if (res < 0) _mgr->fatalError("MdlGo1", "Power Protect Triggered!");

  _udp.SetSend(_cmd);
  _udp.Send();
}

void MdlGo1::TorqueControl(double t) {
  _udp.Recv();
  _udp.GetRecv(_state);
  fprintf(stderr, "t = %.3f\n", t);  // add this temporarily

_cmd.motorCmd[FR_0].tau = -0.65f;
_cmd.motorCmd[FL_0].tau = +0.65f;
_cmd.motorCmd[RR_0].tau = -0.65f;
_cmd.motorCmd[RL_0].tau = +0.65f;

  const double A     = 0.4;   // amplitude [rad]
  const double f     = 0.2;   // frequency [Hz]
  const double q0    = 0.0;  // center position offset [rad]

  double q_ref  = q0 + A * sin(2.0 * M_PI * f * (t-3.0));
  double dq_ref =      A * 2.0 * M_PI * f * cos(2.0 * M_PI * f * (t-3.0));

  double torque = (q_ref - _state.motorState[FR_1].q) * 10.0f + (dq_ref - _state.motorState[FR_1].dq) * 5.0f;

  // TODO: tune Kp and Kd for your test conditions
  _cmd.motorCmd[FR_1].q   = PosStopF;
  _cmd.motorCmd[FR_1].dq  = VelStopF;
  _cmd.motorCmd[FR_1].Kp  = 0.0f;
  _cmd.motorCmd[FR_1].Kd  = 0.0f;
  _cmd.motorCmd[FR_1].tau = torque;

  Eigen::Vector3d pos(0.0, q_ref, 0.0);
  Eigen::Vector3d vel(0.0, dq_ref, 0.0);

  _leg->setTargetAngles(pos, vel);
  
  int res = _safe.PowerProtect(_cmd, _state, 1);
  if (res < 0) _mgr->fatalError("MdlGo1", "Power Protect Triggered!");

  _udp.SetSend(_cmd);
  _udp.Send();
}

void MdlGo1::update() {
  _motiontime = _mgr->readTime();
  // PDControl(_motiontime);
  TorqueControl(_motiontime);
}