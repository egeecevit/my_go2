/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "MdlSimDriver.hh"

#include <GLFW/glfw3.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjui.h>
#include <unistd.h>

#include <cmath>

#include "SimClockHW.hh"
#include "rtcore/ModuleManager.hh"
#include "rtcore/quaternion.h"

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...)  // printf(__VA_ARGS__)

using namespace rtcore;

// Define static members
MdlSimDriver* MdlSimDriver::_instance = nullptr;

static void _mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  MdlSimDriver* driver = static_cast<MdlSimDriver*>(glfwGetWindowUserPointer(window));
  if (driver) driver->_handleMouseButton(button, action, mods);
}

void MdlSimDriver::_handleMouseButton(int button, int action, int mods) {
  // Update button state
  if (action == GLFW_PRESS) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) _button_left = true;
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) _button_middle = true;
    if (button == GLFW_MOUSE_BUTTON_RIGHT) _button_right = true;
  } else if (action == GLFW_RELEASE) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) _button_left = false;
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) _button_middle = false;
    if (button == GLFW_MOUSE_BUTTON_RIGHT) _button_right = false;
  }
}

static void _cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
  MdlSimDriver* driver = static_cast<MdlSimDriver*>(glfwGetWindowUserPointer(window));
  if (driver) driver->_handleMouseMove(xpos, ypos);
}

void MdlSimDriver::_handleMouseMove(double xpos, double ypos) {
  if (!_window) return;

  // Calculate mouse movement
  double dx = xpos - _lastx;
  double dy = ypos - _lasty;
  _lastx = xpos;
  _lasty = ypos;

  // Get window size for proper scaling
  int width, height;
  glfwGetWindowSize(_window, &width, &height);

  // Camera rotation with left mouse button
  if (_button_left && !_button_middle && !_button_right) {
    // Fix: Reverse azimuth direction for intuitive left/right rotation
    _cam.azimuth -= 360.0 * dx / width;
    _cam.elevation -= 180.0 * dy / height;

    // Clamp elevation to reasonable bounds
    if (_cam.elevation > 89.0) _cam.elevation = 89.0;
    if (_cam.elevation < -89.0) _cam.elevation = -89.0;
  } else if (_button_right && !_button_left && !_button_middle) {
    // Camera panning with right mouse button
    double pan_scale = _cam.distance * 0.002;  // Adjusted scale factor

    // Calculate pan vectors in camera-relative coordinates
    double azimuth_rad = _cam.azimuth * M_PI / 180.0;

    // Left/right panning (perpendicular to camera direction) - Fixed direction
    double pan_x = -dx * pan_scale * sin(azimuth_rad);
    double pan_y = dx * pan_scale * cos(azimuth_rad);

    // Up/down panning (always in world Z direction) - Fixed direction
    double pan_z = dy * pan_scale;

    _cam.lookat[0] += pan_x;
    _cam.lookat[1] += pan_y;
    _cam.lookat[2] += pan_z;
  }
}

static void _scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  MdlSimDriver* driver = static_cast<MdlSimDriver*>(glfwGetWindowUserPointer(window));
  if (driver) driver->_handleMouseScroll(xoffset, yoffset);
}

// Global key buffer for cross-module keyboard polling
static int g_lastKey = -1;
int simPollKey() { int k = g_lastKey; g_lastKey = -1; return k; }

// ---------------------------------------------------------------------------
// Cross-module simulation introspection and debug drawing.
//
// These free functions exist so that target-agnostic modules living in
// libquadruped (which must also build for the go1 and robot targets, where
// MuJoCo is absent) can reach the simulation without linking against it. They
// follow the same weak-symbol convention already used for simPollKey(): callers
// declare them with __attribute__((weak)) and test the pointer before calling,
// so on non-sim targets the symbols resolve to null and the feature disappears.
//
// No locking is needed: MdlSimDriver::update(), the estimator and the behaviors
// all run sequentially inside the single ModuleManager main loop.
// ---------------------------------------------------------------------------

// The live driver, for the introspection helpers below. Set in _createSimulation().
static MdlSimDriver* g_simdriver = nullptr;

bool simGetGroundTruth(double pos[3], double quat[4], double linvel[3],
                       double angvel[3]) {
  if (!g_simdriver) return false;
  return g_simdriver->getGroundTruth(pos, quat, linvel, angvel);
}

bool simGetFootContacts(bool contacts[4], double forces[4]) {
  if (!g_simdriver) return false;
  return g_simdriver->getFootContacts(contacts, forces);
}

bool simGetFootPositions(double pos[4][3]) {
  if (!g_simdriver) return false;
  return g_simdriver->getFootPositions(pos);
}

// Debug geom queue. Fixed capacity, no allocation on the control path. Producers
// overwrite the queue every cycle (typically at 1kHz); the renderer drains
// whatever is present when it happens to run (typically at 30Hz).
#define MAX_DEBUG_GEOMS 512

typedef struct {
  int type;         // mjGEOM_SPHERE / mjGEOM_CAPSULE
  double size[3];
  double pos[3];
  double mat[9];
  float rgba[4];
} debug_geom_t;

static debug_geom_t g_debugGeoms[MAX_DEBUG_GEOMS];
static int g_numDebugGeoms = 0;

static void _pushDebugGeom(int type, const double size[3], const double pos[3],
                           const double mat[9], const double rgba[4]) {
  if (g_numDebugGeoms >= MAX_DEBUG_GEOMS) return;
  debug_geom_t& g = g_debugGeoms[g_numDebugGeoms++];
  g.type = type;
  for (int i = 0; i < 3; i++) {
    g.size[i] = size[i];
    g.pos[i] = pos[i];
  }
  for (int i = 0; i < 9; i++) g.mat[i] = mat[i];
  for (int i = 0; i < 4; i++) g.rgba[i] = (float)rgba[i];
}

static const double _identityMat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

void simDebugClear() { g_numDebugGeoms = 0; }

void simDebugSphere(const double pos[3], double radius, const double rgba[4]) {
  const double size[3] = {radius, radius, radius};
  _pushDebugGeom(mjGEOM_SPHERE, size, pos, _identityMat, rgba);
}

void simDebugLine(const double from[3], const double to[3], double width,
                  const double rgba[4]) {
  // Drawn as a capsule spanning the two endpoints rather than through
  // mjv_connector()/mjv_makeConnector(), whose name and signature have changed
  // across MuJoCo releases (this project tracks the mujoco main branch).
  double d[3] = {to[0] - from[0], to[1] - from[1], to[2] - from[2]};
  double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  if (len < 1e-9) return;

  for (int i = 0; i < 3; i++) d[i] /= len;

  // Orthonormal basis with the capsule's local z axis along d.
  double up[3] = {0, 0, 1};
  if (std::fabs(d[2]) > 0.9) {
    up[0] = 1;
    up[2] = 0;
  }
  double x[3] = {up[1] * d[2] - up[2] * d[1], up[2] * d[0] - up[0] * d[2],
                 up[0] * d[1] - up[1] * d[0]};
  double xn = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
  for (int i = 0; i < 3; i++) x[i] /= xn;
  double y[3] = {d[1] * x[2] - d[2] * x[1], d[2] * x[0] - d[0] * x[2],
                 d[0] * x[1] - d[1] * x[0]};

  // Column major in the MuJoCo sense: mat[3*row + col], columns are x, y, d.
  const double mat[9] = {x[0], y[0], d[0], x[1], y[1], d[1], x[2], y[2], d[2]};
  const double size[3] = {width, width, 0.5 * len};
  const double pos[3] = {0.5 * (from[0] + to[0]), 0.5 * (from[1] + to[1]),
                         0.5 * (from[2] + to[2])};
  _pushDebugGeom(mjGEOM_CAPSULE, size, pos, mat, rgba);
}

void MdlSimDriver::_renderDebugGeoms() {
  for (int i = 0; i < g_numDebugGeoms && _scn.ngeom < _scn.maxgeom; i++) {
    const debug_geom_t& g = g_debugGeoms[i];
    mjvGeom* target = &_scn.geoms[_scn.ngeom++];
    mjv_initGeom(target, g.type, g.size, g.pos, g.mat, g.rgba);
    // Debug markers are annotations, not part of the model: exclude them from
    // MuJoCo's own category filtering so they are always drawn.
    target->category = mjCAT_DECOR;
    target->objtype = mjOBJ_UNKNOWN;
    target->objid = -1;
    target->segid = -1;
  }
}

bool MdlSimDriver::getGroundTruth(double pos[3], double quat[4], double linvel[3],
                                  double angvel[3]) {
  // Requires a free joint at the root: qpos[0:3] position, qpos[3:7] orientation
  // (w,x,y,z), qvel[0:3] world-frame linear velocity, qvel[3:6] body-frame
  // angular velocity.
  if (!_model || !_data || _model->nq < 7 || _model->nv < 6) return false;

  if (pos)
    for (int i = 0; i < 3; i++) pos[i] = _data->qpos[i];
  if (quat)
    for (int i = 0; i < 4; i++) quat[i] = _data->qpos[3 + i];
  if (linvel)
    for (int i = 0; i < 3; i++) linvel[i] = _data->qvel[i];
  if (angvel)
    for (int i = 0; i < 3; i++) angvel[i] = _data->qvel[3 + i];

  return true;
}

bool MdlSimDriver::getFootContacts(bool contacts[4], double forces[4]) {
  if (!_model || !_data) return false;

  double normal[4] = {0.0, 0.0, 0.0, 0.0};

  bool resolved = false;
  for (int i = 0; i < 4; i++)
    if (_footGeomId[i] >= 0) resolved = true;
  if (!resolved) return false;

  for (int c = 0; c < _data->ncon; c++) {
    const mjContact& con = _data->contact[c];

    // The geom class in go2.xml carries margin="0.001", so MuJoCo lists a
    // contact as soon as the gap closes to a millimetre. Those records are real
    // but carry no load, and a foot skimming through that shell on its way
    // through swing would otherwise be reported as planted. Positive dist means
    // still separated; a nonzero exclude means the record is not in the
    // constraint set at all.
    if (con.dist > 0.0 || con.exclude != 0) continue;

    mjtNum wrench[6];
    mj_contactForce(_model, _data, c, wrench);
    // Contact frame, normal first, and always non-negative by construction.
    const double f = wrench[0];
    if (f <= 0.0) continue;

    for (int i = 0; i < 4; i++) {
      if (_footGeomId[i] >= 0 &&
          (con.geom1 == _footGeomId[i] || con.geom2 == _footGeomId[i]))
        normal[i] += f;
    }
  }

  for (int i = 0; i < 4; i++) {
    if (contacts) contacts[i] = normal[i] > 0.0;
    if (forces) forces[i] = normal[i];
  }
  return true;
}

bool MdlSimDriver::getFootPositions(double pos[4][3]) {
  if (!_model || !_data || !pos) return false;

  bool resolved = false;
  for (int i = 0; i < 4; i++) {
    if (_footGeomId[i] < 0) {
      pos[i][0] = pos[i][1] = pos[i][2] = 0.0;
      continue;
    }
    resolved = true;
    for (int j = 0; j < 3; j++) pos[i][j] = _data->geom_xpos[3 * _footGeomId[i] + j];
  }
  return resolved;
}

// Static callback function for GLFW keyboard interaction
static void _keyCallback(GLFWwindow* window, int key, int scancode, int action,
                         int mods) {
  MdlSimDriver* driver = static_cast<MdlSimDriver*>(glfwGetWindowUserPointer(window));
  if (driver) driver->_handleKeyboard(key, scancode, action, mods);
}

void MdlSimDriver::_handleKeyboard(int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    if (key >= GLFW_KEY_A && key <= GLFW_KEY_Z)
      g_lastKey = 'a' + (key - GLFW_KEY_A);
    else if (key == GLFW_KEY_ESCAPE)
      g_lastKey = 'q';
  }
}

void MdlSimDriver::_handleMouseScroll(double xoffset, double yoffset) {
  // Zoom in/out with mouse wheel - scroll up to zoom in
  double zoom_factor = 1.0 - 0.1 * yoffset;
  _cam.distance *= zoom_factor;

  // Clamp distance to reasonable bounds
  if (_cam.distance < 0.1) _cam.distance = 0.1;
  if (_cam.distance > 50.0) _cam.distance = 50.0;
}

MdlSimDriver::MdlSimDriver() : Module(MDLSIMDRIVER_NAME, 0, SINGLE_USER) {
  DBGPRINT("MdlSimDriver::MdlSimDriver\n");
}

MdlSimDriver::~MdlSimDriver() { DBGPRINT("MdlSimDriver::~MdlSimDriver\n"); }

void MdlSimDriver::init() {
  DBGPRINT("MdlSimDriver::init\n");
  _createSimulation();
}

void MdlSimDriver::uninit() {
  DBGPRINT("MdlSimDriver::uninit\n");
  _destroySimulation();
}

void MdlSimDriver::activate() { DBGPRINT("MdlSimDriver::activate\n"); }

void MdlSimDriver::deactivate() { DBGPRINT("MdlSimDriver::deactivate\n"); }

class MujocoStateAccessor : public LogAccessor {
 public:
  MujocoStateAccessor(mjModel* model, mjData* data) : LogAccessor(LOG_DOUBLE, model->nq) {
    _mjmodel = model;
    _mjdata = data;
  }

 protected:
  mjModel* _mjmodel;
  mjData* _mjdata;
};

class MujocoConfigAccessor : public MujocoStateAccessor {
 public:
  MujocoConfigAccessor(mjModel* model, mjData* data) : MujocoStateAccessor(model, data) {}
  virtual ~MujocoConfigAccessor() {}

  void getVar(int size, unsigned char* p) override {
    memcpy(p, _mjdata->qpos, _mjmodel->nq * sizeof(double));
  }
};

class MujocoVelAccessor : public MujocoStateAccessor {
 public:
  MujocoVelAccessor(mjModel* model, mjData* data) : MujocoStateAccessor(model, data) {}
  virtual ~MujocoVelAccessor() {}

  void getVar(int size, unsigned char* p) override {
    // Warning: This assumes that mjtNum is double
    memcpy(p, _mjdata->qvel, _mjmodel->nq * sizeof(double));
  }
};

class MujocoAccelAccessor : public MujocoStateAccessor {
 public:
  MujocoAccelAccessor(mjModel* model, mjData* data) : MujocoStateAccessor(model, data) {}
  virtual ~MujocoAccelAccessor() {}

  void getVar(int size, unsigned char* p) override {
    // Warning: This assumes that mjtNum is double
    memcpy(p, _mjdata->qacc, _mjmodel->nq * sizeof(double));
  }
};

class MujocoControlAccessor : public MujocoStateAccessor {
 public:
  MujocoControlAccessor(mjModel* model, mjData* data) : MujocoStateAccessor(model, data) {
    _array_size = model->nu;
    _type = LOG_DOUBLE;
  }
  virtual ~MujocoControlAccessor() {}

  void getVar(int size, unsigned char* p) override {
    // Warning: This assumes that mjtNum is double
    memcpy(p, _mjdata->ctrl, _mjmodel->nu * sizeof(double));
  }
};

void MdlSimDriver::update() {
  // DBGPRINT("MdlSimDriver::update %lf\n", _mgr->readTime());

  if (!_logserver) {
    _logserver = (LogServer*)_mgr->findModule(LOGSERVER_NAME, 0);
    _qpos_a = new MujocoConfigAccessor(_model, _data);
    _logserver->registerVar("MdlSimDriver", "qpos", _qpos_a);
    _qvel_a = new MujocoVelAccessor(_model, _data);
    _logserver->registerVar("MdlSimDriver", "qvel", _qvel_a);
    _qacc_a = new MujocoAccelAccessor(_model, _data);
    _logserver->registerVar("MdlSimDriver", "qacc", _qacc_a);
    _ctrl_a = new MujocoControlAccessor(_model, _data);
    _logserver->registerVar("MdlSimDriver", "ctrl", _ctrl_a);
  }

  _integrate(CLOCK_TO_SEC(_mgr->getStepPeriod()));

  double sim_time = _data ? _data->time : 0.0;
  _now = sim_time * 1000000;  // Update current simulation time

  if (!_headless && _window && glfwWindowShouldClose(_window) != 0)
    _mgr->exitMainLoop();
  
  // Render the simulation (only if not in headless mode)
  if (!_headless && _window && !glfwWindowShouldClose(_window) &&
      _now - _last_render_time > _render_interval_us) {
    _last_render_time += _render_interval_us;

    // Get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(_window, &viewport.width, &viewport.height);

    // Update scene and render. Debug geoms are appended after mjv_updateScene(),
    // which resets scn.ngeom, and before mjr_render().
    mjv_updateScene(_model, _data, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
    _renderDebugGeoms();
    mjr_render(viewport, &_scn, &_con);

    SimClockHW* clockhw = (SimClockHW*)ClockHW::instance();
    // Render simulation time overlay at the top of the window
    char time_text[100];
    snprintf(time_text, sizeof(time_text), "tsim = %.3f s", sim_time);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, time_text, NULL, &_con);

    snprintf(time_text, sizeof(time_text), "treal = %.1f s",
             clockhw ? ((double)clockhw->readRealClock() / 1000000.0) : 0);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, time_text, NULL, &_con);

    // Swap front and back buffers
    glfwSwapBuffers(_window);

    // Poll for and process events
    glfwPollEvents();
  }

  _readJointStates();
  _readIMUData();
  // usleep(1000);
}

void MdlSimDriver::getJointState(unsigned int ind, MotorHW::state_t& state) {
  state = _state[ind];
}

void MdlSimDriver::setJointCommand(unsigned int ind, MotorHW::cmd_t& cmd) {
  _cmd[ind] = cmd;
}

void MdlSimDriver::getJointCommand(unsigned int ind, MotorHW::cmd_t& cmd) {
  cmd = _cmd[ind];
}

bool MdlSimDriver::getIMUData(IMUHW::imudata_t& imu) {
  if (imu.t == _imu.t) return false;
  imu = _imu;
  return true;
}

void MdlSimDriver::_createSimulation() {
  DBGPRINT("MdlSimDriver::_createSimulation\n");

  // Load MuJoCo model
  char error[1000] = "Could not load binary model";
  std::string model_path = "~/quadcontrol/models/unitree_go2/scene.xml";

  // Read simulation configuration from TOML
  ConfigTable simConfig;
  bool hasSimConfig = _mgr->getConfigTable("simulation", simConfig);
  DBGPRINT("MdlSimDriver: Simulation config %s\n", hasSimConfig ? "found" : "not found");

  double gravity[3] = {0.0, 0.0, -9.81};
  double initial_position[3] = {0.0, 0.0, 0.5};
  double initial_roll = 0.0, initial_pitch = 0.0, initial_yaw = 0.0;
  double framerate = 10.0;

  if (hasSimConfig) {
    // Check if the model path is specified in the configuration
    std::string config_path = simConfig.getString("model_path");
    if (config_path.size() > 0) {
      model_path = config_path;
      DBGPRINT("MdlSimDriver: Using configured model path: %s\n", model_path);
    } else {
      DBGPRINT("MdlSimDriver: Using default model path: %s\n", model_path);
    }
  }

  _model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  if (!_model) _mgr->fatalError("MdlSimDriver", "Load model error: %s", error);

  _data = mj_makeData(_model);

  // Try to load "home" keyframe as default initial state
  bool keyframeLoaded = false;
  for (int i = 0; i < _model->nkey; i++) {
    const char *name = mj_id2name(_model, mjOBJ_KEY, i);
    if (name && std::string(name) == "home") {
      mj_resetDataKeyframe(_model, _data, i);
      keyframeLoaded = true;
      break;
    }
  }
  if (!keyframeLoaded) {
    // Go2 resting pose: belly on ground, legs splayed
    _data->qpos[2] = 0.077;
    // FL
    _data->qpos[7] = 0.18;  _data->qpos[8] = 1.22;  _data->qpos[9] = -2.70;
    // FR
    _data->qpos[10] = -0.18; _data->qpos[11] = 1.22; _data->qpos[12] = -2.70;
    // RL
    _data->qpos[13] = 0.48;  _data->qpos[14] = 1.25; _data->qpos[15] = -2.72;
    // RR
    _data->qpos[16] = -0.48; _data->qpos[17] = 1.25; _data->qpos[18] = -2.72;
  }

  bool hasExplicitPos = false;
  bool hasExplicitOri = false;

  if (hasSimConfig) {
    ConfigArray gravityArray;
    if (simConfig.getArray("gravity", gravityArray)) {
      if (gravityArray.size() >= 3) {
        gravity[0] = gravityArray.getDoubleAt(0, gravity[0]);
        gravity[1] = gravityArray.getDoubleAt(1, gravity[1]);
        gravity[2] = gravityArray.getDoubleAt(2, gravity[2]);
        DBGPRINT("MdlSimDriver: Using configured gravity: [%.3f, %.3f, %.3f]\n",
                 gravity[0], gravity[1], gravity[2]);
      }
    }

    ConfigArray posArray;
    if (simConfig.getArray("initial_position", posArray) && posArray.size() >= 3) {
      initial_position[0] = posArray.getDoubleAt(0, initial_position[0]);
      initial_position[1] = posArray.getDoubleAt(1, initial_position[1]);
      initial_position[2] = posArray.getDoubleAt(2, initial_position[2]);
      hasExplicitPos = true;
      DBGPRINT("MdlSimDriver: Using configured initial position: [%.3f, %.3f, %.3f]\n",
               initial_position[0], initial_position[1], initial_position[2]);
    }

    // Only override orientation if explicitly configured
    ConfigTable oriTable;
    if (simConfig.getDouble("initial_roll", initial_roll) ||
        simConfig.getDouble("initial_pitch", initial_pitch) ||
        simConfig.getDouble("initial_yaw", initial_yaw)) {
      hasExplicitOri = true;
    }

    framerate = simConfig.getDouble("framerate", framerate);
    _headless = simConfig.getBool("headless", _headless);
  }

  // Store configured gravity values for use in _integrate()
  _configured_gravity[0] = gravity[0];
  _configured_gravity[1] = gravity[1];
  _configured_gravity[2] = gravity[2];

  _render_interval_us = (CLOCK)(1000000.0 / framerate);

  // Only override base pose if explicitly configured (otherwise keep keyframe values)
  if (hasExplicitPos) {
    _data->qpos[0] = initial_position[0];
    _data->qpos[1] = initial_position[1];
    _data->qpos[2] = initial_position[2];
  }

  if (hasExplicitOri) {
    double pitch_rad = initial_pitch * M_PI / 180.0;
    double roll_rad = initial_roll * M_PI / 180.0;
    double yaw_rad = initial_yaw * M_PI / 180.0;

    double cp = cos(pitch_rad / 2.0), sp = sin(pitch_rad / 2.0);
    double cr = cos(roll_rad / 2.0), sr = sin(roll_rad / 2.0);
    double cy = cos(yaw_rad / 2.0), sy = sin(yaw_rad / 2.0);

    _data->qpos[3] = cr * cp * cy + sr * sp * sy;
    _data->qpos[4] = sr * cp * cy - cr * sp * sy;
    _data->qpos[5] = cr * sp * cy + sr * cp * sy;
    _data->qpos[6] = cr * cp * sy - sr * sp * cy;
  }

  // Apply gravity configuration to model
  _model->opt.gravity[0] = gravity[0];
  _model->opt.gravity[1] = gravity[1];
  _model->opt.gravity[2] = gravity[2];

  // Forward the simulation state to ensure consistency
  mj_forward(_model, _data);

  // Populate _state[] so getJointState works before first update
  _readJointStates();

  // Initialize GLFW and create window only if not in headless mode
  if (!_headless) {
    // Initialize GLFW
    if (!glfwInit()) _mgr->fatalError("MdlSimDriver", "Could not initialize GLFW");

    // Create window
    _window = glfwCreateWindow(640, 480, "MuJoCo Simulation", NULL, NULL);
    if (!_window) {
      glfwTerminate();
      _mgr->fatalError("MdlSimDriver", "Could not create GLFW window");
    }
    _last_render_time = 0;

    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    // Initialize MuJoCo visualization data structures
    mjv_defaultCamera(&_cam);
    mjv_defaultOption(&_opt);
    mjv_defaultScene(&_scn);
    mjr_defaultContext(&_con);

    // Create scene and context
    mjv_makeScene(_model, &_scn, 2000);  // max geoms
    mjr_makeContext(_model, &_con, mjFONTSCALE_150);

    // Initialize camera position
    _cam.azimuth = 90;
    _cam.elevation = -20;
    _cam.distance = 2.0;
    _cam.lookat[0] = 0;
    _cam.lookat[1] = 0;
    _cam.lookat[2] = 0.2;

    // Follow the floating robot base while keeping azimuth, elevation, and
    // distance under the existing mouse controls.
    for (int joint = 0; joint < _model->njnt; joint++) {
      if (_model->jnt_type[joint] == mjJNT_FREE) {
        _cam.type = mjCAMERA_TRACKING;
        _cam.trackbodyid = _model->jnt_bodyid[joint];
        break;
      }
    }

    // Set the user pointer for the window to this instance
    glfwSetWindowUserPointer(_window, this);

    // Set GLFW callback functions for mouse interactions
    glfwSetMouseButtonCallback(_window, _mouseButtonCallback);
    glfwSetCursorPosCallback(_window, _cursorPosCallback);
    glfwSetScrollCallback(_window, _scrollCallback);
    glfwSetKeyCallback(_window, _keyCallback);  // Set keyboard callback

    DBGPRINT("MdlSimDriver: Visualization window created successfully\n");
  } else {
    DBGPRINT(
        "MdlSimDriver: Running in headless mode - no visualization window created\n");
    _window = nullptr;
  }

  _instance = this;  // Set static instance pointer for controller callback
  g_simdriver = this;  // Set file-static pointer for the simGet*() helpers

  // Resolve foot geom ids once, in QuadrupedKinematics::LegIndex order
  // (FL, FR, RL, RR). Looked up by name so the mapping is independent of the
  // order in which bodies happen to be declared in the model.
  static const char* footGeomNames[4] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; _model && i < 4; i++) {
    _footGeomId[i] = mj_name2id(_model, mjOBJ_GEOM, footGeomNames[i]);
    if (_footGeomId[i] < 0)
      _mgr->warning("MdlSimDriver", "No geom named '%s'; contact truth unavailable",
                    footGeomNames[i]);
  }

  // Register MuJoCo controller callback
  mjcb_control = _controllerCallback_static;
}

void MdlSimDriver::_destroySimulation() {
  DBGPRINT("MdlSimDriver::_destroySimulation\n");

  g_simdriver = nullptr;
  g_numDebugGeoms = 0;

  // Clean up MuJoCo data and model objects
  if (_data) {
    mj_deleteData(_data);
    _data = nullptr;
  }

  if (_model) {
    mj_deleteModel(_model);
    _model = nullptr;
  }

  if (!_headless && _window) {
    mjr_freeContext(&_con);
    mjv_freeScene(&_scn);
    glfwDestroyWindow(_window);
    _window = nullptr;
    glfwTerminate();
  }
}

void MdlSimDriver::_integrate(double dt) {
  // Forward integrate simulation for dt seconds (in simulation time)
  if (_model && _data) {
    // Apply gravity setting before integration
    if (_gravityFlag) {
      // Use configured gravity values from TOML
      _model->opt.gravity[0] = _configured_gravity[0];
      _model->opt.gravity[1] = _configured_gravity[1];
      _model->opt.gravity[2] = _configured_gravity[2];
    } else {
      // Disable gravity by setting all components to zero
      _model->opt.gravity[0] = 0.0;
      _model->opt.gravity[1] = 0.0;
      _model->opt.gravity[2] = 0.0;
    }

    // Calculate how many simulation steps we need to advance by dt
    double current_time = _data->time;
    double target_time = current_time + dt;

    // Step the simulation until we reach the target time
    while (_data->time < target_time) {
      // Check if we need a smaller timestep for the final step
      double remaining_time = target_time - _data->time;
      if (remaining_time < _model->opt.timestep) {
        // Save original timestep
        double original_timestep = _model->opt.timestep;
        // Set smaller timestep for final step
        _model->opt.timestep = remaining_time;
        // DBGPRINT("MdlSimDriver: Adjusting timestep to %.3f seconds for final step\n",
        // remaining_time);
        mj_step(_model, _data);
        // Restore original timestep
        _model->opt.timestep = original_timestep;
        break;
      } else {
        // DBGPRINT("MdlSimDriver: Stepping simulation by %.3f seconds\n",
        // _model->opt.timestep);
        mj_step(_model, _data);
      }
    }
  }
}

void MdlSimDriver::_controllerCallback_static(const mjModel* m, mjData* d) {
  if (_instance) _instance->_controllerCallback(m, d);
}

// MuJoCo controller callback implementation
void MdlSimDriver::_controllerCallback(const mjModel* m, mjData* d) {
  // This callback is called by MuJoCo during each simulation step
  // before the physics integration, allowing us to compute control inputs

  if (!m || !d) return;

  // Compute control torques using PD control
  for (int i = 0; i < m->nu; i++) {
    if (!_enabled[i]) {
      d->ctrl[i] = 0.0;  // Disable control if not enabled
      continue;
    }

    // Set default PD gains if not set
    if (_cmd[i].kp <= 0) _cmd[i].kp = 0.0;
    if (_cmd[i].kd <= 0) _cmd[i].kd = 5.0;

    // Read current actuator states directly from MuJoCo data
    double current_pos = _polarity[i] * d->actuator_length[i];
    double current_vel = _polarity[i] * d->actuator_velocity[i];

    double pos_error = current_pos - _cmd[i].pos;
    double vel_error = current_vel - _cmd[i].vel;

    _tau[i] = -_cmd[i].kp * pos_error - _cmd[i].kd * vel_error + _cmd[i].tau;

    d->ctrl[i] = _polarity[i] * _tau[i];
  }
}

void MdlSimDriver::_readJointStates() {
  // Retrieve actuator states from the simulation and fill in _state[]
  if (_model && _data) {
    // Read actuator states instead of joint states
    // In MuJoCo, actuators are the controllable elements that apply forces/torques

    for (int i = 0; i < _model->nu; i++) {
      _state[i].t = _now;
      // Polarity mirrors the command path — keeps the stack's sign convention consistent
      _state[i].pos = _polarity[i] * _data->actuator_length[i];
      _state[i].vel = _polarity[i] * _data->actuator_velocity[i];
      _state[i].tau = _polarity[i] * _data->actuator_force[i];
    }
  }
}

void MdlSimDriver::_readIMUData() {
  if (_model && _data) {
    using quaternion::Quaternion;
    // Get orientation quaternion (qpos[3:7])
    Quaternion<double> q(_data->qpos[3], _data->qpos[4], _data->qpos[5], _data->qpos[6]);
    for (int i = 0; i < 4; ++i) _imu.q.v[i] = q.to_array()[i];
    // RPY (roll, pitch, yaw)
    auto q_unit = quaternion::normalize(q);
    auto rpy = quaternion::to_euler(q_unit);
    _imu.rpy[0] = rpy[2];  // roll
    _imu.rpy[1] = rpy[1];  // pitch
    _imu.rpy[2] = rpy[0];  // yaw
    // Linear acceleration (subtract gravity, then rotate to body frame)
    double acc_world[3] = {_data->qacc[0] - _model->opt.gravity[0],
                           _data->qacc[1] - _model->opt.gravity[1],
                           _data->qacc[2] - _model->opt.gravity[2]};
    Quaternion<double> acc_q(0, acc_world[0], acc_world[1], acc_world[2]);
    Quaternion<double> acc_body_q = quaternion::conj(q_unit) * acc_q * q_unit;
    _imu.acc.v[0] = acc_body_q.b();
    _imu.acc.v[1] = acc_body_q.c();
    _imu.acc.v[2] = acc_body_q.d();
    // Angular velocity: MuJoCo qvel[3:6] is already in body frame
    if (_model->nq >= 6) {
      _imu.gyro.v[0] = _data->qvel[3];
      _imu.gyro.v[1] = _data->qvel[4];
      _imu.gyro.v[2] = _data->qvel[5];
    } else {
      _imu.gyro.v[0] = _imu.gyro.v[1] = _imu.gyro.v[2] = 0.0;
    }
    // Magnetometer: world north (1,0,0) rotated to body frame
    Quaternion<double> mag_q(0, 1.0, 0.0, 0.0);
    Quaternion<double> mag_body_q = quaternion::conj(q_unit) * mag_q * q_unit;
    _imu.mag.v[0] = mag_body_q.b();
    _imu.mag.v[1] = mag_body_q.c();
    _imu.mag.v[2] = mag_body_q.d();
    _imu.t = _now;
  }
}
