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

// Static callback function for GLFW keyboard interaction
static void _keyCallback(GLFWwindow* window, int key, int scancode, int action,
                         int mods) {
  MdlSimDriver* driver = static_cast<MdlSimDriver*>(glfwGetWindowUserPointer(window));
  if (driver) driver->_handleKeyboard(key, scancode, action, mods);
}

void MdlSimDriver::_handleKeyboard(int key, int scancode, int action, int mods) {
  // Handle keyboard input for controlling the simulation
  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    switch (key) {
      case GLFW_KEY_Q:
      case GLFW_KEY_ESCAPE:
        // Exit the main loop when 'q' is pressed
        DBGPRINT("MdlSimDriver: 'q' key pressed - exiting main loop\n");
        _mgr->exitMainLoop();
        break;
    }
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

    // Update scene and render
    mjv_updateScene(_model, _data, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
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
    // Keyframe not found (may happen with <include> in some MuJoCo versions).
    // Apply Go2 home pose directly: z=0.27, joints=[0,0.9,-1.8] x4
    _data->qpos[2] = 0.27;
    for (int leg = 0; leg < 4; leg++) {
      _data->qpos[7 + leg * 3 + 0] = 0.0;
      _data->qpos[7 + leg * 3 + 1] = 0.9;
      _data->qpos[7 + leg * 3 + 2] = -1.8;
    }
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
  // Register MuJoCo controller callback
  mjcb_control = _controllerCallback_static;
}

void MdlSimDriver::_destroySimulation() {
  DBGPRINT("MdlSimDriver::_destroySimulation\n");

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
      _state[i].t = _now;  // Set timestamp

      _state[i].pos = _data->actuator_length[i];
      _state[i].vel = _data->actuator_velocity[i];
      _state[i].tau = _data->actuator_force[i];
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

// Helper: Quaternion conjugate
void quat_conj(const double q[4], double q_conj[4]) {
  q_conj[0] = q[0];
  q_conj[1] = -q[1];
  q_conj[2] = -q[2];
  q_conj[3] = -q[3];
}

// Helper: Quaternion multiplication (a * b)
void quat_mult(const double a[4], const double b[4], double out[4]) {
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// Helper: Rotate vector v (length 3) by quaternion q (w,x,y,z), result in v_out
void quat_rotate_inv(const double q[4], const double v[3], double v_out[3]) {
  double q_conj[4];
  quat_conj(q, q_conj);
  double vq[4] = {0, v[0], v[1], v[2]};
  double tmp[4], res[4];
  quat_mult(q_conj, vq, tmp);
  quat_mult(tmp, q, res);
  v_out[0] = res[1];
  v_out[1] = res[2];
  v_out[2] = res[3];
}

// Helper: Quaternion to RPY (roll, pitch, yaw)
void quat_to_rpy(const double q[4], double rpy[3]) {
  // Assuming q = [w, x, y, z]
  double w = q[0], x = q[1], y = q[2], z = q[3];
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  rpy[0] = std::atan2(sinr_cosp, cosr_cosp);
  // Pitch (y-axis rotation)
  double sinp = 2.0 * (w * y - z * x);
  if (std::abs(sinp) >= 1)
    rpy[1] = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    rpy[1] = std::asin(sinp);
  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}
