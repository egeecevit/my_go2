/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#ifndef _VN100DRIVER_HH
#define _VN100DRIVER_HH

#include <pthread.h>

#include <atomic>
#include <string>

#include "hardware/IMUHW.hh"
#include "rtcore/ThreadedLoop.hh"

class ThreadUtil;

#define VN100_THREADNAME "vn100"

class VN100Driver : public rtcore::ThreadedLoop {
 public:
  VN100Driver(std::string device, rtcore::ModuleManager *mgr);
  ~VN100Driver();

  // Retrieve latest IMU data if its timestamp is different than the
  // one in the supplied struct. Return false if no new data is
  // present.
  bool getIMUData(IMUHW::imudata_t &imu) {
    if (imu.t == _imu.t) return false;  // No new data, don't copy
    pthread_mutex_lock(&_data_lock);
    imu = _imu;
    pthread_mutex_unlock(&_data_lock);
    return true;
  }

  void threadEnter(void);
  void threadLoop(void);
  void threadExit(void);

 private:
  rtcore::ModuleManager *_mgr = nullptr;

  // Serial device to read data from
  std::string _dev;

  // Mutex to coordinate data access between the reader thread and
  // getIMUData()
  pthread_mutex_t _data_lock;

  // Latest IMU data. This will be filled in by the IMU reader thread
  IMUHW::imudata_t _imu;
};

#endif
