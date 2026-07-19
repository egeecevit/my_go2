/*
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 *
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
 */

#include "VN100Driver.hh"

#include <unistd.h>

#include "rtcore/ModuleManager.hh"
#include "rtcore/ThreadUtil.hh"

// Comment in/out beyond printf to enable/disable debug messages
#define DBGPRINT(...) //printf(__VA_ARGS__);

using namespace rtcore;

VN100Driver::VN100Driver(std::string device, ModuleManager *mgr) {
  pthread_mutex_init(&_data_lock, NULL);
  _dev = device;
  _mgr = mgr;
}

VN100Driver::~VN100Driver() {
  terminate();
  pthread_mutex_destroy(&_data_lock);
}

void VN100Driver::threadEnter() { DBGPRINT("VN100Driver: Thread started.\n"); }

void VN100Driver::threadLoop() {
  waitSync();
  usleep(100000);
}

void VN100Driver::threadExit() { DBGPRINT("VN100Driver: Thread exiting.\n"); }
