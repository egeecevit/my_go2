/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef MODULECONFIG_HH
#define MODULECONFIG_HH

#include "rtcore/ClockHW.hh"
#include "rtcore/ModuleManager.hh"

#include <map>
#include <string>

typedef struct {
  std::string name;
  int index;        // Use -1 to cover all indices
} ModuleId_t;

typedef struct  {
  bool operator()(const ModuleId_t& m1, const ModuleId_t& m2)const {
    return ((m1.name < m2.name)
            || (m1.name == m2.name
                && m1.index != -1
                && m2.index != -1
                && (m1.index < m2.index)));
  }
} ModuleCompare_t;

typedef struct {
  unsigned int period;
  unsigned int offset;
  unsigned int order;
} ModuleConfig_t;

const std::map<ModuleId_t, ModuleConfig_t, ModuleCompare_t> moduleConfig = {
  // Sim HW related modules
  { {"MdlSimDriver", -1},  { 1, 0, ACTUATOR_MODULES } },

  // Robot HW related modules
  { {"MdlMotorMaster", -1},  { 500, 0, ACTUATOR_MODULES } },

    // Faulhaber Robot HW related modules
  { {"MdlGo1", -1},  { 1, 0, ACTUATOR_MODULES } },

  // Core modules
  { {"LogServer", -1},    { 1, 0, LOGGING_MODULES   } },
  { {"MdlHWTest", -1},       { 2, 0, BEHAVIORAL_CONTROLLERS   } },
  { {"MdlDrawSquare", -1},   { 1, 0, BEHAVIORAL_CONTROLLERS   } },
  { {"MdlLegControl", 0},     { 1, 0, BEHAVIORAL_CONTROLLERS+1 } },
  { {"MdlLegControl", 1},     { 1, 0, BEHAVIORAL_CONTROLLERS+2 } },
  { {"MdlLegControl", 2},     { 1, 0, BEHAVIORAL_CONTROLLERS+3 } },
  { {"MdlLegControl", 3},     { 1, 0, BEHAVIORAL_CONTROLLERS+4 } },
};


#endif
