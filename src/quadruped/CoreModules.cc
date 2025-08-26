/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/ModuleManager.hh"
#include "rtcore/LogServer.hh"

#include "CoreModules.hh"
#include "MdlDrawSquare.hh"
#include "MdlLegControl.hh"
#include "ModuleDefs.hh"

using namespace rtcore;

static LogServer  *_logserver = nullptr;
static MdlDrawSquare *_wm = nullptr;
static MdlLegControl  *_lm[4] = {nullptr, nullptr, nullptr, nullptr};

void AddCoreModules( ModuleManager *mgr ) {
  CREATE_MODULE(mgr, LogServer, _logserver );

  for (int i = 0; i < 4; i++)
    CREATE_MODULE(mgr, MdlLegControl(i), _lm[i] );

  CREATE_MODULE(mgr, MdlDrawSquare, _wm );
}

void ActivateCoreModules(ModuleManager *mgr ) {
  ACTIVATE_MODULE(mgr, _logserver);
}

void DeactivateCoreModules(ModuleManager *mgr ) {
  DEACTIVATE_MODULE(mgr, _wm);
  for (int i = 0; i < 4; i++) DEACTIVATE_MODULE(mgr, _lm[i]);
  DEACTIVATE_MODULE(mgr, _logserver);
}

void RemoveCoreModules(ModuleManager *mgr ) {
  DESTROY_MODULE(mgr, _wm);
  for (int i = 0; i < 4; i++) DESTROY_MODULE(mgr, _lm[i]);
  DESTROY_MODULE(mgr, _logserver);
}
