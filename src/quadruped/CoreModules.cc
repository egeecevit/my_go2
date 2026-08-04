/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#include "rtcore/ModuleManager.hh"
#include "rtcore/LogServer.hh"

#include "quadruped/CoreModules.hh"
#include "quadruped/MdlDrawSquare.hh"
#include "quadruped/MdlTrot.hh"
#include "quadruped/MdlStand.hh"
#include "quadruped/MdlSit.hh"
#include "quadruped/MdlLegControl.hh"
#include "quadruped/MdlOrientationEstimator.hh"
#include "quadruped/MdlPosVelEstimator.hh"
#include "quadruped/ModuleDefs.hh"

using namespace rtcore;

static LogServer  *_logserver = nullptr;
static MdlOrientationEstimator *_ori = nullptr;
static MdlPosVelEstimator *_pv = nullptr;
static MdlDrawSquare *_wm = nullptr;
static MdlTrot *_trot = nullptr;
static MdlStand *_sm = nullptr;
static MdlSit *_sit = nullptr;
static MdlLegControl  *_lm[4] = {nullptr, nullptr, nullptr, nullptr};

void AddCoreModules( ModuleManager *mgr ) {
  CREATE_MODULE(mgr, LogServer, _logserver );
  // Orientation before position: the second stage looks the first up in its
  // init(), and addModule() runs init() as the module is added, so creating
  // them the other way round would leave that lookup permanently empty.
  CREATE_MODULE(mgr, MdlOrientationEstimator, _ori);
  CREATE_MODULE(mgr, MdlPosVelEstimator, _pv);

  for (int i = 0; i < 4; i++)
    CREATE_MODULE(mgr, MdlLegControl(i), _lm[i] );

  CREATE_MODULE(mgr, MdlDrawSquare, _wm );
  CREATE_MODULE(mgr, MdlTrot, _trot);
  CREATE_MODULE(mgr, MdlStand, _sm);
  CREATE_MODULE(mgr, MdlSit, _sit);
}

void ActivateCoreModules(ModuleManager *mgr ) {
  ACTIVATE_MODULE(mgr, _logserver);
  // Both estimator stages always run: they are sensors for everything else, so
  // they are activated here rather than being grabbed by an individual
  // behavior.
  ACTIVATE_MODULE(mgr, _ori);
  ACTIVATE_MODULE(mgr, _pv);
}

void DeactivateCoreModules(ModuleManager *mgr ) {
  DEACTIVATE_MODULE(mgr, _sit);
  DEACTIVATE_MODULE(mgr, _sm);
  DEACTIVATE_MODULE(mgr, _trot);
  DEACTIVATE_MODULE(mgr, _wm);
  for (int i = 0; i < 4; i++) DEACTIVATE_MODULE(mgr, _lm[i]);
  DEACTIVATE_MODULE(mgr, _pv);
  DEACTIVATE_MODULE(mgr, _ori);
  DEACTIVATE_MODULE(mgr, _logserver);
}

void RemoveCoreModules(ModuleManager *mgr ) {
  DESTROY_MODULE(mgr, _sit);
  DESTROY_MODULE(mgr, _sm);
  DESTROY_MODULE(mgr, _trot);
  DESTROY_MODULE(mgr, _wm);
  for (int i = 0; i < 4; i++) DESTROY_MODULE(mgr, _lm[i]);
  DESTROY_MODULE(mgr, _pv);
  DESTROY_MODULE(mgr, _ori);
  DESTROY_MODULE(mgr, _logserver);
}
