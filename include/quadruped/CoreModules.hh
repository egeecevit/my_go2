/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef _COREMODULES_H
#define _COREMODULES_H

#include "rtcore/ModuleManager.hh"

/** This function adds all core modules to the ModuleManager's
    database. It should be called in main() before mainLoop() is
    invoked. */
extern void AddCoreModules( rtcore::ModuleManager *mgr );

/** This function activates critical modules. It should be
    called in main() before mainLoop() is invoked. */
extern void ActivateCoreModules( rtcore::ModuleManager *mgr );

/** This function deactivates critical modules that were
    activated with ActivateCoreModules(). It should be called after
    mainLoop() returns. */
extern void DeactivateCoreModules( rtcore::ModuleManager *mgr );

/** This function removes standard RHex modules from the
    ModuleManager's database */
extern void RemoveCoreModules( rtcore::ModuleManager *mgr );

#endif
