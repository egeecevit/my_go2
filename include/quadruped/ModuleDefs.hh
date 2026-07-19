/* 
 * Copyright (C) 2005-2025 Uluç Saranlı. All Rights Reserved
 * 
 * This file is part of the RoboMETU robot control software library
 * collection. Unauthorized copying of this file, via any medium is
 * strictly prohibited.
*/

#ifndef MODULEDEFS_HH
#define MODULEDEFS_HH

#include "quadruped/ModuleConfig.hh"

#define CREATE_MODULE(MGR, CLASS, VAR)                                \
  {                                                                   \
    VAR = new CLASS;                                                  \
    if (moduleConfig.count({VAR->getName(),VAR->getIndex()}) == 0)    \
      MGR->fatalError("CREATE_MODULE",                                \
                      "Could not find config for %s",                 \
                      VAR->getName());                                \
    ModuleConfig_t c                                                  \
      = moduleConfig.find({VAR->getName(),VAR->getIndex()})->second;  \
    MGR->addModule(VAR, c.period, c.offset, c.order);                 \
  }

#define DESTROY_MODULE(MGR, VAR)    \
  {                                 \
    MGR->removeModule(VAR);         \
    if (VAR) delete VAR;            \
    VAR = nullptr;                  \
  }

#define ACTIVATE_MODULE(MGR, VAR) MGR->activateModule(VAR)
#define DEACTIVATE_MODULE(MGR, VAR) MGR->deactivateModule(VAR)

#endif
