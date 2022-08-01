// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>

#include "state_machine/states/perform_mission.h"

namespace autonomy
{
PerformMission::PerformMission(){};

State& PerformMission::Instance()
{
  static PerformMission singleton;
  return singleton;
}

void PerformMission::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                         formatStateEntry("MISSION (PERFORMING)"));
}

void PerformMission::onExit(Autonomy&)
{
}

}  // namespace autonomy
