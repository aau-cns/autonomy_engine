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

#include "state_machine/states/preflight.h"

namespace autonomy
{
Preflight::Preflight()
{
}

State& Preflight::Instance()
{
  static Preflight singleton;
  return singleton;
}

void Preflight::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatStateEntry("PREFLIGHT"));

  // Chek that provided mission exists
  if (!autonomy.missionFileSanityCheck())
  {
    autonomy.stateTransition("failure");
  }

  // Perform preflight checks and state transition
  if (!autonomy.preFlightChecks())
  {
    autonomy.stateTransition("failure");
  }
  autonomy.stateTransition("start_mission");
}

void Preflight::onExit(Autonomy&)
{
}

}  // namespace autonomy
