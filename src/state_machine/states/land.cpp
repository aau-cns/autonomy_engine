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

#include "state_machine/states/land.h"

namespace autonomy
{
Land::Land()
{
}

State& Land::Instance()
{
  static Land singleton;
  return singleton;
}

void Land::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatStateEntry("LAND"));

  // Print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Landing...", 2));

  // Request landing to the mission sequencer if flying
  if (autonomy.in_flight_ && autonomy.in_mission_)
  {
    autonomy.land_expected_ = true;
    autonomy.in_mission_ = false;
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                           formatMsg("Requesting [land] to Mission Sequencer"));
    autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::LAND);
  }
  else
  {
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                           formatMsg("The platform is not flying, skipped LAND request", 2));
  }
}

void Land::onExit(Autonomy&)
{
}

}  // namespace autonomy
