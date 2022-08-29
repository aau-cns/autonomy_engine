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

#include "state_machine/states/mission_iterator.h"

namespace autonomy
{
MissionIterator::MissionIterator()
{
}

State& MissionIterator::Instance()
{
  static MissionIterator singleton;
  return singleton;
}

void MissionIterator::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                         formatStateEntry("MISSION (ITERATOR)"));

  // Print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                         formatMsg("Iteration " + std::to_string(autonomy.filepaths_cnt_) + " of mission ID: " +
                                   std::to_string(autonomy.mission_id_) + " succesfully completed"));
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                         formatMsg("Continuing with iteration " + std::to_string(++autonomy.filepaths_cnt_), 2));

  // Check flags to manage mission iterations
  if (autonomy.armed_ && autonomy.in_flight_ && autonomy.in_mission_)
  {
    // Reset last_waypoint_reached_ flag
    autonomy.last_waypoint_reached_ = false;

    // Transition to START MISSION
    autonomy.stateTransition("start_mission");
  }
  else
  {
    // Give room for "phisical" landing and disarming by sleeping 10 seconds
    if (!autonomy.opts_->activate_landing_detection)
    {
      autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                             formatMsg("Waiting 10 seconds...", 2));
      std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    // Setting state to PREFLIGHT
    autonomy.stateTransition("preflight");
  }
}

void MissionIterator::onExit(Autonomy&)
{
}

}  // namespace autonomy
