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

#include "state_machine/states/end_mission.h"

namespace autonomy
{
EndMission::EndMission(){};

State& EndMission::Instance()
{
  static EndMission singleton;
  return singleton;
}

void EndMission::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatStateEntry("MISSION (ENDING)"));

  // Assume the disarm will happen automatically if we do not activate the landing detection thus set the armed_ flag
  // Otherwise send a disarm request and wait for disarming
  if (!autonomy.opts_->activate_landing_detection)
  {
    // Reset armed_ and in_flight_ flag
    autonomy.armed_ = false;
    autonomy.in_flight_ = false;

    // Stop flight timer
    if (!autonomy.multiple_touchdowns_ ||
        (autonomy.filepaths_cnt_ == static_cast<int>(autonomy.missions_.at(autonomy.mission_id_).getTouchdowns())))
    {
      autonomy.flight_timer_->stopTimer();
    }
  }
  else
  {
    // Print info
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Disarm...", 2));

    // Disarm request
    if (autonomy.armed_)
    {
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::DISARM);
    }
    else
    {
      autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                             formatMsg("The platform is already disarmed, skipped DISARM request", 2));
    }

    // Wait until disarm request got accepted
    while (autonomy.armed_)
    {
      autonomy.polling(10);
    }
  }

  // Check if we are here because mission got succesfully completed (last waypoint reached)
  // or if we safely land
  if (autonomy.last_waypoint_reached_)
  {
    if (autonomy.multiple_touchdowns_ && !autonomy.opts_->sequence_multiple_in_flight_ &&
        (autonomy.filepaths_cnt_ < static_cast<int>(autonomy.missions_.at(autonomy.mission_id_).getTouchdowns())))
    {
      autonomy.stateTransition("mission_iterator");
    }
    else
    {
      // Print info
      autonomy.logger_.logUI(
          getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
          formatMsg("Mission ID: " + std::to_string(autonomy.mission_id_) + " succesfully completed", 2));

      // Terminate
      Terminate(autonomy);
    }
  }
  else
  {
    // Print info
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), formatMsg("Safety land"));
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                           formatMsg("Mission ID: " + std::to_string(autonomy.mission_id_) + " safely interrupted", 2));

    // Terminate
    Terminate(autonomy);
  }
}

void EndMission::onExit(Autonomy&)
{
}

void EndMission::Terminate(Autonomy& autonomy)
{
  // Stop data recording if data is getting recorded
  if (autonomy.opts_->activate_data_recording && autonomy.is_recording_)
  {
    autonomy.DataRecording(false);
  }

  // Unsubscribe to landing after landing if detection if active
  if (autonomy.opts_->activate_landing_detection)
  {
    autonomy.sub_landing_detection_.shutdown();
  }

  // Call state transition to TERMINATION
  autonomy.stateTransition("termination");
}

}  // namespace autonomy
