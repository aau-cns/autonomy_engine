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
EndMission::EndMission()
{
}

State& EndMission::Instance()
{
  static EndMission singleton;
  return singleton;
}

void EndMission::onEntry(Autonomy& autonomy)
{
  // This state is reached, if the landing has been confirmed by mission sequencer or landing detector

  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatStateEntry("MISSION (ENDING)"));

  // Print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Disarm...", 2));

  if (autonomy.opts_->activate_landing_detection)
  {
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
  }
  // else
  // {
  //   // If landing detection is not active assume the disarm will happen automatically otherwise send a disarm request
  //   // and wait for disarming. Assuming the disarm will happen automatically rather than sending an explicit request is
  //   // safer since we are not sure the platform is on the ground. For the sake of completeness, a commented disarm
  //   // request is kept here
  //   autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::DISARM);
  // }

  // Wait until disarm request got accepted
  while (autonomy.armed_)
  {
    autonomy.polling(10);
  }

  // Check if we are here because mission got succesfully completed (last waypoint reached)
  // or if we safely land
  if (autonomy.last_waypoint_reached_)
  {
    // Check if we need to iterate
    // (filepaths counter + 1) * (instance counter + 1) is grater than the number of touchdowns only at the last
    // iteration, thus stop iterating
    size_t t = (autonomy.filepaths_cnt_ + 1) * (autonomy.instances_cnt_ + 1);

    if (autonomy.multiple_touchdowns_ && !autonomy.opts_->sequence_multiple_in_flight &&
        t <= autonomy.missions_.at(autonomy.mission_id_).getTouchdowns())
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
