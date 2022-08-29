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

#include "state_machine/states/failure.h"

namespace autonomy
{
Failure::Failure(){};

State& Failure::Instance()
{
  static Failure singleton;
  return singleton;
}

void Failure::onEntry(Autonomy& autonomy)
{
  // Print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE), formatStateEntry("FAILURE"));

  // Stop data recording if data is getting recorded after waiting autonomy.opts_->data_recording_delay_after_failure_s
  // seconds
  if (autonomy.opts_->activate_data_recording && autonomy.is_recording_)
  {
    for (int cnt = 0; cnt <= std::ceil(autonomy.opts_->data_recording_delay_after_failure_s * 100); ++cnt)
    {
      autonomy.polling(10);
    }
    autonomy.DataRecording(false);
  }

  // Stop any timer
  autonomy.watchdog_timer_->stopTimer();
  autonomy.flight_timer_->stopTimer();
  for (const auto& it : autonomy.pending_failures_)
  {
    it.second->stopTimer();
  }

  // Clear panding failure
  autonomy.pending_failures_.clear();

  // Clear waypoints
  autonomy.waypoints_.clear();

  // If flying send an abort request to mission sequencer
  if (autonomy.in_flight_)
  {
    autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::ABORT);
  }

  // Unsubscribe from all the subscribed topics
  autonomy.sub_watchdog_heartbeat_.shutdown();
  autonomy.sub_watchdog_status_.shutdown();
  autonomy.sub_landing_detection_.shutdown();
  autonomy.sub_mission_sequencer_response_.shutdown();

  // Call state transition to TERMINATION
  autonomy.stateTransition("termination");
}

void Failure::onExit(Autonomy&)
{
}

}  // namespace autonomy
