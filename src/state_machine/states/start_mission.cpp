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
#include "state_machine/states/start_mission.h"

namespace autonomy
{
StartMission::StartMission(){};

State& StartMission::Instance()
{
  static StartMission singleton;
  return singleton;
}

void StartMission::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                         formatStateEntry("MISSION (STARTING)"));
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Arming...", 2));

  // Request arming to the mission sequencer if not armed, check that there are no pending failures before issuing an
  // arming request
  if (!autonomy.armed_)
  {
    if (autonomy.pending_failures_.size() != 0)
    {
      autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                             formatMsg("Trying to fix pending failure before arming... (max waiting time: " +
                                           std::to_string(autonomy.opts_->preflight_fix_timeout) + " s)",
                                       2));
      for (int cnt = 0; cnt <= std::ceil(autonomy.opts_->preflight_fix_timeout / 10); ++cnt)
      {
        autonomy.polling(10);
      }
      if (autonomy.pending_failures_.size() != 0)
      {
        autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                               formatMsg("Failed to fix pending failure before arming", 2));
        autonomy.stateTransition("failure");
      }
    }
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                           formatMsg("Requesting [arm] to Mission Sequencer"));
    autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::ARM);
  }
  else
  {
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                           formatMsg("The platform is already armed, skipped ARM request", 2));
  }

  // Wait until the platform is armed
  while (!autonomy.armed_)
  {
    autonomy.polling(10);
  }

  // Print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Taking off...", 2));

  // Takeoff, if not already in flight and if armed
  if (!autonomy.in_flight_ && autonomy.armed_)
  {
    if (autonomy.pending_failures_.size() != 0)
    {
      autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                             formatMsg("Trying to fix pending failure before taking off... (max waiting time: " +
                                           std::to_string(autonomy.opts_->preflight_fix_timeout) + " s)",
                                       2));
      for (int cnt = 0; cnt <= std::ceil(autonomy.opts_->preflight_fix_timeout / 10); ++cnt)
      {
        autonomy.polling(10);
      }
      if (autonomy.pending_failures_.size() != 0)
      {
        autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                               formatMsg("Failed to fix pending failure before taking off", 2));
        autonomy.stateTransition("failure");
      }
    }
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                           formatMsg("Requesting [takeoff] to Mission Sequencer"));
    autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::TAKEOFF);
  }
  else
  {
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                           formatMsg("The platform is already flying, skipped TAKEOFF request", 2));
  }

  // Wait until the platform is ready for the mission
  while (!autonomy.in_mission_)
  {
    autonomy.polling(10);
  }

  // Set filename to waypoint parser
  autonomy.waypoints_parser_->setFilename(
      autonomy.missions_.at(autonomy.mission_id_).getFilepaths().at(static_cast<size_t>(autonomy.filepaths_cnt_)));

  // Parse waypoint file
  autonomy.waypoints_parser_->readParseCsv();

  // Get the data
  autonomy.waypoints_ = autonomy.waypoints_parser_->getData();

  // Check existence of subscribers, if not trigger a land command
  // Check if we have loaded waypoints, if so send waypoints to mission sequencer, otherwise trigger a land command
  if (autonomy.waypoints_.size() > 0 && autonomy.pub_mission_sequencer_waypoints_.getNumSubscribers() > 0)
  {
    // Print info
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                           formatMsg("Communicating waypoints to the mission sequencer...", 2));

    // Define waypoints message to mission sequencer
    mission_sequencer::MissionWaypoint wp;
    mission_sequencer::MissionWaypointArray wps;

    // Set header and clear buffer action
    wps.header.stamp = ros::Time::now();
    wps.action = mission_sequencer::MissionWaypointArray::CLEAR;

    // Assign waypoints TODO
    for (const auto& it : autonomy.waypoints_)
    {
      wp.x = it.x;
      wp.y = it.y;
      wp.z = it.z;
      wp.yaw = it.yaw;
      wp.holdtime = it.holdtime;
      wps.waypoints.emplace_back(wp);
    }

    // Log message
    autonomy.logger_.logMessage(getStringFromState(), autonomy.opts_->watchdog_action_topic, false,
                                "[mission sequencer message] Waypoints");

    // publish mission waypoints
    autonomy.pub_mission_sequencer_waypoints_.publish(wps);

    // Call inflight sensor init service
    if (autonomy.opts_->estimator_init_service)
    {
      if (!autonomy.InFlightSensorInit())
      {
        autonomy.stateTransition("failure");
      }
    }

    // Setting state to PERFORM_MISSION
    autonomy.stateTransition("perform_mission");
  }
  else
  {
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                           formatMsg("Impossible to communicate waypoints to mission sequencer", 2));

    // Call state transition to LAND
    autonomy.stateTransition("land");
  }

}  // namespace autonomy

void StartMission::onExit(Autonomy&)
{
}

}  // namespace autonomy
