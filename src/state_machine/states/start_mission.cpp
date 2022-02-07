// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.


#include "state_machine/states/start_mission.h"

namespace autonomy {

  StartMission::StartMission() {};

  State& StartMission::Instance() {
    static StartMission singleton;
    return singleton;
  }

  void StartMission::onEntry(Autonomy& autonomy) {

    // print info
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> System state: MISSION (STARTING) <<< \n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")) << std::endl);

    // Print info
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Arming...\n")) << std::endl);

    // Request arming to the mission sequencer if not armed, check that there are no pending failures before issuing an arming request
    if (!autonomy.armed_) {
      if (autonomy.pending_failures_.size() != 0) {
        std::cout << BOLD(YELLOW(" >>> Trying to fix pending failure before arming... (waiting time: " + std::to_string(autonomy.opts_->preflight_fix_timeout) + " s)\n")) << std::endl;
        for (int cnt = 0; cnt <= 100*autonomy.opts_->preflight_fix_timeout; ++cnt) {
          autonomy.polling(10);
        }
        if (autonomy.pending_failures_.size() != 0) {
          std::cout << BOLD(RED(" >>> Failed to fix pending failure before arming\n")) << std::endl;
          autonomy.stateTransition("failure");
        }
      }
      std::cout << BOLD(GREEN(" >>> Arming...\n")) << std::endl;
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::ARM);
    } else {
      AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> the platform is already armed, skipped ARM request\n")) << std::endl);
    }

    // Wait until the platform is armed
    while (!autonomy.armed_) {
      autonomy.polling(10);
    }

    // Print info
    autonomy.in_takeoff_ = true;
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Taking off...\n")) << std::endl);

    // Takeoff, if not already in flight and if armed
    if (!autonomy.in_flight_ && autonomy.armed_) {
      if (autonomy.pending_failures_.size() != 0) {
        std::cout << BOLD(YELLOW(" >>> Trying to fix pending failure before taking off... (waiting time: " + std::to_string(autonomy.opts_->preflight_fix_timeout) + " s)\n")) << std::endl;
        for (int cnt = 0; cnt <= 100*autonomy.opts_->preflight_fix_timeout; ++cnt) {
          autonomy.polling(10);
        }
        if (autonomy.pending_failures_.size() != 0) {
          std::cout << BOLD(RED(" >>> Failed to fix pending failure before taking off\n")) << std::endl;
          autonomy.stateTransition("failure");
        }
      }
      std::cout << BOLD(GREEN(" >>> Taking off...\n")) << std::endl;
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::TAKEOFF);
    } else {
      AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> the platform is already flying, skipped TAKEOFF request\n")) << std::endl);
    }

    // Wait until the platform is ready for the mission
    while (!autonomy.in_mission_) {
      autonomy.polling(10);
    }

    // Set filename to waypoint parser
    autonomy.waypoints_parser_->setFilename(autonomy.missions_.at(autonomy.mission_id_).getFilepaths().at(static_cast<size_t>(autonomy.filepaths_cnt_)));

    // Parse waypoint file
    autonomy.waypoints_parser_->readParseCsv();

    // Get the data
    autonomy.waypoints_ = autonomy.waypoints_parser_->getData();

    // Check existence of subscribers, if not trigger a land command
    // Check if we have loaded waypoints, if so send waypoints to mission sequencer, otherwise trigger a land command
    if (autonomy.waypoints_.size() > 0 && autonomy.pub_mission_sequencer_waypoints_.getNumSubscribers() > 0 && autonomy.state_->getStringFromState().compare("failure") != 0) {

      // Print info
      AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Communicating waypoints to the mission sequencer...\n")) << std::endl);

      // Define waypoints message to mission sequencer
      mission_sequencer::MissionWaypoint wp;
      mission_sequencer::MissionWaypointArray wps;

      // Set header and clear buffer action
      wps.header.stamp = ros::Time::now();
      wps.action = mission_sequencer::MissionWaypointArray::CLEAR;

      // Assign waypoints TODO
      for (const auto &it : autonomy.waypoints_) {
        wp.x = it.x;
        wp.y = it.y;
        wp.z = it.z;
        wp.yaw = it.yaw;
        wp.holdtime = it.holdtime;
        wps.waypoints.emplace_back(wp);
      }

      // publish mission waypoints
      autonomy.pub_mission_sequencer_waypoints_.publish(wps);

      // Call inflight sensor init service
      if (autonomy.opts_->estimator_init_service) {
        if (!autonomy.InFlightSensorInit()) {
          autonomy.stateTransition("failure");
        }
      }

      // Setting state to PERFORM_MISSION
      autonomy.stateTransition("perform_mission");

    } else {

      AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> No subscriber for mission sequencer waypoint. Triggering a safely land request...\n")) << std::endl);

      // Call state transition to LAND
      autonomy.stateTransition("land");
    }

  }

  void StartMission::onExit(Autonomy&) {}

}
