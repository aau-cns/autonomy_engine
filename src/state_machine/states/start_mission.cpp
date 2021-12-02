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
    std::cout << BOLD(GREEN("-------------------------------------------------\n"));
    std::cout << BOLD(GREEN(" >>> System state: MISSION (STARTING) <<< \n"));
    std::cout << BOLD(GREEN("-------------------------------------------------\n")) << std::endl;

    // Print info
    std::cout << BOLD(GREEN(" >>> Arming...\n")) << std::endl;

    // Request arming to the mission sequencer if not armed
    if (!autonomy.armed_) {
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::ARM);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already armed, skipped ARM request\n")) << std::endl;
    }

    // Wait until the platform is armed
    while (!autonomy.armed_) {
      autonomy.polling(10);
    }

    // Print info
    std::cout << BOLD(GREEN(" >>> Taking off...\n")) << std::endl;

    // Takeoff, if not already in flight and if armed
    if (!autonomy.in_flight_ && autonomy.armed_) {
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::TAKEOFF);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already flying, skipped TAKEOFF request\n")) << std::endl;
    }

    // Wait until the platform is flying
    while (!autonomy.in_flight_) {
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
    if (autonomy.waypoints_.size() > 0 && autonomy.pub_mission_sequencer_waypoints_.getNumSubscribers() > 0) {

      // Print info
      std::cout << BOLD(GREEN(" >>> Communicating waypoints to the mission sequencer...\n")) << std::endl;

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

      // publish mission start request
      autonomy.pub_mission_sequencer_waypoints_.publish(wps);

      // Setting state to PERFORM_MISSION
      autonomy.stateTransition("perform_mission");

    } else {

      std::cout << BOLD(YELLOW(" >>> No subscriber for mission sequencer waypoint. Triggering a safely land request...\n")) << std::endl;

      // Call state transition to LAND
      autonomy.stateTransition("land");
    }

  }

  void StartMission::onExit(Autonomy&) {}

}
