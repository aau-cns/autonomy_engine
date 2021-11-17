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

#include <iostream>
#include <amaze_mission_sequencer/request.h>
#include <amaze_mission_sequencer/response.h>

#include "state_machine/states/takeoff.h"
#include "utils/colors.h"

namespace autonomy {

  Takeoff::Takeoff() {};

  State& Takeoff::Instance() {
    static Takeoff singleton;
    return singleton;
  }

  void Takeoff::onEntry(Autonomy& autonomy) {

    // print info
    std::cout << BOLD(GREEN("-------------------------------------------------\n"));
    std::cout << BOLD(GREEN(" >>> System state: TAKEOFF <<< \n"));
    std::cout << BOLD(GREEN("-------------------------------------------------\n")) << std::endl;

    // Perform preflight checks
    autonomy.preFlightChecks();

    // Print info
    std::cout << BOLD(GREEN(" >>> Arming...\n")) << std::endl;

    // Request arming to the mission sequencer if not armed
    if (!autonomy.armed_) {
      autonomy.missionSequencerRequest(amaze_mission_sequencer::request::ARM);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already armed, skipped ARM request\n")) << std::endl;
    }

    // Wait until the platform is armed
    while (!autonomy.armed_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Print info
    std::cout << BOLD(GREEN(" >>> Taking off...\n")) << std::endl;

    // Takeoff, if not already in flight and if armed
    if (!autonomy.in_flight_ && autonomy.armed_) {
      autonomy.missionSequencerRequest(amaze_mission_sequencer::request::TAKEOFF);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already flying, skipped TAKEOFF request\n")) << std::endl;
    }

    // Wait until the platform is flying
    while (!autonomy.in_flight_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Setting state to FLIGHT
    autonomy.next_state_ = AutonomyState::FLIGHT;
    autonomy.stateTransition();

  }

  void Takeoff::onExit(Autonomy& autonomy) {

    // Set filename to waypoint parser
    autonomy.waypoints_parser_->setFilename(autonomy.missions_.at(autonomy.mission_id_).getFilepaths().at(static_cast<size_t>(autonomy.filepaths_cnt_)));

    // Parse waypoint file
    autonomy.waypoints_parser_->readParseCsv();

    // Get the data
    autonomy.waypoints_ = autonomy.waypoints_parser_->getData();

    // Wait until the platform is flying (takeoff completed)
    while (!autonomy.in_flight_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Check if we have loaded waypoints, if so send waypoints to mission sequencer, otherwise trigger a land command
    if (autonomy.waypoints_.size() > 0) {

      // Print info
      std::cout << BOLD(GREEN(" >>> Communicating waypoints to the mission sequencer...\n")) << std::endl;

      // If we already performed a takeoff
      if (autonomy.in_flight_) {

        // TODO: Send waypoints
        //    // Define waypoints message to mission sequencer
        //    amaze_mission_sequencer::waypoints wps;

        //    // Set mission id and request
        //    wps.header.stamp = ros::Time::now();
        //    wps.action = amaze_mission_sequencer::waypoints

        //    // publish mission start request
        //    pub_mission_sequencer_waypoints_.publish(wps);

      } else {
        std::cout << BOLD(YELLOW(" >>> the platform is not flying, waypoints cannot be sent to mission sequencer\n")) << std::endl;
      }

    } else {

      // Call state transition to LAND
      autonomy.next_state_ = AutonomyState::LAND;
      autonomy.stateTransition();

    }

  }

}
