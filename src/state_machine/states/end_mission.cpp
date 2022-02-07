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

#include "state_machine/states/end_mission.h"

namespace autonomy {

  EndMission::EndMission() {};

  State& EndMission::Instance() {
    static EndMission singleton;
    return singleton;
  }

  void EndMission::onEntry(Autonomy& autonomy) {

    // print info
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> System state: MISSION (ENDING) <<< \n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")) << std::endl);

    // Reset in_flight_ flag
    autonomy.in_flight_ = false;

    // Assume the disarm will happen automatically if we do not activate the landing detection thus set the armed_ flag
    // Otherwise send a disarm request and wait for disarming
    if (!autonomy.opts_->activate_landing_detection) {

      // Set armed_ flag
      autonomy.armed_ = false;

      // Stop flight timer
      if (!autonomy.multiple_touchdowns_ || (autonomy.filepaths_cnt_ == static_cast<int>(autonomy.missions_.at(autonomy.mission_id_).getTouchdowns()))) {
        autonomy.flight_timer_->stopTimer();
      }

    } else {

      // Disarm request
      if (autonomy.armed_) {
        autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::DISARM);
      } else {
        AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> the platform is already disarmed, skipped DISARM request\n")) << std::endl);
      }

      // Wait until disarm request got accepted
      while (autonomy.armed_) {
        autonomy.polling(10);
      }

    }

    // Check if we are here because mission got succesfully completed (last waypoint reached)
    // or if we safely land
    if (autonomy.last_waypoint_reached_) {

      if (autonomy.multiple_touchdowns_ && (autonomy.filepaths_cnt_ < static_cast<int>(autonomy.missions_.at(autonomy.mission_id_).getTouchdowns()))) {

        // Print info
        AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Iteration of mission ID: " + std::to_string(autonomy.mission_id_) + " succesfully completed.\n\n")));
        AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Continuing with next iteration ...\n")) << std::endl);

        // Give room for "phisical" landing and disarming by sleeping 10 seconds
        if (!autonomy.opts_->activate_landing_detection) {
          AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Waiting 10 seconds ...\n")) << std::endl);
          std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Increment the filepaths counter
        ++autonomy.filepaths_cnt_;

        // Setting state to START MISSION
        autonomy.stateTransition("preflight");

      } else {

        // Print info
        std::cout << BOLD(GREEN(" >>> Mission ID: " + std::to_string(autonomy.mission_id_) + " succesfully completed.\n")) << std::endl;

        // Terminate
        Terminate(autonomy);
      }

    } else {

      // Print info
      AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> Safety land <<<\n\n")));
      AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> Mission ID: " + std::to_string(autonomy.mission_id_) + " safely interrupted.\n")) << std::endl);

      // Terminate
      Terminate(autonomy);
    }
  }

  void EndMission::onExit(Autonomy&) {}

  void EndMission::Terminate(Autonomy& autonomy) {

    // Stop data recording if data is getting recorded
    if (autonomy.opts_->activate_data_recording && autonomy.is_recording_) {
      autonomy.DataRecording(false);
    }

    // Unsubscribe to landing after landing if detection if active
    if (autonomy.opts_->activate_landing_detection) {
      autonomy.sub_landing_detection_.shutdown();
    }

    // Call state transition to TERMINATION
    autonomy.stateTransition("termination");

  }

}
