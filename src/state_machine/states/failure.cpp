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

#include "state_machine/states/failure.h"
#include "utils/colors.h"

namespace autonomy {

  Failure::Failure() {};

  State& Failure::Instance() {
    static Failure singleton;
    return singleton;
  }

  void Failure::onEntry(Autonomy& autonomy) {

    // Print info
    std::cout << BOLD(RED("-------------------------------------------------\n"));
    std::cout << BOLD(RED(" >>> System state: FAILURE <<< \n"));
    std::cout << BOLD(RED("-------------------------------------------------\n")) << std::endl;

    // Stop data recording if data is getting recorded
    if (autonomy.opts_->activate_data_recording && autonomy.is_recording_) {
      autonomy.DataRecording(false);
    }

    // Stop any timer
    autonomy.watchdog_timer_->stopTimer();
    autonomy.flight_timer_->stopTimer();
    for (const auto& it : autonomy.pending_failures_) {
      it.second->stopTimer();
    }

    // Clear panding failure
    autonomy.pending_failures_.clear();

    // Clear waypoints
    autonomy.waypoints_.clear();

    // Send abort request to mission sequencer
    autonomy.missionSequencerRequest(amaze_mission_sequencer::request::ABORT);

    // Unsubscribe from all the subscribed topics
    autonomy.sub_watchdog_heartbeat_.shutdown();
    autonomy.sub_watchdog_status_.shutdown();
    autonomy.sub_landing_detection_.shutdown();
    autonomy.sub_mission_sequencer_responce_.shutdown();

    // Throw exception
    throw FailureException();

  }

  void Failure::onExit(Autonomy&) {}

} // namespace autonomy

