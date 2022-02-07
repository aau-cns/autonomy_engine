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

#include "state_machine/states/failure.h"

namespace autonomy {

  Failure::Failure() {};

  State& Failure::Instance() {
    static Failure singleton;
    return singleton;
  }

  void Failure::onEntry(Autonomy& autonomy) {

    // Print info
    AUTONOMY_UI_STREAM(BOLD(RED("-------------------------------------------------\n")));
    AUTONOMY_UI_STREAM(BOLD(RED(" >>> System state: FAILURE <<< \n")));
    AUTONOMY_UI_STREAM(BOLD(RED("-------------------------------------------------\n")) << std::endl);

    // Stop data recording if data is getting recorded after waiting autonomy.opts_->data_recording_delay_after_failure_s s
    if (autonomy.opts_->activate_data_recording && autonomy.is_recording_) {
      for (int cnt = 0; cnt <= 100*autonomy.opts_->data_recording_delay_after_failure_s; ++cnt) {
        autonomy.polling(10);
      }
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

    // If flying send an abort request to mission sequencer
    if (autonomy.in_flight_) {
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::ABORT);
    }

    // Unsubscribe from all the subscribed topics
    autonomy.sub_watchdog_heartbeat_.shutdown();
    autonomy.sub_watchdog_status_.shutdown();
    autonomy.sub_landing_detection_.shutdown();
    autonomy.sub_mission_sequencer_responce_.shutdown();

    // Call state transition to TERMINATION
    autonomy.stateTransition("termination");

  }

  void Failure::onExit(Autonomy&) {}

} // namespace autonomy

