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

#ifndef AUTONOMY_OPTIONS_H
#define AUTONOMY_OPTIONS_H

#include <iostream>
#include <vector>
#include <map>

#include "utils/colors.h"
#include "utils/utilities.h"
#include "autonomy_core/autonomy_defs.h"
#include "autonomy_core/mission.h"

namespace autonomy {

  /**
   * @brief Immutable Struct for autonomy options.
   *
   * This struct contains information mainly got from the
   * parameter server that are used by the autonomy.
   */
  struct autonomyOptions {

    /// Topic Names
    const std::string watchdog_heartbeat_topic;
    const std::string watchdog_status_topic;
    const std::string watchdog_action_topic;
    const std::string mission_sequencer_request_topic;
    const std::string mission_sequencer_responce_topic;
    const std::string landing_detection_topic;
    const std::string mission_sequencer_waypoints_topic;

    /// service Names
    const std::string watchdog_start_service_name;
    const std::string estimator_supervisor_service_name;
    const std::string data_recrding_service_name;
    const std::string takeoff_service_name;
    const std::string estimator_init_service_name;

    /// Timeout in milliseconds for watchdog heartbeat
    const int watchdog_timeout;

    /// Timeout in milliseconds for the maximum flight time
    const int flight_timeout;

    /// Timeout in milliseconds for the maximum time to wait for a sensor fix
    const int fix_timeout;

    /// Watchdog startup time in seconds needed to check entities
    const int watchdog_startup_time;

    /// Booleans, setup of the autonomy
    const bool activate_user_interface;
    const bool activate_watchdog;
    const bool activate_data_recording;
    const bool estimator_init_service;
    const bool perform_takeoff_check;
    const bool perform_estimator_check;
    const bool activate_landing_detection;

    /// Boolean to decide what to do in case of mission completion
    /// Note: This is not const because it can be overwritten during runtime
    bool hover_after_mission_completion;

    /// Mission to be loaded in case of no user interface
    const int mission_id_no_ui;

    /// Print function
    inline void printAutonomyOptions() {

      std::cout << BOLD(YELLOW("--------------- LOADED PARAMETERS ---------------\n\n"));

      std::cout << BOLD(YELLOW(" - User Interface:                        " + getStringfromBool(activate_user_interface) + "\n"));
      std::cout << BOLD(YELLOW(" - Watchdog:                              " + getStringfromBool(activate_watchdog) + "\n"));
      std::cout << BOLD(YELLOW(" - Data recording:                        " + getStringfromBool(activate_data_recording) + "\n"));
      std::cout << BOLD(YELLOW(" - Service call to initilize estimator:   " + getStringfromBool(estimator_init_service) + "\n"));
      std::cout << BOLD(YELLOW(" - Takeoff checks:                        " + getStringfromBool(perform_takeoff_check) + "\n"));
      std::cout << BOLD(YELLOW(" - Estimation quality checks:             " + getStringfromBool(perform_estimator_check) + "\n"));
      std::cout << BOLD(YELLOW(" - Landing detection:                     " + getStringfromBool(activate_landing_detection) + "\n"));
      std::cout << BOLD(YELLOW(" - Hover after mission completion:        " + getStringfromBool(hover_after_mission_completion) + "\n\n"));

      if (!activate_user_interface) {
        std::cout << BOLD(YELLOW(" - Loaded mission iwth ID:              " + std::to_string(mission_id_no_ui) + "\n\n"));
      }

      if (activate_watchdog) {
        std::cout << BOLD(YELLOW(" - Subscribed to:                         " + watchdog_heartbeat_topic + "\n"));
        std::cout << BOLD(YELLOW(" - Subscribed to:                         " + watchdog_status_topic + "\n"));
        std::cout << BOLD(YELLOW(" - Publishing on:                         " + watchdog_action_topic + "\n"));
        std::cout << BOLD(YELLOW(" - Service available at:                  " + watchdog_start_service_name + "\n"));
        std::cout << BOLD(YELLOW(" - Watchdog heartbeat timeout:            " + std::to_string(watchdog_timeout) + " ms\n"));
        std::cout << BOLD(YELLOW(" - Watchdog startup time:                 " + std::to_string(watchdog_startup_time) + " s\n\n"));
      }

      if (estimator_init_service) {
        std::cout << BOLD(YELLOW(" - Service available at:                  " + estimator_init_service_name + "\n\n"));
      }

      if (perform_estimator_check) {
        std::cout << BOLD(YELLOW(" - Service available at:                  " + estimator_supervisor_service_name + "\n\n"));
      }

      if (perform_takeoff_check) {
        std::cout << BOLD(YELLOW(" - Service available at:                  " + takeoff_service_name + "\n\n"));
      }

      if (activate_data_recording) {
        std::cout << BOLD(YELLOW(" - Service available at:                  " + data_recrding_service_name + "\n\n"));
      }

      if (activate_landing_detection) {
        std::cout << BOLD(YELLOW(" - Subscribed to:                         " + landing_detection_topic + "\n\n"));
      }

      std::cout << BOLD(YELLOW(" - Subscribed to:                         " + mission_sequencer_request_topic + "\n"));
      std::cout << BOLD(YELLOW(" - Publishing on:                         " + mission_sequencer_responce_topic + "\n"));
      std::cout << BOLD(YELLOW(" - Publishing on:                         " + mission_sequencer_waypoints_topic + "\n\n"));

      std::cout << BOLD(YELLOW(" - Maximum flight time:                   " + std::to_string(flight_timeout/60000) + " min\n\n"));

      std::cout << BOLD(YELLOW(" - Maximum waiting time for a sensor fix: " + std::to_string(fix_timeout) + " ms\n\n"));

      std::cout << BOLD(YELLOW("-------------------------------------------------\n")) << std::endl;
    }

    inline const std::string getStringfromBool(const bool& flag) const {
      if (flag) {
        return "Active";
      } else {
        return "Inactive";
      }
    }

  }; // struct AutonomyOptions

} // namespace autonomy

#endif  // AUTONOMY_OPTIONS_H
