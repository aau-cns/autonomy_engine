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

#ifndef AUTONOMY_OPTIONS_H
#define AUTONOMY_OPTIONS_H

#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "autonomy_core/autonomy_defs.h"
#include "autonomy_core/mission.h"
#include "utils/colors.h"
#include "utils/logger.h"
#include "utils/utilities.h"

namespace autonomy
{
/**
 * @brief Immutable Struct for autonomy options.
 *
 * This struct contains information mainly got from the
 * parameter server that are used by the autonomy.
 */
struct autonomyOptions
{
  /// Topic Names
  const std::string watchdog_heartbeat_topic;
  const std::string watchdog_status_topic;
  const std::string watchdog_action_topic;
  const std::string mission_sequencer_request_topic;
  const std::string mission_sequencer_response_topic;
  const std::string landing_detection_topic;
  const std::string mission_sequencer_waypoints_topic;
  const std::string mission_sequencer_waypoint_reached_topic;
  const std::string rc_topic;
  const std::string battery_status_topic;

  /// service Names
  const std::string watchdog_start_service_name;
  const std::string estimator_supervisor_service_name;
  const std::string data_recrding_service_name;
  const std::string takeoff_service_name;
  const std::string estimator_init_service_name;
  const std::string logger_filepath;
  const std::string trajectory_dir;
  const std::vector<std::string> inflight_sensor_init_services_name;

  /// Timeout in milliseconds for watchdog heartbeat
  const int watchdog_timeout;

  /// Timeout in milliseconds for the maximum flight time
  const int flight_timeout;

  /// Timeout in milliseconds for the maximum time to wait for a sensor fix
  const int fix_timeout;
  const int preflight_fix_timeout;

  /// Watchdog startup time in seconds needed to check entities
  const int watchdog_startup_time;

  /// Time waited before stopping data recording after failure
  const int data_recording_delay_after_failure_s;

  /// Booleans, setup of the autonomy
  const bool activate_user_interface;
  const bool activate_watchdog;
  const bool activate_data_recording;
  const bool estimator_init_service;
  const bool perform_takeoff_check;
  const bool perform_estimator_check;
  const bool activate_landing_detection;
  const bool inflight_sensors_init_service;
  const bool register_aux;

  /// Boolean to decide what to do in case of mission completion
  /// Note: This is not const because it can be overwritten during runtime
  bool hover_after_mission_completion;

  /// Bolean to decide if multiple files are sequenced in-flight
  const bool sequence_multiple_in_flight;

  /// Mission to be loaded in case of no user interface
  const int mission_id_no_ui;

  /// Mission to be loaded in case of no user interface
  const size_t landing_aux_channel;

  /// Log Display Level
  const LogDisplayLevel log_display_level;  // 0: basic log, 1: waypoint logs, 2: interaction logs, 3: all logs

  /// Battery Status Update Interval (percent)
  const float battery_voltage_interval_percent;

  /// Print function
  inline const std::string printAutonomyOptions()
  {
    std::stringstream ss;
    ss << "-------------------------------------------------\n"
       << "LOADED PARAMETERS:\n\n"
       << " - User Interface:                      " << getStringfromBool(activate_user_interface) << '\n'
       << " - Logger filepath:                     " << logger_filepath << '\n'
       << " - Trajectories Directory:              " << trajectory_dir << '\n'
       << " - Watchdog:                            " << getStringfromBool(activate_watchdog) << '\n'
       << " - Data recording:                      " << getStringfromBool(activate_data_recording) << '\n'
       << " - Estimator init service:              " << getStringfromBool(estimator_init_service) << '\n'
       << " - Takeoff checks:                      " << getStringfromBool(perform_takeoff_check) << '\n'
       << " - Estimation quality checks:           " << getStringfromBool(perform_estimator_check) << '\n'
       << " - Landing detection:                   " << getStringfromBool(activate_landing_detection) << '\n'
       << " - In-flight sensors init:              " << getStringfromBool(inflight_sensors_init_service) << '\n'
       << " - Hover after mission completion:      " << getStringfromBool(hover_after_mission_completion) << '\n'
       << " - Sequence multiple in-flight:         " << getStringfromBool(sequence_multiple_in_flight) << '\n'
       << " - Register RC AUX channels:            " << getStringfromBool(register_aux) << '\n';

    if (!activate_user_interface)
    {
      ss << " - Loaded mission with ID:              " << mission_id_no_ui << '\n';
    }

    if (activate_watchdog)
    {
      ss << " - Subscribed to:                       " << watchdog_heartbeat_topic << '\n';
      ss << " - Subscribed to:                       " << watchdog_status_topic << '\n';
      ss << " - Publishing on:                       " << watchdog_action_topic << '\n';
      ss << " - Service available at:                " << watchdog_start_service_name << '\n';
      ss << " - Watchdog heartbeat timeout:          " << watchdog_timeout << " ms\n";
      ss << " - Watchdog startup time:               " << watchdog_startup_time << " s\n";
    }

    if (estimator_init_service)
    {
      ss << " - Service available at:                " << estimator_init_service_name << '\n';
    }

    if (perform_estimator_check)
    {
      ss << " - Service available at:                " << estimator_supervisor_service_name << '\n';
    }

    if (perform_takeoff_check)
    {
      ss << " - Service available at:                " << takeoff_service_name + '\n';
    }

    if (activate_data_recording)
    {
      ss << " - Service available at:                " << data_recrding_service_name << '\n';
      ss << " - Delay after failure:                 " << data_recording_delay_after_failure_s << " s\n";
    }

    if (inflight_sensors_init_service)
    {
      for (const auto& it : inflight_sensor_init_services_name)
      {
        ss << " - Service available at:                " << it << '\n';
      }
    }

    if (activate_landing_detection)
    {
      ss << " - Subscribed to:                       " << landing_detection_topic << '\n';
    }

    ss << " - Subscribed to:                       " << mission_sequencer_request_topic << '\n';
    ss << " - Publishing on:                       " << mission_sequencer_response_topic << '\n';
    ss << " - Publishing on:                       " << mission_sequencer_waypoints_topic << '\n';

    ss << " - Subscribed to:                       " << rc_topic << '\n';

    ss << " - Maximum flight time:                 " << flight_timeout / 60000 << " min\n";

    ss << " - Max time for a sensor fix:           " << fix_timeout << " ms\n";
    ss << " - Max time for a sensor fix preflight: " << preflight_fix_timeout << " s\n";

    ss << " - Landing AUX channel:                 " << landing_aux_channel << '\n';

    ss << " - Display Logging Level:               " << log_display_level << '\n';
    if (log_display_level >= LogDisplayLevel::WAYPOINT)
    {
      ss << " - Subscribed to:                       " << mission_sequencer_waypoint_reached_topic << '\n';
    }
    if (log_display_level >= LogDisplayLevel::SYSTEM)
    {
      ss << " - Subscribed to:                       " << battery_status_topic << '\n';
      ss << " - Battery Status Interval (%):         " << battery_voltage_interval_percent * 100 << '\n';
    }

    ss << "-------------------------------------------------\n\n";

    return ss.str();
  }

  inline const std::string getStringfromBool(const bool& flag) const
  {
    return flag ? "Active" : "Inactive";
  }

};  // struct AutonomyOptions

}  // namespace autonomy

#endif  // AUTONOMY_OPTIONS_H
