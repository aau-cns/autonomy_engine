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

#include "autonomy_core/autonomy.h"

#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>

#include "state_machine/states/end_mission.h"
#include "state_machine/states/failure.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/initialization.h"
#include "state_machine/states/land.h"
#include "state_machine/states/mission_iterator.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/perform_mission.h"
#include "state_machine/states/preflight.h"
#include "state_machine/states/start_mission.h"
#include "state_machine/states/termination.h"
#include "state_machine/states/undefined.h"
#include "utils/colors.h"
#include "utils/format.h"

namespace std
{
/**
 * @brief Overload std::to_string for std::string
 */
static string to_string(const string& str)
{
  return str;
}
}  // namespace std

namespace autonomy
{
Autonomy::Autonomy(ros::NodeHandle& nh) : logger_(nh), nh_(nh)
{
  // Setting state to UNDEFINED
  state_ = &Undefined::Instance();

  // Get actual time
  time_t now = time(nullptr);
  tm* ltm = localtime(&now);

  // Parse parameters and options
  parseParams();

  // Setup LogDisplay Level
  logger_.setLogDisplayLevel(opts_->log_display_level);

  std::ostringstream ss;
  ss << "autonomy-" << std::setw(4) << std::setfill('0') << (1900 + ltm->tm_year) << "-" << std::setw(2)
     << std::setfill('0') << int(1 + ltm->tm_mon) << "-" << std::setw(2) << std::setfill('0') << int(ltm->tm_mday)
     << "-" << std::setw(2) << std::setfill('0') << int(ltm->tm_hour) << "-" << std::setw(2) << std::setfill('0')
     << int(ltm->tm_min) << "-" << std::setw(2) << std::setfill('0') << int(ltm->tm_sec) << ".log";
  logger_.initFileLogger(std::string(opts_->logger_filepath) + ss.str());

  // Init message
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatInitMsg());

  // Print option
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), opts_->printAutonomyOptions());

  // Advertise watchdog service and action topic and instanciate timer
  if (opts_->activate_watchdog)
  {
    watchdog_start_service_client_ = nh_.serviceClient<watchdog_msgs::Start>(opts_->watchdog_start_service_name);
    pub_watchdog_action_ = nh.advertise<watchdog_msgs::ActionStamped>(opts_->watchdog_action_topic, 10);
    watchdog_timer_ = std::make_unique<Timer>(opts_->watchdog_timeout);
    watchdog_timer_->sh_.connect(boost::bind(&Autonomy::watchdogTimerOverflowHandler, this));
  }

  // Advertise estimator init service
  if (opts_->estimator_init_service)
  {
    estimator_init_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->estimator_init_service_name);
  }

  // Advertise in flight sensor init service
  if (opts_->inflight_sensors_init_service)
  {
    for (size_t i = 0; i < opts_->inflight_sensor_init_services_name.size(); ++i)
    {
      inflight_sensor_init_service_client_.emplace_back(
          nh_.serviceClient<std_srvs::Empty>(opts_->inflight_sensor_init_services_name.at(i)));
    }
  }

  // Advertise estimator supervisor service
  if (opts_->perform_estimator_check)
  {
    estimator_supervisor_service_client_ =
        nh_.serviceClient<std_srvs::Trigger>(opts_->estimator_supervisor_service_name);
  }

  // Advertise takeoff service
  if (opts_->perform_takeoff_check)
  {
    takeoff_service_client_ = nh_.serviceClient<std_srvs::Trigger>(opts_->takeoff_service_name);
  }

  // Advertise data recording service
  if (opts_->activate_data_recording)
  {
    data_recording_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->data_recrding_service_name);
  }

  // Advertise mission sequencer request topic
  pub_mission_sequencer_request_ =
      nh.advertise<mission_sequencer::MissionRequest>(opts_->mission_sequencer_request_topic, 10);

  // Advertise mission sequencer waypoints topic
  pub_mission_sequencer_waypoints_ =
      nh.advertise<mission_sequencer::MissionWaypointArray>(opts_->mission_sequencer_waypoints_topic, 10);

  // Subscribe to mission sequencer response
  sub_mission_sequencer_response_ =
      nh_.subscribe(opts_->mission_sequencer_response_topic, 100, &Autonomy::missionSequencerResponceCallback, this);

  // Subscribe to RC
  sub_rc_ = nh_.subscribe(opts_->rc_topic, 100, &Autonomy::rcCallback, this);

  // Subscribe to info callbacks (for logging only)
  if (opts_->log_display_level >= LogDisplayLevel::WAYPOINT)
  {
    sub_ms_waypoint_reached_ = nh_.subscribe(opts_->mission_sequencer_waypoint_reached_topic, 1,
                                             &Autonomy::missionSequencerWaypointReachedCallback, this);

    if (opts_->log_display_level >= LogDisplayLevel::SYSTEM)
    {
      sub_battery_ = nh_.subscribe(opts_->battery_status_topic, 1, &Autonomy::batteryCallback, this);
    }
  }

  // Instanciate flight timer and connect signal
  flight_timer_ = std::make_unique<Timer>(opts_->flight_timeout);
  flight_timer_->sh_.connect(boost::bind(&Autonomy::flightTimerOverflowHandler, this));

  // Instanciate waypoints_parser_ with categories {"x", "y", "z", "yaw", "holdtime"}
  waypoints_parser_ = std::make_unique<WaypointsParser>();

  logger_.logInfo(state_->getStringFromState(), "Started autonomy");
}

template <typename T>
void Autonomy::getParameter(T& param, const std::string& name, const std::string& msg)
{
  if (!nh_.getParam(name, param))
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE), formatParamNotFound(name, msg));
    logger_.logInfo(state_->getStringFromState(), "[" + name + "] parameter not defined");
    stateTransition("failure");
  }
}

template <typename T>
void Autonomy::getParameterDefault(T& param, const std::string& name, const T& value, const std::string& msg)
{
  if (!nh_.getParam(name, param))
  {
    param = value;
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), formatParamNotFound(name, msg));
    logger_.logInfo(state_->getStringFromState(),
                    "[" + name + "] parameter not defined, using " + std::to_string(value) + ".");
  }
}

void Autonomy::getMissions()
{
  // Define auxilliary variables foreach paramter: XmlRpc::XmlRpcValue
  XmlRpc::XmlRpcValue XRV_missions;
  XmlRpc::XmlRpcValue XRV_filepaths;
  XmlRpc::XmlRpcValue XRV_entities_states;

  // Get specific parameter relative to the mission
  getParameter(XRV_missions, "missions");

  // Check mission type
  if (XRV_missions.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatParamNotFound("missions", "Please check the specified mission config file"));
    stateTransition("failure");
  }

  // Check whether missions are defined and parse them
  if (XRV_missions.size() == 0)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatParamWrong("missions", "No missions defined on the specified mission config file"));
    stateTransition("failure");
  }
  else
  {
    // Loop through missions
    for (int i = 1; i <= XRV_missions.size(); ++i)
    {
      // Declare mission specific variables
      std::string description, state;
      std::vector<std::string> filepaths;
      Entity entity;
      std::map<Entity, std::string> entity_state_map;
      int instances;

      // Get mission parmaters
      getParameter(description, "missions/mission_" + std::to_string(i) + "/description");
      getParameter(XRV_filepaths, "missions/mission_" + std::to_string(i) + "/filepaths");

      // Check type to be array
      if (XRV_filepaths.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        // Loop through the filepaths
        for (int j = 0; j < XRV_filepaths.size(); ++j)
        {
          // Check type to be string
          if (XRV_filepaths[j].getType() == XmlRpc::XmlRpcValue::TypeString)
          {
            // assign filepath
            std::string full_path = std::string(opts_->trajectory_dir) + std::string(XRV_filepaths[j]);
            filepaths.emplace_back(opts_->trajectory_dir + std::string(XRV_filepaths[j]));
            //            filepaths.emplace_back(std::string(XRV_filepaths[j]));
          }
          else
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                          formatParamWrong("mission_" + std::to_string(i) + "/filepath", "wrongly defined in filepaths "
                                                                                         "list"));
            stateTransition("failure");
          }
        }
      }
      else
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatParamWrong("mission_" + std::to_string(i) + "/filepaths", "wrongly defined, it must be a "
                                                                                      "list"));
        stateTransition("failure");
      }

      // get the mission repetitions
      getParameterDefault(instances, "missions/mission_" + std::to_string(i) + "/instances", 1,
                          "Defaulting to single instance.");

      // Check value of instances of the mission
      // If it is less then -1 or 0 trigger a failure
      if (instances == 0 || instances < -1)
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatParamWrong("mission_" + std::to_string(i) + "/repetitions", "wrongly defined, it must be a "
                                                                                        "-1 or grater than 0"));
        stateTransition("failure");
      }

      // Get entities and next states
      getParameter(XRV_entities_states, "missions/mission_" + std::to_string(i) + "/entities_actions");

      // Check type to be array
      if (XRV_entities_states.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        // Loop through entities and actions
        for (int k = 0; k < XRV_entities_states.size(); ++k)
        {
          // Check type to be array
          if (XRV_entities_states[k].getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            // Check type to be string and get entity
            if (XRV_entities_states[k][0].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
              if (!getEntityFromString(std::string(XRV_entities_states[k][0]), entity))
              {
                logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                              formatParamWrong("mission_" + std::to_string(i) + "/entity", "wrongly defined in "
                                                                                           "entities_actions list"));
                stateTransition("failure");
              }
            }

            // Check type to be string and get action
            if (XRV_entities_states[k][1].getType() == XmlRpc::XmlRpcValue::TypeString)
            {
              // Check the existance of predefined state for the given string action
              if (!checkStateFromString(std::string(XRV_entities_states[k][1])))
              {
                logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                              formatParamWrong("mission_" + std::to_string(i) + "/action", "wrongly defined in "
                                                                                           "entities_actions list"));
                stateTransition("failure");
              }
              else
              {
                state = std::string(XRV_entities_states[k][1]);
              }
            }

            // Build the entity_state_map
            entity_state_map.try_emplace(entity, state);
          }
          else
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                          formatParamWrong("mission_" + std::to_string(i) + "/entitiy_action", "wrongly defined in "
                                                                                               "entities_actions "
                                                                                               "list"));
            stateTransition("failure");
          }
        }
      }
      else
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatParamWrong("mission_" + std::to_string(i) + "/entities_actions", "wrongly defined"));
        stateTransition("failure");
      }

      // Build the mission
      missions_.try_emplace(
          i, Mission(i, description, filepaths, entity_state_map,
                     instances == -1 ? std::numeric_limits<float>::infinity() : static_cast<float>(instances)));
    }
  }
}

void Autonomy::parseParams()
{
  // Define auxilliary variables foreach paramter: std::String
  std::string estimator_supervisor_service_name;
  std::string watchdog_start_service_name;
  std::string watchdog_heartbeat_topic;
  std::string watchdog_status_topic;
  std::string watchdog_action_topic;
  std::string mission_sequencer_request_topic;
  std::string mission_sequencer_response_topic;
  std::string data_recrding_service_name;
  std::string takeoff_service_name;
  std::string landing_detection_topic;
  std::string estimator_init_service_name;
  std::string mission_sequencer_waypoints_topic;
  std::string mission_sequencer_waypoint_reached_topic;
  std::string logger_filepath;
  std::string trajectory_dir;
  std::string rc_topic;
  std::string battery_status_topic;

  // Define auxilliary variables foreach paramter: std::vector
  std::vector<std::string> inflight_sensor_init_services_name;

  // Define auxilliary variables foreach paramter: float
  float watchdog_rate = 0.0;
  float watchdog_heartbeat_timeout_multiplier = 0.0;

  // Define auxilliary variables foreach paramter: int
  int watchdog_startup_time_s = 0;
  int watchdog_timeout_ms = 0;
  int flight_timeout_ms = 0;
  int fix_timeout_ms = 0;
  int preflight_fix_timeout_ms = 0;
  int data_recording_delay_after_failure_s = 0;
  int mission_id_no_ui = -1;
  int landing_aux_channel = -1;

  // Define auxilliary variables foreach paramter: bool
  bool activate_user_interface;
  bool activate_watchdog;
  bool activate_data_recording;
  bool estimator_init_service;
  bool perform_takeoff_check;
  bool perform_estimator_check;
  bool activate_landing_detection;
  bool hover_after_mission_completion;
  bool sequence_multiple_in_flight;
  bool inflight_sensors_init_service;
  bool register_aux;

  // Define auxilliary variables foreach logging parameter: various
  LogDisplayLevel log_display_level = LogDisplayLevel::BASIC;
  float battery_voltage_interval_percent = 0.05;

  // Get general parmaters from ros param server
  getParameter(activate_user_interface, "activate_user_interface");
  getParameter(activate_watchdog, "activate_watchdog");
  getParameter(activate_data_recording, "activate_data_recording");
  getParameter(estimator_init_service, "estimator_init_service");
  getParameter(perform_takeoff_check, "perform_takeoff_check");
  getParameter(perform_estimator_check, "perform_estimator_check");
  getParameter(activate_landing_detection, "activate_landing_detection");
  getParameter(hover_after_mission_completion, "hover_after_mission_completion");
  getParameter(sequence_multiple_in_flight, "sequence_multiple_in_flight");
  getParameter(inflight_sensors_init_service, "inflight_sensors_init_service");
  getParameter(mission_sequencer_request_topic, "mission_sequencer_request_topic");
  getParameter(mission_sequencer_response_topic, "mission_sequencer_response_topic");
  getParameter(mission_sequencer_waypoints_topic, "mission_sequencer_waypoints_topic");
  getParameter(logger_filepath, "logger_filepath");
  getParameter(trajectory_dir, "trajectory_dir");
  getParameter(flight_timeout_ms, "maximum_flight_time_min");
  getParameter(fix_timeout_ms, "fix_timeout_ms");
  getParameter(preflight_fix_timeout_ms, "preflight_fix_timeout_ms");
  getParameter(data_recording_delay_after_failure_s, "data_recording_delay_after_failure_s");
  getParameter(rc_topic, "rc_topic");
  getParameter(register_aux, "register_aux");

  // Convert given flight time (in minutes) from minutes to milliseconds
  flight_timeout_ms *= 60000;

  // Get specific parameter if user interface is not active
  if (!activate_user_interface)
  {
    getParameter(mission_id_no_ui, "mission_id_no_ui");
  }

  // Get specific parameter if watchdog is not active
  if (activate_watchdog)
  {
    getParameter(watchdog_start_service_name, "watchdog_start_service_name");
    getParameter(watchdog_heartbeat_topic, "watchdog_heartbeat_topic");
    getParameter(watchdog_status_topic, "watchdog_status_topic");
    getParameter(watchdog_action_topic, "watchdog_action_topic");
    getParameter(watchdog_rate, "watchdog_rate");
    getParameter(watchdog_heartbeat_timeout_multiplier, "watchdog_heartbeat_timeout_multiplier",
                 "This set the heartbeat timer timeout to be a scaled version of the period of the watchdog rate. "
                 "Please set it higher than 1.0");
    if (watchdog_heartbeat_timeout_multiplier >= 1)
    {
      // Set watchdog timer timeout to be n/watchdog_rate ms (n >= 1)
      watchdog_timeout_ms = static_cast<int>(std::ceil((1000 * watchdog_heartbeat_timeout_multiplier) / watchdog_rate));
    }
    else
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                    formatParamWrong("watchdog_heartbeat_timeot_multiplier", "paramter smaller than 1.0. Please set it "
                                                                             "higher than 1.0"));
      stateTransition("failure");
    }
    getParameter(watchdog_heartbeat_timeout_multiplier, "watchdog_heartbeat_timeout_multiplier");
    getParameter(watchdog_startup_time_s, "watchdog_startup_time_s", "Please set it higher than 10");
  }

  // Get specific parameter if data recording is active
  if (activate_data_recording)
  {
    getParameter(data_recrding_service_name, "data_recrding_service_name");
  }

  // Get specific parameter if estimator init is active
  if (estimator_init_service)
  {
    getParameter(estimator_init_service_name, "estimator_init_service_name");
  }

  // Get specific parameter if in-flight sensor init is active
  if (inflight_sensors_init_service)
  {
    nh_.param<std::vector<std::string>>("inflight_sensor_init_services_name", inflight_sensor_init_services_name,
                                        std::vector<std::string>{ "" });
    for (const auto& it : inflight_sensor_init_services_name)
    {
      if (it == "")
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatParamNotFound("inflight_sensor_init_service_name"));
        stateTransition("failure");
      }
    }
  }

  // Get specific parameter if takeoff checks are active
  if (perform_takeoff_check)
  {
    getParameter(takeoff_service_name, "takeoff_service_name");
  }

  // Get specific parameter if estimator check is active
  if (perform_estimator_check)
  {
    getParameter(estimator_supervisor_service_name, "estimator_supervisor_service_name");
  }

  // Get specific parameter if landing detection is active
  if (activate_landing_detection)
  {
    getParameter(landing_detection_topic, "landing_detection_topic");
  }

  // Get aux channels
  getParameter(landing_aux_channel, "landing_aux_channel");

  // Get Log Display level
  int ldl;
  getParameterDefault<int>(ldl, "log_display_level", 0);
  log_display_level = static_cast<LogDisplayLevel>(ldl);

  // get parameters depending on log display level
  if (log_display_level >= LogDisplayLevel::WAYPOINT)
  {
    // Get mission sequencer waypoint reached topic only if log display level for waypoints is enabled
    getParameterDefault<std::string>(mission_sequencer_waypoint_reached_topic,
                                     "mission_sequencer_waypoint_reached_topic", "/mission_sequencer/waypoint_reached");
  }

  if (log_display_level >= LogDisplayLevel::SYSTEM)
  {
    // Get battery status topic
    getParameterDefault<std::string>(battery_status_topic, "battery_status_topic", "/mavros/battery_status");

    // Get battery status update level
    getParameterDefault<float>(battery_voltage_interval_percent, "battery_voltage_interval_percent",
                               battery_voltage_interval_percent);
    // check if input is in percent or fraction
    if (battery_voltage_interval_percent > 1.0)
      battery_voltage_interval_percent /= 100.0;
  }

  // Make options
  opts_ = std::make_unique<autonomyOptions>(autonomyOptions({ watchdog_heartbeat_topic,
                                                              watchdog_status_topic,
                                                              watchdog_action_topic,
                                                              mission_sequencer_request_topic,
                                                              mission_sequencer_response_topic,
                                                              landing_detection_topic,
                                                              mission_sequencer_waypoints_topic,
                                                              mission_sequencer_waypoint_reached_topic,
                                                              rc_topic,
                                                              battery_status_topic,
                                                              watchdog_start_service_name,
                                                              estimator_supervisor_service_name,
                                                              data_recrding_service_name,
                                                              takeoff_service_name,
                                                              estimator_init_service_name,
                                                              logger_filepath,
                                                              trajectory_dir,
                                                              inflight_sensor_init_services_name,
                                                              watchdog_timeout_ms,
                                                              flight_timeout_ms,
                                                              fix_timeout_ms,
                                                              preflight_fix_timeout_ms,
                                                              watchdog_startup_time_s,
                                                              data_recording_delay_after_failure_s,
                                                              activate_user_interface,
                                                              activate_watchdog,
                                                              activate_data_recording,
                                                              estimator_init_service,
                                                              perform_takeoff_check,
                                                              perform_estimator_check,
                                                              activate_landing_detection,
                                                              inflight_sensors_init_service,
                                                              register_aux,
                                                              hover_after_mission_completion,
                                                              sequence_multiple_in_flight,
                                                              mission_id_no_ui,
                                                              static_cast<size_t>(landing_aux_channel),
                                                              log_display_level,
                                                              battery_voltage_interval_percent }));

  // Get missions
  getMissions();
}

bool Autonomy::getSensorStatusFromMsg(const watchdog_msgs::Status& msg, SensorStatus& status)
{
  // Get debug infos
  status.debug_name = msg.name;
  status.debug_info = msg.info;

  // Get entity
  if (!getEntityFromString(msg.entity, status.entity))
  {
    return false;
  }

  // Get type and action (based on type and mission specification)
  // the action will be different from NOTHING only if a hold status is required
  switch (msg.type)
  {
    case watchdog_msgs::Status::GLOBAL:
      status.type = Type::GLOBAL;
      status.action = Action::NOTHING;
      break;
    case watchdog_msgs::Status::TOPIC:
      status.type = Type::TOPIC;
      status.action = Action::NOTHING;
      break;
    case watchdog_msgs::Status::NODE:
      status.type = Type::NODE;
      if (missions_.at(mission_id_).getNextState(status.entity).compare("hold") == 0)
      {
        status.action = Action::FIX_NODE;
      }
      else
      {
        status.action = Action::NOTHING;
      }
      break;
    case watchdog_msgs::Status::DRIVER:
      status.type = Type::DRIVER;
      if (missions_.at(mission_id_).getNextState(status.entity).compare("hold") == 0)
      {
        status.action = Action::FIX_DRIVER;
      }
      else
      {
        status.action = Action::NOTHING;
      }
      break;
  }

  // Get event. Increase pending failure in case of failure,
  // reduce pending failures in case of fix
  if (msg.status == watchdog_msgs::Status::ERROR)
  {
    status.event = Event::ENTITY_FAILURE;

    // Increase pending failures in case of hold and start a timer if we are fliying
    // if the error needs to be fixed in the specified mission
    if (missions_.at(mission_id_).getNextState(status.entity).compare("hold") == 0)
    {
      pending_failures_.emplace_back(std::make_pair(status, std::make_unique<Timer>(opts_->fix_timeout)));
      pending_failures_.back().second->sh_.connect(boost::bind(&Autonomy::failureTimerOverflowHandler, this));
      if (in_flight_)
      {
        pending_failures_.back().second->resetTimer();
      }
      // Increase pending failures in case of land only if we are not fliying
    }
    else if (!in_flight_ && missions_.at(mission_id_).getNextState(status.entity).compare("land") == 0)
    {
      pending_failures_.emplace_back(std::make_pair(status, std::make_unique<Timer>(opts_->fix_timeout)));
      pending_failures_.back().second->sh_.connect(boost::bind(&Autonomy::failureTimerOverflowHandler, this));
    }
  }
  else if (msg.status == watchdog_msgs::Status::NOMINAL || msg.status == watchdog_msgs::Status::DEFECT)
  {
    // search and remove fixed failure from pending failures, the fix must be the consequence of an action of fixing
    // which can only be triggered if the hold status is requested. If for some reason we got a NOMINAL status without
    // requiring a specific action we set the event to OTHER
    const auto& it = std::find_if(pending_failures_.begin(), pending_failures_.end(),
                                  [&status](const std::pair<SensorStatus, std::unique_ptr<Timer>>& failure) {
                                    return failure.first.isSame(status);
                                  });

    if (it != pending_failures_.end())
    {
      it->second->stopTimer();
      pending_failures_.erase(it);
      status.event = Event::ENTITY_FIX;
    }
    else
    {
      status.event = Event::ENTITY_OTHER;
    }
  }
  else
  {
    status.event = Event::ENTITY_OTHER;
  }

  return true;
}

bool Autonomy::getRequestfromMsg(const mission_sequencer::MissionRequest& msg, std::string& request_str)
{
  // Get request
  switch (msg.request)
  {
    case mission_sequencer::MissionRequest::ABORT:
      request_str = "abort";
      break;
    case mission_sequencer::MissionRequest::ARM:
      request_str = "arm";
      break;
    case mission_sequencer::MissionRequest::TAKEOFF:
      request_str = "takeoff";
      break;
    case mission_sequencer::MissionRequest::HOLD:
      request_str = "hold";
      break;
    case mission_sequencer::MissionRequest::RESUME:
      request_str = "resume";
      break;
    case mission_sequencer::MissionRequest::LAND:
      request_str = "land";
      break;
    case mission_sequencer::MissionRequest::HOVER:
      request_str = "hover";
      break;
    case mission_sequencer::MissionRequest::UNDEF:
      request_str = "undef";
      break;
    case mission_sequencer::MissionRequest::DISARM:
      request_str = "disarm";
      break;
    default:
      return false;
  }
  return true;
}

void Autonomy::watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStampedConstPtr& msg)
{
  // Loop through all the chenges with respect to last iteration
  for (const auto& it : msg->data.changes)
  {
    // Define status
    SensorStatus status;

    // Assign time information
    status.timestamp = msg->header.stamp.toSec();

    // Parse information of status changes
    if (getSensorStatusFromMsg(it, status))
    {
      std::string code = std::to_string(status.entity) + std::to_string(status.type) + std::to_string(status.event);
      logger_.logMessage(state_->getStringFromState(), opts_->watchdog_status_topic, true,
                         "[watchdog status message] EntityTypeEvent code: " + code);

      // Define auxilliary strings
      std::string entity;
      std::string type;

      // Check event
      if (status.event != Event::ENTITY_OTHER)
      {
        if (status.event == Event::ENTITY_FAILURE)
        {
          // Redundant check, this should never fail.
          // If a failure happend then not all the possible choices are covered by getStringFromEntity
          if (!getStringFromEntity(status.entity, entity))
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                          formatMsg("[watchdogStatusCallback][FAILURE] No string defined for required entity: " +
                                    std::to_string(status.entity)));
          }

          // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
          if (!getStringFromType(status.type, type))
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                          formatMsg("[watchdogStatusCallback][FAILURE] No string defined for required type: " +
                                    std::to_string(status.type)));
          }

          // Print failure report
          logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                        formatFailure(code, entity, type));

          // Failure -- search an action (next state) on specific mission (mission id) [Next states strings can be:
          // continue, hold, land, failure] If the state string is not failure and we are not armed yet then do not
          // perform any state transition
          if (missions_.at(mission_id_).getNextState(status.entity).compare("failure") != 0)
          {
            // check if next state is continue, then we only log the failure and continue
            if (missions_.at(mission_id_).getNextState(status.entity).compare("continue") == 0)
            {
              logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                            formatMsg("Failure logged,; Action: continue -> continuing..."));
            }
            // otherwise check if we are flying to execute state transition
            else if (in_flight_)
            {
              stateTransition(missions_.at(mission_id_).getNextState(status.entity));
            }
          }
          else
          {
            stateTransition("failure");
          }

          // Always perform an action to react to the failure (the action can be NOTHING)
          watchdogActionRequest(status, it);
        }
        else if (status.event == Event::ENTITY_FIX)
        {
          // Redundant check, this should never fail, if so not all the possible choices are coverd by
          // getStringFromEntity
          if (!getStringFromEntity(status.entity, entity))
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                          formatMsg("[watchdogStatusCallback][FIX] No string defined for required entity: " +
                                    std::to_string(status.entity)));
          }

          // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
          if (!getStringFromType(status.type, type))
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                          formatMsg("[watchdogStatusCallback][FIX] No string defined for required type: " +
                                    std::to_string(status.type)));
          }

          // Print fix report
          logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatFix(entity, type));

          // Fix -- At this stage the pending failure relative to the fix has already been removed.
          // Check if pending failure size == 0 then stop holding and resulme the mission otherwise keep holding
          if (pending_failures_.size() == 0)
          {
            // Check if we are in mission (we could have a fix before taking off)
            if (in_mission_)
            {
              // Call state transition to PERFORM_MISSION
              stateTransition("perform_mission");

              // Resume mission
              missionSequencerRequest(mission_sequencer::MissionRequest::RESUME);
            }
            else
            {
              logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                            formatMsg("[watchdogStatusCallback][FIX] Received Fix while not flying"));
            }
          }
        }
      }
      else if (it.status == watchdog_msgs::Status::DEFECT)
      {
        // Redundant check, this should never fail, if so not all the possible choices are coverd by
        // getStringFromEntity
        if (!getStringFromEntity(status.entity, entity))
        {
          std::string info = "[watchdogStatusCallback][DEFECT] No string defined for required entity: " +
                             std::to_string(status.entity);
          logger_.logInfo(state_->getStringFromState(), info);
        }

        // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
        if (!getStringFromType(status.type, type))
        {
          std::string info =
              "[watchdogStatusCallback][DEFECT] No string defined for required type: " + std::to_string(status.type);
          logger_.logInfo(state_->getStringFromState(), info);
        }

        // Log the Defect
        std::string defect = "Error code: " + code + ", Entity: " + entity + ", Type: " + type;
        logger_.logInfo(state_->getStringFromState(), "[watchdogStatusCallback][DEFECT] " + defect);
      }
    }
    else
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                    formatMsg("[watchdogStatusCallback] Wrong message received from watchdog"));
    }
  }
}

void Autonomy::watchdogActionRequest(SensorStatus& status, const watchdog_msgs::Status& status_msg)
{
  // Check existence of subscribers
  if (pub_watchdog_action_.getNumSubscribers() > 0)
  {
    // Define action and action message
    watchdog_msgs::ActionStamped action_msg;

    // Fill action message
    action_msg.header.stamp = ros::Time::now();
    action_msg.action.entity = status_msg;

    switch (status.action)
    {
      case Action::NOTHING:
        action_msg.action.action = watchdog_msgs::Action::NOTHING;
        break;
      case Action::FIX_NODE:
        action_msg.action.action = watchdog_msgs::Action::FIX_NODE;
        break;
      case Action::FIX_DRIVER:
        action_msg.action.action = watchdog_msgs::Action::FIX_DRIVER;
        break;
    }

    // UI print
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                  formatMsg("Action communicated to the watchdog"));

    // Log action message
    logger_.logMessage(state_->getStringFromState(), opts_->watchdog_action_topic, false,
                       "[watchdog action message] Action: " + std::to_string(action_msg.action.action));

    // Publish action
    pub_watchdog_action_.publish(action_msg);
  }
  else
  {
    // Print info
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                  formatMsg("No subscribers for watchdog action. Action will be ignored", 2));
  }
}

void Autonomy::landingDetectionCallback(const std_msgs::BoolConstPtr& msg)
{
  if (msg)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Flat land detected"));
    //    logger_.logMessage(state_->getStringFromState(), opts_->landing_detection_topic, true,
    //                       "[Landing detection message] Flat land detected");
  }
  else
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), " >>> Non flat land detected");
    //    logger_.logMessage(state_->getStringFromState(), opts_->landing_detection_topic, true,
    //                       "[Landing detection message] Non-flat land detected");
  }

  // Stop data recording after landing in case mission has been succesfully completed
  if (land_expected_)
  {
    // Reset in_flight_ and land_expected_ flag
    in_flight_ = false;
    land_expected_ = false;

    // Call state transition to END MISSION
    stateTransition("end_mission");
  }
  else
  {
    // Print info
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatMsg("Unexpected touchdown detected"));

    // Call state transition to LAND
    stateTransition("land");
  }
}

void Autonomy::missionSequencerResponceCallback(const mission_sequencer::MissionResponseConstPtr& msg)
{
  // Define request
  std::string req;

  // Get request and check
  if (getRequestfromMsg(msg->request, req))
  {
    logger_.logMessage(state_->getStringFromState(), opts_->mission_sequencer_response_topic, true,
                       "[mission sequencer message] Request: " + std::to_string(msg->request.request) + ", Response: " +
                           std::to_string(msg->response) + ", Completed: " + std::to_string(msg->completed));

    // Check pending request flag
    if (ms_request_pending_)
    {
      // Reset pending request flag
      ms_request_pending_ = false;

      // Check if mission sequencer request has been accepted
      if (msg->response && msg->request.request != mission_sequencer::MissionRequest::UNDEF)
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                      formatMsg("Request [" + req + "] accepted from Mission Sequencer", 2));

        // Acts specifically based on the request
        switch (msg->request.request)
        {
          case mission_sequencer::MissionRequest::ARM:

            break;

          case mission_sequencer::MissionRequest::TAKEOFF:

            // Set in_flight_ flag
            in_flight_ = true;

            break;

          case mission_sequencer::MissionRequest::HOLD:

            // Set holding_ flag
            holding_ = true;

            break;

          case mission_sequencer::MissionRequest::RESUME:

            // Reset holding_ flag
            holding_ = false;

            break;

          case mission_sequencer::MissionRequest::LAND:

            // If we are hovering and a land is requested then reset hovering_ flag
            if (hovering_)
            {
              hovering_ = false;
            }

            break;

          case mission_sequencer::MissionRequest::HOVER:

            // Set hovering_ flag
            hovering_ = true;

            break;

          case mission_sequencer::MissionRequest::DISARM:

            break;
        }
      }

      // Check if mission sequencer request has been rejected
      if (!msg->response && !msg->completed)
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatMsg("Request [" + req + "] for mission ID: " + std::to_string(msg->request.id) +
                                    "  rejected from Mission Sequencer",
                                2));
        stateTransition("failure");
      }
    }
    else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::UNDEF)
    {
      logger_.logUI(
          state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
          formatMsg("Mission ID: " + std::to_string(msg->request.id) + " succesfully reached last waypoint", 2));

      // set last_waypoint_reached_ flag
      last_waypoint_reached_ = true;

      // Check if sequencing of multiple in-air files happens
      // (filepaths counter + 1) * (instance counter + 1) is grater than the number of touchdowns only at the last
      // iteration, thus stop iterating
      size_t t = (filepaths_cnt_ + 1) * (instances_cnt_ + 1);
      if (multiple_touchdowns_ && opts_->sequence_multiple_in_flight &&
          (t <= missions_.at(mission_id_).getTouchdowns()))
      {
        stateTransition("mission_iterator");
      }
      else
      {
        // Check parameter server (this parameter can be changed during runtime)
        bool hover_after_mission_completion;
        nh_.getParam("hover_after_mission_completion", hover_after_mission_completion);

        // Check if the parameter has changed from previously assigned value
        if (hover_after_mission_completion != opts_->hover_after_mission_completion)
        {
          // Assign new flag
          opts_->hover_after_mission_completion = hover_after_mission_completion;
          // Print info
          logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                        formatMsg("Changed behaviour at mission completion: Hover is now " +
                                  opts_->getStringfromBool(opts_->hover_after_mission_completion)));
        }

        // Call state transition to LAND or request HOVER to mission sequencer based on param
        if (!opts_->hover_after_mission_completion)
        {
          stateTransition("land");
        }
        else
        {
          // Print info
          logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Hovering..."));

          // Request hovering to the mission sequencer if not hovering
          if (!hovering_)
          {
            missionSequencerRequest(mission_sequencer::MissionRequest::HOVER);
          }
          else
          {
            logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                          formatMsg("the platform is already hovering, skipped HOVER request"));
          }
        }
      }
    }
    else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::ARM)
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Arming completed", 2));

      // Set armed_ flag
      armed_ = true;

      // Reset flight_timer only at first arming (if not active)
      if (!flight_timer_->isActive())
      {
        flight_timer_->resetTimer();
      }
    }
    else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::TAKEOFF)
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Takeoff completed", 2));

      // Set in_mission_ and last_waypoint_reached_ flag
      in_mission_ = true;
      last_waypoint_reached_ = false;

      // Subscribe to landing after taking off if detection if active
      if (opts_->activate_landing_detection)
      {
        sub_landing_detection_ =
            nh_.subscribe(opts_->landing_detection_topic, 1, &Autonomy::landingDetectionCallback, this);
      }
    }
    else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::LAND)
    {
      // Check if landing detection is active, if not transit to END_MISSION
      if (!opts_->activate_landing_detection && land_expected_)
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Landing completed"));

        // Reset in_flight_ and land_expected_ flag
        in_flight_ = false;
        land_expected_ = false;

        // Call state transition to END MISSION
        stateTransition("end_mission");
      }
    }
    else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::DISARM)
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg("Disarming completed"));

      // Reset armed_ and in_flight_ flag
      armed_ = false;

      // Stop flight timer.
      // If doing multiple touchdown stop the timer when the filepaths counter is equal to the number of filepaths -1,
      // and when the istances counter is equal to the number of instances -1 (since both counters start at 0)
      if (!multiple_touchdowns_ ||
          (filepaths_cnt_ == (missions_.at(mission_id_).getFilepaths().size() - 1) &&
           instances_cnt_ == (static_cast<size_t>(missions_.at(mission_id_).getInstances()) - 1)))
      {
        flight_timer_->stopTimer();
      }
    }
    else
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                    formatMsg("Received response from mission sequencer without a prior request. Ignoring it", 2));
    }
  }
  else
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                  formatMsg("Received response from mission sequencer to a unknown request. Ignoring it", 2));
  }
}

void Autonomy::missionSequencerWaypointReachedCallback(const mission_sequencer::MissionWaypointStampedConstPtr& msg)
{
  // log message received
  logger_.logMessage(state_->getStringFromState(), opts_->mission_sequencer_waypoint_reached_topic, true,
                     "[mission sequencer message] waypoint reached received");

  // format msg
  std::stringstream ss;
  ss << "Waypoint [" << std::fixed << std::setprecision(3) << msg->waypoint.x << ", " << msg->waypoint.y << ", "
     << msg->waypoint.z << ", " << msg->waypoint.yaw << "] reached."
     << "\n"
     << "       Holding at this waypoint for " << msg->waypoint.holdtime << "s..";

  logger_.logUI(state_->getStringFromState(), GREEN_ESCAPE, formatMsg(ss.str(), 1), LogDisplayLevel::WAYPOINT);
}

void Autonomy::batteryCallback(const mavros_msgs::BatteryStatusConstPtr& msg)
{
  // log message received
  logger_.logMessage(state_->getStringFromState(), opts_->battery_status_topic, true,
                     "[mavros message] battery status received");

  if (msg->remaining < last_battery_voltage_percent_ - opts_->battery_voltage_interval_percent)
  {
    // format msg
    std::stringstream ss1, ss2;
    ss1 << "Reported battery percentage (" << (int)(msg->remaining * 100) << "%) < "
        << (int)(100 * (last_battery_voltage_percent_ - opts_->battery_voltage_interval_percent)) << "%";
    ss2 << "Voltage: " << std::fixed << std::setprecision(2) << msg->voltage << "V";

    logger_.logUI(state_->getStringFromState(), MAGENTA_ESCAPE, formatMsg(ss1.str(), 1), LogDisplayLevel::SYSTEM);
    logger_.logInfo(state_->getStringFromState(), ss2.str());

    last_battery_voltage_percent_ -= opts_->battery_voltage_interval_percent;
  }

  // TODO(scm): new feature to add saftey landing if battery remaining or voltage below a certain threshold
}

void Autonomy::missionSequencerRequest(const int& request)
{
  // Check existence of subscribers
  if (pub_mission_sequencer_request_.getNumSubscribers() > 0)
  {
    // Check to not be in failure state
    if (state_->getStringFromState().compare("failure") != 0)
    {
      // Define request message to mission sequencer
      mission_sequencer::MissionRequest req;

      // Set mission id and request
      req.header.stamp = ros::Time::now();
      req.id = uint8_t(mission_id_);
      req.request = uint8_t(request);

      // publish mission start request
      logger_.logMessage(state_->getStringFromState(), opts_->mission_sequencer_request_topic, false,
                         "[mission sequencer message] Request: " + std::to_string(req.request));
      pub_mission_sequencer_request_.publish(req);

      // Set flag
      ms_request_pending_ = true;
    }
    else
    {
      // Print info
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                    formatMsg("No communication allowed to Mission sequencer from FAILURE"));
    }
  }
  else
  {
    // Print info
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatMsg("No subscribers for mission sequencer request", 2));

    // At this stage we are not flying yet, state transition to FAILURE
    stateTransition("failure");
  }
}

void Autonomy::rcCallback(const mavros_msgs::RCInConstPtr& msg)
{
  // Check channels field is not empty
  if (register_aux_ && !aux_registered_)
  {
    if (!msg->channels.empty())
    {
      // Register aux
      for (size_t id = 0; id < msg->channels.size(); ++id)
      {
        aux_.setValue(id, msg->channels.at(id));
        std::stringstream ss;
        ss << "Registered AUX [" << id << "] with value [" << aux_.getValue(id) << "]";
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatMsg(ss.str(), 1));
      }

      // Set aux_registered_
      aux_registered_ = true;
    }
  }

  if (aux_registered_ && register_aux_)
  {
    // Check for changes with tollerance of 50
    // for (size_t id = 0; id < msg->channels.size(); ++id)
    // {
    size_t id = opts_->landing_aux_channel;
    if (msg->channels.size() >= id)
    {
      if (std::abs(msg->channels.at(id) - aux_.getValue(id)) > 50)
      {
        // Update aux values
        std::stringstream ss;
        ss << "AUX [" << id << "] changed from [" << aux_.getValue(id) << "] to [" << msg->channels.at(id) << "]";
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), formatMsg(ss.str(), 2));
        aux_.setValue(id, msg->channels.at(id));

        // Check if landing aux changed
        // if (id == opts_->landing_aux_channel)
        // {
        stateTransition("land");
        // }

        // Manage other aux
      }
    }
    // }
  }
}

bool Autonomy::registerRCAux()
{
  // UI AUX registration
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                formatMsg("Please set all the switches on the safety pilot remote, and press [SPACE] and then [ENTER] "
                          "to register the actual position of the switches",
                          0));
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
  logger_.logUserInput(state_->getStringFromState(), "[SPACE]");
  logger_.logUserInput(state_->getStringFromState(), "[ENTER]");
  std::cout << std::endl;

  // Set register_aux_ flag
  register_aux_ = true;

  // Wait for aux being registered (max wait 10 seconds)
  int cnt = 0;
  while (!aux_registered_)
  {
    polling(10);
    if (++cnt == 1000)
    {
      // reset register_aux_ and return
      register_aux_ = false;
      return false;
    }
  }

  // UI
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Remote Switches succesfully registered", 2));

  // reset register_aux_ and return
  register_aux_ = false;
  return true;
}

bool Autonomy::startWatchdog()
{
  // Define service request
  watchdog_msgs::Start watchdog_start;
  watchdog_start.request.header.stamp = ros::Time::now();
  watchdog_start.request.startup_time = opts_->watchdog_startup_time;

  // UI and log
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Starting Watchdog... Please wait", 2));

  // Call service request
  logger_.logServiceCall(state_->getStringFromState(), opts_->watchdog_start_service_name);
  if (watchdog_start_service_client_.call(watchdog_start))
  {
    // Check response
    if (watchdog_start.response.successful)
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                    formatMsg("Watchdog is running", 2));
      logger_.logServiceResponse(state_->getStringFromState(), opts_->watchdog_start_service_name,
                                 "Watchdog started successfully");
      // Subscriber to watchdog (system status) heartbeat
      sub_watchdog_heartbeat_ =
          nh_.subscribe(opts_->watchdog_heartbeat_topic, 1, &Autonomy::watchdogHeartBeatCallback, this);

      // Subscribe to watchdog status changes
      sub_watchdog_status_ = nh_.subscribe(opts_->watchdog_status_topic, 1, &Autonomy::watchdogStatusCallback, this);

      // Success
      return true;
    }
    else
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                    formatMsg("FAILED TO START WATCHDOG - Please perform a system hard restart", 2));
      logger_.logServiceResponse(state_->getStringFromState(), opts_->watchdog_start_service_name,
                                 "Failed to start watchdog");

      // Define status to get debug info
      SensorStatus status;

      // Get timestamp
      status.timestamp = watchdog_start.response.header.stamp.toSec();

      // Log debug info
      if (getSensorStatusFromMsg(watchdog_start.response.status, status))
      {
        // Define auxilliary strings
        std::string entity;
        std::string type;

        // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromEntity
        if (!getStringFromEntity(status.entity, entity))
        {
          logger_.logInfo(state_->getStringFromState(), "[watchdog start failure] getStringFromEntity: no string "
                                                        "deifined for required entity: " +
                                                            std::to_string(status.entity));
        }

        // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
        if (!getStringFromType(status.type, type))
        {
          logger_.logInfo(state_->getStringFromState(), "[watchdog start failure] getStringFromType: no string defined "
                                                        "for required type: " +
                                                            std::to_string(status.type));
        }

        logger_.logInfo(state_->getStringFromState(), "[watchdog start failure] debug name: " + status.debug_name);
        logger_.logInfo(state_->getStringFromState(), "[watchdog start failure] debug info: " + status.debug_info);
      }

      // Failure
      return false;
    }
  }
  else
  {
    // Failure
    return false;
  }
}

void Autonomy::missionSelection()
{
  // If using the user interface
  if (opts_->activate_user_interface)
  {
    // UI Missions selection
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                  formatMsg("Please select one of the following mission by inserting the mission ID", 2));
    for (auto& it : missions_)
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                    formatMsg("\t - ID: " + (RESET + std::to_string(it.first) + ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE)) +
                              '\t' + (RESET + it.second.getDescription())));
    }

    // Get ID of mission being executed
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), "\n >>> Mission ID: ");
    std::cin >> mission_id_;
    logger_.logUserInput(state_->getStringFromState(), std::to_string(mission_id_));
  }
  else
  {
    // Log
    logger_.logInfo(state_->getStringFromState(),
                    "[UI disabled] Choosing mission with ID " + std::to_string(opts_->mission_id_no_ui));

    // Load mission from parameter
    mission_id_ = opts_->mission_id_no_ui;
  }

  // Check validity of mission id
  if (mission_id_ < 1 || mission_id_ > static_cast<int>(missions_.size()))
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE), "\n >>> Wrong mission ID chosen\n\n");

    // Call recursively mission selection
    missionSelection();
  }
  else
  {
    // UI
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                  formatMsg("Loaded mission with ID: " + std::to_string(mission_id_), 2));

    // If mission has multiple touchdowns then print a message
    if (missions_.at(mission_id_).getTouchdowns() > 0)
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                    formatMsg("Mission with Multiple touchdowns: " +
                                  std::to_string(missions_.at(mission_id_).getTouchdowns()) + " touchdown(s)",
                              2));
      multiple_touchdowns_ = true;
    }
  }
}

bool Autonomy::preFlightChecks()
{
  // UI and log
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Starting Pre-Flight Checks..."));

  if (opts_->perform_takeoff_check && !takeoffChecks())
  {
    return false;
  }

  if (opts_->perform_estimator_check && !estimatorCheck())
  {
    return false;
  }

  // Trigger State estimation initialization
  if (opts_->estimator_init_service && !initializeStateEstimation())
  {
    return false;
  }

  // UI and log
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Pre-Flight checks succeeded", 2));

  return true;
}

bool Autonomy::takeoffChecks()
{
  // Define takeoff request
  std_srvs::Trigger takeoff;

  // UI and Log
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Starting Takeoff Checks..."));

  // service call to check if we are ready to takeoff
  if (takeoff_service_client_.call(takeoff))
  {
    logger_.logServiceCall(state_->getStringFromState(), opts_->takeoff_service_name);

    // Check response
    if (takeoff.response.success)
    {
      logger_.logServiceResponse(state_->getStringFromState(), opts_->takeoff_service_name, "takeoff checks succeeded");
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                    formatMsg("Takeoff checks succeeded", 2));
    }
    else
    {
      logger_.logServiceResponse(state_->getStringFromState(), opts_->takeoff_service_name, "Takeoff checks failed");
      takeoff.response.success = false;
    }
  }
  else
  {
    takeoff.response.success = false;
    logger_.logInfo(state_->getStringFromState(), "[Takeoff checks] Failed to call service");
  }

  if (!takeoff.response.success)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE), formatMsg("Takeoff checks failed", 2));
    return false;
  }

  return true;
}

bool Autonomy::estimatorCheck()
{
  // Estimator init UI
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Please, Initialize estimator now"));
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("When done, press [SPACE] and then [ENTER] to start the experiment", 0));
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');
  logger_.logUserInput(state_->getStringFromState(), "[SPACE]");
  logger_.logUserInput(state_->getStringFromState(), "[ENTER]");
  std::cout << std::endl;

  // Define takeoff request
  std_srvs::Trigger superivsion;

  // service call to check if we are ready to takeoff
  if (estimator_supervisor_service_client_.call(superivsion))
  {
    logger_.logServiceCall(state_->getStringFromState(), opts_->estimator_supervisor_service_name);

    // Check response
    if (superivsion.response.success)
    {
      logger_.logServiceResponse(state_->getStringFromState(), opts_->estimator_supervisor_service_name,
                                 "State estimator checks succeeded");
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                    formatMsg("State estimator correctly initialized", 2));
    }
    else
    {
      superivsion.response.success = false;
      logger_.logServiceResponse(state_->getStringFromState(), opts_->estimator_supervisor_service_name,
                                 "State estimator checks failed");
    }
  }
  else
  {
    superivsion.response.success = false;
    logger_.logInfo(state_->getStringFromState(), "[Estimator checks] Failed to call service");
  }

  if (!superivsion.response.success)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatMsg("State estimator initialization failed", 2));
    return false;
  }

  return true;
}

bool Autonomy::initializeStateEstimation()
{
  // Define init service
  std_srvs::SetBool init;

  // Set data recording start request
  init.request.data = true;

  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Initializing state estimator... Please wait"));

  // service call to check if we are ready to takeoff
  if (estimator_init_service_client_.call(init))
  {
    logger_.logServiceCall(state_->getStringFromState(), opts_->estimator_init_service_name);

    // Check response
    if (init.response.success)
    {
      logger_.logServiceResponse(state_->getStringFromState(), opts_->estimator_init_service_name,
                                 "State estimator intialization succeed");
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                    formatMsg("State estimator intialization succeeded", 2));
    }
    else
    {
      init.response.success = false;
      logger_.logServiceResponse(state_->getStringFromState(), opts_->estimator_init_service_name,
                                 "State estimator initialization failed");
    }
  }
  else
  {
    init.response.success = false;
    logger_.logInfo(state_->getStringFromState(), "[State estimator initialization] Failed to call service");
  }

  if (!init.response.success)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatMsg("State estimator initialization failed", 2));
    return false;
  }

  return true;
}

bool Autonomy::InFlightSensorInit()
{
  // Define init service
  std_srvs::Empty init;

  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("Starting in-flight initialization..."));

  // Loop trough all the service call
  for (size_t i = 0; i < inflight_sensor_init_service_client_.size(); ++i)
  {
    // Print info
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                  formatMsg("Calling service: " + opts_->inflight_sensor_init_services_name.at(i)));

    // service call to check if we are ready to takeoff
    logger_.logServiceCall(state_->getStringFromState(), opts_->inflight_sensor_init_services_name.at(i));
    if (!inflight_sensor_init_service_client_.at(i).call(init))
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                    formatMsg(opts_->inflight_sensor_init_services_name.at(i) + " failed"));
      logger_.logServiceResponse(state_->getStringFromState(), opts_->inflight_sensor_init_services_name.at(i),
                                 "Failed in-flight init");
      return false;
    }
    else
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                    formatMsg(opts_->inflight_sensor_init_services_name.at(i) + " succeeded"));
      logger_.logServiceResponse(state_->getStringFromState(), opts_->inflight_sensor_init_services_name.at(i),
                                 "Succeeded in-flight init");
    }
  }

  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                formatMsg("All the sensors have been correctly initialized", 2));
  return true;
}

void Autonomy::DataRecording(const bool& start_stop)
{
  // Define data recording
  std_srvs::SetBool data_rec;

  // Set data recording start/stop request
  data_rec.request.data = start_stop;

  // service call (before stopping check if we are recording)
  if (!start_stop && !is_recording_)
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatMsg("Data recording stop failure, cannot stop data recording if the recording is inactive", 2));
  }
  else
  {
    if (data_recording_service_client_.call(data_rec))
    {
      logger_.logServiceCall(state_->getStringFromState(), opts_->data_recrding_service_name);

      // Check response
      if (data_rec.response.success)
      {
        if (data_rec.request.data)
        {
          logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                        formatMsg("Data recording started successfully", 2));
          logger_.logServiceResponse(state_->getStringFromState(), opts_->data_recrding_service_name,
                                     "Started recording successfully");
          is_recording_ = true;
        }
        else
        {
          logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                        formatMsg("Data recording stopped successfully", 2));
          logger_.logServiceResponse(state_->getStringFromState(), opts_->data_recrding_service_name,
                                     "Stoped recording successfully");
          is_recording_ = false;
        }
      }
      else
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatMsg("Data recording service failure", 2));
        logger_.logServiceResponse(state_->getStringFromState(), opts_->data_recrding_service_name,
                                   "Failure while calling data recorder service");
      }
    }
    else
    {
      if (data_rec.request.data)
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatMsg("Data recording start failure", 2));
        logger_.logServiceResponse(state_->getStringFromState(), opts_->data_recrding_service_name,
                                   "Failure while starting data recorder");
      }
      else
      {
        logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                      formatMsg("Data recording stop failure", 2));
        logger_.logServiceResponse(state_->getStringFromState(), opts_->data_recrding_service_name,
                                   "Failure while stopping data recorder");
      }
    }
  }
}

void Autonomy::startAutonomy()
{
  logger_.logUI("undefined", ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                " >>> Press [ENTER] to start the CNS-FLIGHT Autonomy\n");
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  // Mission selection
  missionSelection();

  // Setting state to INITIALIZATION and initialize the system (watchdog)
  // This will trigger a state transition to either NOMINAL or FAILURE
  // depending by the result of the initialization
  // If the state is set to NOMINAL after succesfull initialization the data
  // recording will start if enabled and the waypoints for the selected mission will be loaded
  stateTransition("initialization");
}

void Autonomy::stateTransition(std::string str)
{
  // Execute predefined behavior on Exiting old state
  state_->onExit(*this);

  auto transitionAllowed = [&]() {
    const auto& allowed = allowed_state_transitions_.at(state_->getStringFromState());
    return std::find(allowed.begin(), allowed.end(), str) != allowed.end() ? true : false;
  };

  // Log
  logger_.logStateChange(state_->getStringFromState(), str);

  // Execute state transition
  if (str.compare("undefined") == 0 && transitionAllowed())
  {
    state_ = &Undefined::Instance();
  }
  else if (str.compare("initialization") == 0 && transitionAllowed())
  {
    state_ = &Initialization::Instance();
  }
  else if ((str.compare("nominal") == 0) && transitionAllowed())
  {
    state_ = &Nominal::Instance();
  }
  else if ((str.compare("failure") == 0) && transitionAllowed())
  {
    state_ = &Failure::Instance();
  }
  else if ((str.compare("preflight") == 0) && transitionAllowed())
  {
    state_ = &Preflight::Instance();
  }
  else if ((str.compare("start_mission") == 0) && transitionAllowed())
  {
    state_ = &StartMission::Instance();
  }
  else if ((str.compare("perform_mission") == 0) && transitionAllowed())
  {
    state_ = &PerformMission::Instance();
  }
  else if ((str.compare("mission_iterator") == 0) && transitionAllowed())
  {
    state_ = &MissionIterator::Instance();
  }
  else if ((str.compare("end_mission") == 0) && transitionAllowed())
  {
    state_ = &EndMission::Instance();
  }
  else if ((str.compare("land") == 0) && transitionAllowed())
  {
    state_ = &Land::Instance();
  }
  else if ((str.compare("hold") == 0) && transitionAllowed())
  {
    state_ = &Hold::Instance();
  }
  else if (str.compare("termination") == 0 && transitionAllowed())
  {
    state_ = &Termination::Instance();
  }
  else
  {
    logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                  formatMsg("Not allowed to transition to '" + str + "' state", 2));
    state_ = &Failure::Instance();
  }

  // Execute predefined behavior on Entering new state
  state_->onEntry(*this);
}

void Autonomy::watchdogTimerOverflowHandler()
{
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                formatMsg("Timeout overflow -- no heartbeat from system watchdog", 2));
  sub_watchdog_heartbeat_.shutdown();
  sub_watchdog_status_.shutdown();
  stateTransition("land");
}

void Autonomy::flightTimerOverflowHandler()
{
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                formatMsg("Timeout overflow -- maximum flight time achieved -- the platform will land.", 2));
  stateTransition("land");
}

void Autonomy::failureTimerOverflowHandler()
{
  logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                formatMsg("Timeout overflow -- maximum waiting time for a sensor fix achieved -- the platform "
                          "will land",
                          2));
  stateTransition("land");
}

bool Autonomy::missionFileSanityCheck()
{
  for (const auto& filename : missions_.at(mission_id_).getFilepaths())
  {
    waypoints_parser_->setFilename(filename);
    if (!waypoints_parser_->fileSanityCheck())
    {
      logger_.logUI(state_->getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE),
                    formatMsg("Failed file checks on: " + filename, 2));
      return false;
    }
  }
  return true;
}
}  // namespace autonomy
