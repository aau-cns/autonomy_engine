// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
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

#include <limits>

#include "autonomy_core/autonomy.h"
#include "state_machine/states/failure.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/hover.h"
#include "state_machine/states/initialization.h"
#include "state_machine/states/land.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/takeoff.h"
#include "state_machine/states/flight.h"
#include "state_machine/states/undefined.h"
#include "utils/colors.h"

namespace autonomy {

  Autonomy::Autonomy(ros::NodeHandle &nh) :
    nh_(nh) {

    // Parse parameters and options
    if(!parseParams()) {
        handleFailure();
    }

    // Print option
    opts_->printAutonomyOptions();

    // Advertise watchdog service
    watchdog_start_service_client_ = nh_.serviceClient<watchdog_msgs::Start>(opts_->watchdog_start_service_name);

    // Advertise takeoff service
    takeoff_service_client_ = nh_.serviceClient<std_srvs::Trigger>(opts_->takeoff_service_name);

    // Advertise estimator supervisor service
    estimator_supervisor_service_client_ = nh_.serviceClient<std_srvs::Trigger>(opts_->estimator_supervisor_service_name);

    // Advertise data recording service
    data_recording_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->data_recrding_service_name);

    // Advertise data recording service
    estimator_init_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->estimator_init_service_name);

    // Advertise watchdog action topic
    pub_watchdog_action_ = nh.advertise<watchdog_msgs::ActionStamped>(opts_->watchdog_action_topic, 10);

    // Advertise mission sequencer request topic
    pub_mission_sequencer_request_ = nh.advertise<amaze_mission_sequencer::request>(opts_->mission_sequencer_request_topic, 10);

    // Subscribe to mission sequencer responce
    sub_mission_sequencer_responce_ = nh_.subscribe(opts_->mission_sequencer_responce_topic, 1, &Autonomy::missionSequencerResponceCallback, this);

    // Instanciate watchdog timer and connect signal
    watchdog_timer_ = std::make_unique<Timer>(opts_->watchdog_timeout);
    watchdog_timer_->sh_.connect(boost::bind(&Autonomy::watchdogTimerOverflowHandler, this));

    // Instanciate flight timer and connect signal
    flight_timer_ = std::make_unique<Timer>(opts_->flight_timeout);
    flight_timer_->sh_.connect(boost::bind(&Autonomy::flightTimerOverflowHandler, this));

    // Setting state to UNDEFINED
    state_ = &Undefined::Instance();

    // Instanciate waypoints_parser_ with categories {"x", "y", "z", "yaw", "holdtime"}
    waypoints_parser_ = std::make_unique<WaypointsParser>();

  }

  bool Autonomy::parseParams() {

    // Define auxilliary variables
    std::string estimator_supervisor_service_name;
    std::string watchdog_start_service_name;
    std::string watchdog_heartbeat_topic;
    std::string watchdog_status_topic;
    std::string watchdog_action_topic;
    std::string mission_sequencer_request_topic;
    std::string mission_sequencer_responce_topic;
    std::string data_recrding_service_name;
    std::string takeoff_service_name;
    std::string landing_detection_topic;
    std::string estimator_init_service_name;
    std::vector<Mission> missions;
    XmlRpc::XmlRpcValue XRV_missions;
    XmlRpc::XmlRpcValue XRV_filepaths;
    XmlRpc::XmlRpcValue XRV_entities_states;
    float watchdog_rate = 0.0;
    float watchdog_heartbeat_timeout_multiplier = 0.0;
    int watchdog_startup_time_s = 0;
    int watchdog_timeout_ms = 0;
    int flight_timeout_ms = 0;
    int fix_timeout_ms = 0;
    bool activate_user_interface;
    bool activate_watchdog;
    bool activate_data_recording;
    bool estimator_init_service;
    bool perform_takeoff_check;
    bool perform_estimator_check;
    bool activate_landing_detection;
    int mission_id_no_ui = -1;

    // Get Parmaters from ros param server
    if (!nh_.getParam("activate_user_interface", activate_user_interface)) {
      std::cout << BOLD(RED(" >>> [activate_user_interface] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("mission_id_no_ui", mission_id_no_ui) && !activate_user_interface) {
      std::cout << BOLD(RED(" >>> [mission_id_no_ui] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("activate_watchdog", activate_watchdog)) {
      std::cout << BOLD(RED(" >>> [activate_watchdog] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("activate_data_recording", activate_data_recording)) {
      std::cout << BOLD(RED(" >>> [activate_data_recording] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("estimator_init_service", estimator_init_service)) {
      std::cout << BOLD(RED(" >>> [estimator_init_service] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("perform_takeoff_check", perform_takeoff_check)) {
      std::cout << BOLD(RED(" >>> [perform_takeoff_check] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("perform_estimator_check", perform_estimator_check)) {
      std::cout << BOLD(RED(" >>> [perform_estimator_check] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("perform_estimator_check", perform_estimator_check)) {
      std::cout << BOLD(RED(" >>> [perform_estimator_check] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("activate_landing_detection", activate_landing_detection)) {
      std::cout << BOLD(RED(" >>> [activate_landing_detection] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("watchdog_start_service_name", watchdog_start_service_name)) {
      std::cout << BOLD(RED(" >>> [watchdog_start_service_name] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("watchdog_heartbeat_topic", watchdog_heartbeat_topic)) {
      std::cout << BOLD(RED(" >>> [watchdog_heartbeat_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("watchdog_status_topic", watchdog_status_topic)) {
      std::cout << BOLD(RED(" >>> [watchdog_status_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("watchdog_action_topic", watchdog_action_topic)) {
      std::cout << BOLD(RED(" >>> [watchdog_action_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("mission_sequencer_request_topic", mission_sequencer_request_topic)) {
      std::cout << BOLD(RED(" >>> [mission_sequencer_request_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("mission_sequencer_responce_topic", mission_sequencer_responce_topic)) {
      std::cout << BOLD(RED(" >>> [mission_sequencer_responce_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("data_recrding_service_name", data_recrding_service_name)) {
      std::cout << BOLD(RED(" >>> [data_recrding_service_name] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("takeoff_service_name", takeoff_service_name)) {
      std::cout << BOLD(RED(" >>> [takeoff_service_name] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("estimator_supervisor_service_name", estimator_supervisor_service_name)) {
      std::cout << BOLD(RED(" >>> [estimator_supervisor_service_name] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("landing_detection_topic", landing_detection_topic)) {
      std::cout << BOLD(RED(" >>> [landing_detection_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("estimator_init_service_name", estimator_init_service_name)) {
      std::cout << BOLD(RED(" >>> [estimator_init_service_name] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("watchdog_rate_Hz", watchdog_rate)) {
       std::cout << BOLD(RED(" >>> [watchdog_rate_Hz] parameter not defined.\n")) << std::endl;
       return false;
    }
    if (!nh_.getParam("watchdog_heartbeat_timeout_multiplier", watchdog_heartbeat_timeout_multiplier)) {
      std::cout << BOLD(RED(" >>> [watchdog_heartbeat_timeot_multiplier] paramter not defined. This set the heartbeat timer timeout to be a scaled version of the period of the watchdog rate. Please set it higher than 1.0.\n")) << std::endl;
      return false;
    } else {
      if (watchdog_heartbeat_timeout_multiplier >= 1) {
        // Set watchdog timer timeout to be n/watchdog_rate ms (n >= 1)
        watchdog_timeout_ms = static_cast<int>(std::ceil((1000*watchdog_heartbeat_timeout_multiplier)/watchdog_rate));
      } else {
        std::cout << BOLD(RED(" >>> [watchdog_heartbeat_timeot_multiplier] paramter smaller than 1.0. Please set it higher than 1.0.\n")) << std::endl;
        return false;
      }
    }
    if (!nh_.getParam("watchdog_startup_time_s", watchdog_startup_time_s)) {
      std::cout << BOLD(RED(" >>> [watchdog_startup_time_s] paramter not defined. Please set it higher than 10.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("missions", XRV_missions)) {
      std::cout << BOLD(RED(" >>> [missions] paramter not defined. Please specify/load correctly a mission config file.\n")) << std::endl;
      return false;
    } else {
      // Check type
      if (XRV_missions.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        std::cout << BOLD(RED(" >>> [missions] paramter not correctly defined. Please check/correct the specified mission config file.\n")) << std::endl;
        return false;
      }
    }
    if (!nh_.getParam("maximum_flight_time_min", flight_timeout_ms)) {
       std::cout << BOLD(RED(" >>> [maximum_flight_time] parameter not defined.\n")) << std::endl;
       return false;
    } else {
      // Convert given time from minutes to milliseconds
      flight_timeout_ms *= 60000;
    }
    if (!nh_.getParam("fix_timeout_ms", fix_timeout_ms)) {
      std::cout << BOLD(RED(" >>> [fix_timeout_ms] parameter not defined.\n")) << std::endl;
      return false;
    }

    // Check whether missions are defined and parse them
    if (XRV_missions.size() == 0) {
      std::cout << BOLD(RED(" >>> No missions defined on the specified mission config file.\n")) << std::endl;
      return false;
    } else {

      // Loop through missions
      for (int i = 1; i <= XRV_missions.size(); ++i) {

        // Declare mission specific variables
        std::string description;
        std::vector<std::string> filepaths;
        Entity entity;
        AutonomyState state;
        std::map<Entity, AutonomyState> entity_state_map;

        // Get mission description
        if (!nh_.getParam("missions/mission_" + std::to_string(i) + "/description", description)) {
          std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [description] missing.\n")) << std::endl;
          return false;
        }

        // Get filepaths
        if (!nh_.getParam("/autonomy/missions/mission_" + std::to_string(i) + "/filepaths", XRV_filepaths)) {
            std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [filepaths] missing.\n")) << std::endl;
            return false;
        }

        // Check type to be array
        if (XRV_filepaths.getType() ==  XmlRpc::XmlRpcValue::TypeArray) {

          // Loop through the filepaths
          for (int j = 1; j <= XRV_filepaths.size(); ++j) {

            // Check type to be string
            if (XRV_filepaths[j].getType() ==  XmlRpc::XmlRpcValue::TypeString) {

              // assign filepath
              filepaths.emplace_back(std::string(XRV_filepaths[j]));
            } else {
              std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [filepath] wrongly defined in [filepaths] list.\n")) << std::endl;
              return false;
            }
          }
        } else {
          std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [filepaths] wrongly defined, it must be a list.\n")) << std::endl;
          return false;
        }

        // Get entities and next states
        if (!nh_.getParam("missions/mission_" + std::to_string(i) + "/entities_actions", XRV_entities_states)) {
          std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [entities_actions] list missing.\n")) << std::endl;
          return false;
        }

        // Check type to be array
        if (XRV_entities_states.getType() ==  XmlRpc::XmlRpcValue::TypeArray) {

          // Loop through entities and actions
          for (int k = 0; k < XRV_entities_states.size(); ++k) {

            // Check type to be array
            if (XRV_entities_states[k].getType() ==  XmlRpc::XmlRpcValue::TypeArray) {

              // Check type to be string and get entity
              if (XRV_entities_states[k][0].getType() == XmlRpc::XmlRpcValue::TypeString) {
                if (!getEntityFromString(std::string(XRV_entities_states[k][0]), entity)) {
                  std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [entity] wrongly defined in [entities_actions] list.\n")) << std::endl;
                  return false;
                }
              }

              // Check type to be string and get action
              if (XRV_entities_states[k][1].getType() == XmlRpc::XmlRpcValue::TypeString) {
                if (!getAutonomyStateFromString(std::string(XRV_entities_states[k][1]), state)) {
                  std::cout << std::endl << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [action] wrongly defined in [entities_actions] list.\n")) << std::endl;
                  return false;
                }
              }

              // Build the entity_state_map
              entity_state_map.try_emplace(entity, state);

            } else {
              std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [entitiy_action] wrongly defined in [entities_actions] list.\n")) << std::endl;
              return false;
            }
          }
        } else {
          std::cout << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [entities_actions] wrongly defined.\n")) << std::endl;
          return false;
        }

        // Build the mission
        missions_.try_emplace(i, Mission(i, description, filepaths, entity_state_map));
      }
    }

    // Make options
    opts_ = std::make_unique<autonomyOptions>(autonomyOptions({watchdog_heartbeat_topic, watchdog_status_topic, watchdog_action_topic, mission_sequencer_request_topic, mission_sequencer_responce_topic, landing_detection_topic, watchdog_start_service_name, estimator_supervisor_service_name, data_recrding_service_name, takeoff_service_name, estimator_init_service_name, watchdog_timeout_ms, flight_timeout_ms, fix_timeout_ms, watchdog_startup_time_s, activate_user_interface, activate_watchdog, activate_data_recording, estimator_init_service, perform_takeoff_check, perform_estimator_check, activate_landing_detection, mission_id_no_ui}));

    // Success
    return true;
  }

  bool Autonomy::getSensorStatusFromMsg(const watchdog_msgs::Status& msg, SensorStatus& status) {

    // Get debug infos
    status.debug_name = msg.name;
    status.debug_info = msg.info;

    // Get entity
    if (!getEntityFromString(msg.entity, status.entity)) {
      return false;
    }

    // Get type and action (based on type and mission specification)
    // the action will be different from NOTHING only if a hold status is required
    switch (msg.type) {
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
      if (missions_.at(mission_id_).getNextState(status.entity) == AutonomyState::HOLD) {
        status.action = Action::FIX_NODE;
      } else {
        status.action = Action::NOTHING;
      }
      break;
    case watchdog_msgs::Status::DRIVER:
      status.type = Type::DRIVER;
      if (missions_.at(mission_id_).getNextState(status.entity) == AutonomyState::HOLD) {
        status.action = Action::FIX_DRIVER;
      } else {
        status.action = Action::NOTHING;
      }
      break;
    }

    // Get event. Increase pending failure in case of failure,
    // reduce pending failures in case of fix
    if (msg.status == watchdog_msgs::Status::ERROR) {
      status.event = Event::ENTITY_FAILURE;
      // Increase pending failures and start a timer
      // if the error needs to be fixed in the specified mission
      if (missions_.at(mission_id_).getNextState(status.entity) == AutonomyState::HOLD) {
        pending_failures_.emplace_back(std::make_pair(status, std::make_unique<Timer>(opts_->fix_timeout)));
        pending_failures_.back().second->sh_.connect(boost::bind(&Autonomy::failureTimerOverflowHandler, this));
      }
    } else if (msg.status == watchdog_msgs::Status::NOMINAL) {
      // search and remove fixed failure from pending failures, the fix must be the consequence of an action of fixing
      // which can only be triggered if the hold status is requested. If for some reason we got a NOMINAL status without
      // requiring a specific action we set the event to OTHER
      const auto &it = std::remove_if(pending_failures_.begin(), pending_failures_.end(), [&status](const std::pair<SensorStatus, std::unique_ptr<Timer>>& failure){return failure.first == status;});
      if (it != pending_failures_.end()) {
        it->second->stopTimer();
        pending_failures_.erase(it, pending_failures_.end());
        status.event = Event::ENTITY_FIX;
      } else {
        status.event = Event::ENTITY_OTHER;
      }
    } else {
      status.event = Event::ENTITY_OTHER;
    }

    return true;
  }

  void Autonomy::getRequestfromMsg(const amaze_mission_sequencer::request& msg, std::string& request_str) {

    // Get request
    switch(msg.request) {
    case amaze_mission_sequencer::request::ABORT:
      request_str = "abort";
      break;
    case amaze_mission_sequencer::request::ARM:
      request_str = "arm";
      break;
    case amaze_mission_sequencer::request::TAKEOFF:
      request_str = "takeoff";
      break;
    case amaze_mission_sequencer::request::HOLD:
      request_str = "hold";
      break;
    case amaze_mission_sequencer::request::RESUME:
      request_str = "resume";
      break;
    case amaze_mission_sequencer::request::LAND:
      request_str = "land";
      break;
    case amaze_mission_sequencer::request::HOVER:
      request_str = "hover";
      break;
    case amaze_mission_sequencer::request::ABORT:
      request_str = "abort";
      break;
    case amaze_mission_sequencer::request::DISARM:
      request_str = "disarm";
      break;
    }
  }

  void Autonomy::watchdogHeartBeatCallback(const watchdog_msgs::StatusStampedConstPtr&) {

    // Restart timeout timer
    watchdog_timer_->resetTimer();
  }

  void Autonomy::watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStampedConstPtr& msg) {

    // Loop through all the chenges with respect to last iteration
    for (const auto &it : msg->data.changes) {

      // Define status
      SensorStatus status;

      // Assign time information
      status.timestamp = msg->header.stamp.toSec();

      // Parse information of status changes
      if (getSensorStatusFromMsg(it, status)) {

        // Check event
        if (status.event != Event::ENTITY_OTHER) { 

          if (status.event == Event::ENTITY_FAILURE) {

            std::string entity;
            std::string type;

            getStringFromEntity(status.entity, entity);
            getStringFromType(status.type, type);

            std::cout << BOLD(YELLOW("-------------------------------------------------\n"));
            std::cout << BOLD(YELLOW(" >>> Sensor failure reported by the watchdog <<< \n"));
            std::cout << BOLD(YELLOW(" >>> Entity:      " + entity + "\n"));
            std::cout << BOLD(YELLOW(" >>> Type:        " + type + "\n"));
            std::cout << BOLD(YELLOW("-------------------------------------------------\n")) << std::endl;

            // Failure -- search an action (next state) on specific mission (mission id)
            // Next states can be: FLIGHT, HOLD, LAND, FAILURE
            next_state_ = missions_.at(mission_id_).getNextState(status.entity);

            // If in_flight_, always call state transition otherwise, trigger a failure for any error that would require either FAILURE, HOLD or LAND
            if (in_flight_ || next_state_ == AutonomyState::FAILURE) {
              stateTransition();
            } else {
              if (next_state_ != AutonomyState::FLIGHT) {
                next_state_ = AutonomyState::FAILURE;
                stateTransition();
              }
            }

            // Always perform an action to react to the failure (the action can be NOTHING)
            watchdogActionRequest(status, it);

          }

          if (status.event == Event::ENTITY_FIX) {

            std::string entity;
            std::string type;

            getStringFromEntity(status.entity, entity);
            getStringFromType(status.type, type);

            std::cout << BOLD(GREEN("-------------------------------------------------\n"));
            std::cout << BOLD(GREEN(" >>> Sensor fix reported by the watchdog <<< \n"));
            std::cout << BOLD(GREEN(" >>> Entity:      " + entity + "\n"));
            std::cout << BOLD(GREEN(" >>> Type:        " + type + "\n"));
            std::cout << BOLD(GREEN("-------------------------------------------------\n")) << std::endl;

            // Fix -- At this stage the pending failure relative to the fix has already been removed.
            // Check if pending failure size == 0 then stop holding and resulme the mission otherwise keep holding
            if (pending_failures_.size() == 0) {

              // Check we are flying (we should not be here if we are not flying)
              if (in_flight_) {

                // Call state transition to FLIGHT
                next_state_ = AutonomyState::FLIGHT;
                stateTransition();

                // Resume mission
                missionSequencerRequest(amaze_mission_sequencer::request::RESUME);

              } else {
                std::cout << BOLD(RED(" >>> Received Fix while not flying.\n")) << std::endl;
              }
            }
          }

        } else if (it.status == watchdog_msgs::Status::DEFECT) {
            // TODO: Log the Defect
        }
      } else {
        std::cout << BOLD(RED(" >>> Wrong message received from watchdog.\n")) << std::endl;
      }
    }
  }

  void Autonomy::watchdogTimerOverflowHandler() {

    // print message of watchdog timer overflow
    std::cout << BOLD(YELLOW(" >>> Timeout overflow -- no heartbeat from system watchdog.\n"));
    std::cout << BOLD(YELLOW(" >>> Mission continue without the support of the watchdog.\n")) << std::endl;
  }

  void Autonomy::flightTimerOverflowHandler() {

    // print message of flight timer overflow
    std::cout << BOLD(YELLOW(" >>> Timeout overflow -- maximum flight time achieved -- the platform will land.\n")) << std::endl;

    // Call state transition to LAND if in_flight_ and set land_expected_ flag
    if (in_flight_) {
      land_expected_ = true;
      next_state_ = AutonomyState::LAND;
      stateTransition();
    } else {
      std::cout << BOLD(YELLOW(" >>> Cannot request a land if the platform is not flying.\n")) << std::endl;
    }

  }

  void Autonomy::failureTimerOverflowHandler() {

    // print message of failure timer overflow
    std::cout << BOLD(YELLOW(" >>> Timeout overflow -- maximum waiting time for a sensor fix achieved -- the platform will land.\n")) << std::endl;

    // Call state transition to LAND if in_flight_ and set land_expected_ flag
    if (in_flight_) {
      land_expected_ = true;
      next_state_ = AutonomyState::LAND;
      stateTransition();
    } else {
      std::cout << BOLD(YELLOW(" >>> Cannot request a land if the platform is not flying.\n")) << std::endl;
    }

  }

  void Autonomy::landingDetectionCallback(const std_msgs::BoolConstPtr& msg) {

    if (msg) {
      std::cout << BOLD(GREEN(" >>> Flat land detected.\n")) << std::endl;
    } else {
      std::cout << BOLD(YELLOW(" >>> Non flat land detected.\n")) << std::endl;
    }

    // Stop data recording after landing in case mission has been succesfully completed
    if (land_expected_) {

      // Disarm request
      if (armed_) {
        missionSequencerRequest(amaze_mission_sequencer::request::DISARM);
      } else {
        std::cout << BOLD(YELLOW(" >>> the platform is already disarmed, skipped DISARM request\n")) << std::endl;
      }

      // Wait until DISARM request got accepted
      while (armed_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      // Check if we succesfully completed the mission and if we are performing ultiple touchdown
      if (last_waypoint_reached_) {

        if (multiple_touchdowns_) {

          // Print info
          std::cout << BOLD(GREEN(" >>> Iteration of mission ID: " + std::to_string(mission_id_) + " succesfully completed.\n"));
          std::cout << BOLD(GREEN(" >>> Continuing with next iteration ...")) << std::endl;

          // Increment the filepaths counter
          ++filepaths_cnt_;

          // Setting state to TAKEOFF
          // This will perform first the preflight checks if they are enabled,
          // it will send the takeoff command to the mission sequencer and then,
          // will wait for success responce from the mission sequencer to send waypoints
          next_state_ = AutonomyState::TAKEOFF;
          stateTransition();

          // Setting state to FLIGHT
          next_state_ = AutonomyState::FLIGHT;
          stateTransition();

        } else {

          // Print info
          std::cout << BOLD(GREEN(" >>> Mission ID: " + std::to_string(mission_id_) + " succesfully completed.\n")) << std::endl;

          // Stop data recording if data is getting recorded
          DataRecording(false);
        }
      }
    } else {

      // Print info
      std::cout << BOLD(RED("-------------------------------------------------\n"));
      std::cout << BOLD(RED(" >>> Unexpected land detected <<<\n"));
      std::cout << BOLD(RED("-------------------------------------------------\n")) << std::endl;

      // Call state transition to LAND if in_flight_ and set land_expected_ flag
      if (in_flight_) {
        land_expected_ = true;
        next_state_ = AutonomyState::LAND;
        stateTransition();
      } else {
        std::cout << BOLD(YELLOW(" >>> Cannot request a land if the platform is not flying.\n")) << std::endl;
      }
    }
  }

  void Autonomy::missionSequencerResponceCallback(const amaze_mission_sequencer::responseConstPtr& msg) {

    // Define request
    std::string req;

    // Get request
    getRequestfromMsg(msg->request, req);

    // Check if mission sequencer request has been accepted
    if (msg->response && msg->request.request != amaze_mission_sequencer::request::UNDEF) {
      std::cout << BOLD(GREEN(" >>> Request [" + req + "] accepted from Mission Sequencer.\n")) << std::endl;

      // ARM request
      if (msg->request.request == amaze_mission_sequencer::request::ARM) {

        // Set armed_ flag
        armed_ = true;

        // Reset flight_timer
        flight_timer_->resetTimer();

        // Subscribe to landing detection if active
        if (opts_->activate_landing_detection) {
          sub_landing_detection_ = nh_.subscribe(opts_->landing_detection_topic, 1, &Autonomy::landingDetectionCallback, this);
        }
      }

      // TAKEOFF request
      if (msg->request.request == amaze_mission_sequencer::request::TAKEOFF) {

        // Set in_flight_ and last_waypoint_reached_ flag
        in_flight_ = true;
        last_waypoint_reached_ = false;
      }

      // HOLD request
      if (msg->request.request == amaze_mission_sequencer::request::HOLD) {

        // Set holding_ flag
        holding_ = true;
      }

      // RESUME request
      if (msg->request.request == amaze_mission_sequencer::request::RESUME) {

        // Set holding_ flag
        holding_ = false;
      }

      // LAND request
      if (msg->request.request == amaze_mission_sequencer::request::LAND) {

        // Set in_flight_ flag
        in_flight_ = false;
      }

      // HOVER request
      if (msg->request.request == amaze_mission_sequencer::request::HOVER) {
      }

      // DISARM request
      if (msg->request.request == amaze_mission_sequencer::request::DISARM) {

        // Set armed_ flag
        armed_ = false;

        // stop flight timer
        flight_timer_->stopTimer();

      }
    }

    if (!msg->response && !msg->completed) {
      std::cout << BOLD(RED(" >>> Request [" + req + "] for mission ID: " + std::to_string(msg->request.id) + "  rejected from Mission Sequencer.\n")) << std::endl;
      handleFailure();
    }

    // Check if the mission is ended (last waypoint reached)
    if (msg->request.request == amaze_mission_sequencer::request::UNDEF) {
      if (msg->completed && !msg->response) {
        std::cout << BOLD(GREEN(" >>> Mission ID: " + std::to_string(msg->request.id) + " succesfully reached last waypoint.\n")) << std::endl;

        // set last_waypoint_reached_ flag
        last_waypoint_reached_ = true;

        // Call state transition to LAND if in_flight_ and set land_expected_ flag
        if (in_flight_) {
          land_expected_ = true;
          next_state_ = AutonomyState::LAND;
          stateTransition();
        } else {
          std::cout << BOLD(YELLOW(" >>> Cannot request a land if the platform is not flying.\n")) << std::endl;
        }

      }
    }
  }

  void Autonomy::missionSequencerRequest(const int& request) {

    // Define request message to mission sequencer
    amaze_mission_sequencer::request req;

    // Set mission id and request
    req.header.stamp = ros::Time::now();
    req.id = uint8_t(mission_id_);
    req.request = uint8_t(request);

    // publish mission start request
    pub_mission_sequencer_request_.publish(req);

  }

  void Autonomy::watchdogActionRequest(SensorStatus& status, const watchdog_msgs::Status& status_msg) {

    // Define action and action message
    watchdog_msgs::ActionStamped action_msg;

    // Fill action message
    action_msg.header.stamp = ros::Time::now();
    action_msg.action.entity = status_msg;

    switch (status.action) {
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

    // Publish action message
    std::cout << BOLD(YELLOW(" >>> Action communicated.\n")) << std::endl;
    pub_watchdog_action_.publish(action_msg);

  }

  void Autonomy::startWatchdog() {

    // Define service request
    watchdog_msgs::Start watchdog_start;
    watchdog_start.request.header.stamp = ros::Time::now();
    watchdog_start.request.startup_time = opts_->watchdog_startup_time;

    std::cout << BOLD(GREEN(" >>> Starting Watchdog... Please wait\n")) << std::endl;

    // Call service request
    if (watchdog_start_service_client_.call(watchdog_start)) {

      // Check responce
      if(watchdog_start.response.successful) {

        std::cout << BOLD(GREEN(" >>> Watchdog is running\n")) << std::endl;

        // Subscriber to watchdog (system status) heartbeat
        sub_watchdog_heartbeat_ = nh_.subscribe("/watchdog/status", 1, &Autonomy::watchdogHeartBeatCallback, this);

        // Subscribe to watchdog status changes
        sub_watchdog_status_ = nh_.subscribe("/watchdog/log", 1, &Autonomy::watchdogStatusCallback, this);

        // State transition to NOMINAL
        next_state_ = AutonomyState::NOMINAL;
        stateTransition();
      }
    } else {
      watchdog_start.response.successful = false;
    }

    if (!watchdog_start.response.successful) {

      std::cout << BOLD(RED("----------- FAILED TO START WATCHDOG ------------\n"));
      std::cout << BOLD(RED(" Please perform a system hard restart \n"));
      std::cout << BOLD(RED("-------------------------------------------------\n")) << std::endl;

      // Define status to get debug info
      SensorStatus status;

      // Get timestamp
      status.timestamp = watchdog_start.response.header.stamp.toSec();

      // print debug info
      if (getSensorStatusFromMsg(watchdog_start.response.status, status)) {

        std::string entity;
        std::string type;

        getStringFromEntity(status.entity, entity);
        getStringFromType(status.type, type);

        std::cout << BOLD(RED("--------------------- DEBUG ---------------------\n"));
        std::cout << BOLD(RED(" Entity:      " + entity + "\n"));
        std::cout << BOLD(RED(" Type:        " + type + "\n"));
        std::cout << BOLD(RED(" Debug name : " + status.debug_name + "\n"));
        std::cout << BOLD(RED(" Debug info : " + status.debug_info + "\n"));
        std::cout << BOLD(RED("-------------------------------------------------\n")) << std::endl;
      }

      // State transition to FAILURE
      next_state_ = AutonomyState::FAILURE;
      stateTransition();

    }
  }

  void Autonomy::missionSelection() {

    // If using the user interface
    if (opts_->activate_user_interface) {

      // Print missions
      std::cout << BOLD(GREEN(" >>> Please select one of the following mission by inserting the mission ID\n\n"));
      for (auto &it : missions_) {
        std::cout << BOLD(GREEN("      - ID: ")) << it.first << BOLD(GREEN(" DESCRIPTION: ")) << it.second.getDescription() << "\n";
      }

      // Get ID of mission being executed
      std::cout << "\n" << BOLD(GREEN(" >>> Mission ID: ")) << std::flush;
      std::cin >> mission_id_;

      // Check validity of mission id
      if (mission_id_ < 1 || mission_id_ > static_cast<int>(missions_.size())) {
        std::cout << "\n" << BOLD(RED(" >>> Wrong mission ID chosen\n")) << std::endl;

        // Call recursively mission selection
        missionSelection();
      }

    } else {

      // Load mission from parameter
      mission_id_ = opts_->mission_id_no_ui;

    }

    std::cout << "\n" << BOLD(GREEN("      - Loaded mission with ID: ")) << mission_id_ << "\n" << std::endl;

    if (missions_.at(mission_id_).getTouchdowns() > 0) {
      std::cout << "\n" << BOLD(YELLOW(" >>> Loaded mission with Multiple touchdowns: ")) << missions_.at(mission_id_).getTouchdowns() << " touchdowns\n" << std::endl;
      multiple_touchdowns_ = true;
    }
  }

  void Autonomy::preFlightChecks() {

    std::cout << BOLD(GREEN(" >>> Starting Pre-Flight Checks...\n")) << std::endl;

    // if (!(check1() & check2() & ...)) {
    if (!(takeoffChecks())) {
      next_state_ = AutonomyState::FAILURE;
      stateTransition();
    }

    if (opts_->perform_estimator_check && !estimatorCheck()) {
      next_state_ = AutonomyState::FAILURE;
      stateTransition();
    }

    // Trigger State estimation initialization if a cascade of estimators are used
    if (opts_->estimator_init_service && !initializeStateEstimation()) {
      next_state_ = AutonomyState::FAILURE;
      stateTransition();
    }

    std::cout << BOLD(GREEN(" >>> Pre-Flight checks successed\n")) << std::endl;

  }

  bool Autonomy::takeoffChecks() {

    // Define takeoff request
    std_srvs::Trigger takeoff;

    // service call to check if we are ready to takeoff
    if (takeoff_service_client_.call(takeoff)) {

      // Check responce
      if(takeoff.response.success) {
        std::cout << BOLD(GREEN(" >>> Vehicle flat on the ground\n"));
        std::cout << BOLD(GREEN(" >>> Takeoff checks successed\n")) << std::endl;
      }
    } else {
      takeoff.response.success = false;
    }

    if (!takeoff.response.success) {
      std::cout << BOLD(RED(" >>> Takeoff checks failed\n")) << std::endl;
      return false;
    }

    return true;
  }

  bool Autonomy::estimatorCheck() {

    std::cout << BOLD(YELLOW(" >>> Please, Initialize estimator now\n"));
    std::cout << BOLD(YELLOW(" >>> When done, press [ENTER] to start the experiment"));
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << std::endl;

    // Define takeoff request
    std_srvs::Trigger superivsion;

    // service call to check if we are ready to takeoff
    if (estimator_supervisor_service_client_.call(superivsion)) {

      // Check responce
      if(superivsion.response.success) {
        std::cout << BOLD(GREEN(" >>> State estimator Correctly initilized\n")) << std::endl;
      } else {
        superivsion.response.success = false;
      }
    } else {
      superivsion.response.success = false;
    }

    if (!superivsion.response.success) {
      std::cout << BOLD(RED(" >>> State estimator initialization failed\n")) << std::endl;
      return false;
    }

    return true;
  }

  bool Autonomy::initializeStateEstimation() {

    // Define data recording
    std_srvs::SetBool init;

    // Set data recording start request
    init.request.data = true;

    std::cout << BOLD(GREEN(" >>> Initializing state estimator... Please wait\n"));

    // service call to check if we are ready to takeoff
    if (estimator_init_service_client_.call(init)) {

      // Check responce
      if (init.response.success) {
        std::cout << BOLD(GREEN(" >>> State estimator initialized succesfully\n")) << std::endl;
      } else {
        init.response.success = false;
      }
    } else {
      init.response.success = false;
    }

    if (!init.response.success) {
      std::cout << BOLD(RED(" >>> State estimator initialization failed\n")) << std::endl;
      return false;
    }

    return true;
  }

  void Autonomy::DataRecording(const bool& start_stop) {

    // Define data recording
    std_srvs::SetBool data_rec;

    // Set data recording start/stop request
    data_rec.request.data = start_stop;

    // service call (before stopping check if we are recording)
    if (!start_stop && is_recording_) {
      if (data_recording_service_client_.call(data_rec)) {

        // Check responce
        if (data_rec.response.success) {
          if (data_rec.request.data) {
            std::cout << BOLD(GREEN(" >>> Data recorded started succesfully\n")) << std::endl;
            is_recording_ = true;
          } else {
            std::cout << BOLD(GREEN(" >>> Data recorded stopped succesfully\n")) << std::endl;
            is_recording_ = false;
          }
        } else {
          std::cout << BOLD(RED(" >>> Data recorded service failure\n")) << std::endl;
        }
      } else {
        if (data_rec.request.data) {
          std::cout << BOLD(RED(" >>> Data recorded start failure\n")) << std::endl;
        } else {
          std::cout << BOLD(RED(" >>> Data recorded stop failure\n")) << std::endl;
        }
      }
    } else {
      std::cout << BOLD(RED(" >>> Data recorded stop failure, cannot stop data recording if the recording is inactive\n")) << std::endl;
    }
  }

  void Autonomy::arm() {

    // Print info
    std::cout << BOLD(GREEN(" >>> Arming...\n")) << std::endl;

    // Request arming, if not already armed
    if (!armed_) {
      missionSequencerRequest(amaze_mission_sequencer::request::ARM);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already armed, skipped ARM request\n")) << std::endl;
    }
  }

  void Autonomy::takeoff() {

    // Print info
    std::cout << BOLD(GREEN(" >>> Taking off...\n")) << std::endl;

    // Takeoff, if not already in flight and if armed
    if (!in_flight_ && armed_) {
      missionSequencerRequest(amaze_mission_sequencer::request::TAKEOFF);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already flying, skipped TAKEOFF request\n")) << std::endl;
    }
  }

  void Autonomy::sendWaypoints() {

    // Set filename to waypoint parser
    waypoints_parser_->setFilename(missions_.at(mission_id_).getFilepaths().at(static_cast<size_t>(filepaths_cnt_)));

    // Parse waypoint file
    waypoints_parser_->readParseCsv();

    // Get the data
    waypoints_ = waypoints_parser_->getData();

    // Print info
    std::cout << BOLD(GREEN(" >>> Communicating waypoints to the mission sequencer...\n")) << std::endl;

    // If we already performed a takeoff
    if (in_flight_) {
      // TODO: Send waypoints
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is not flying, waypoints cannot be sent to mission sequencer\n")) << std::endl;
    }
  }

  void Autonomy::hold() {

    // Print info
    std::cout << BOLD(GREEN(" >>> Holding...\n")) << std::endl;

    // Request holding if not holding
    if(!holding_) {
      missionSequencerRequest(amaze_mission_sequencer::request::HOLD);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already holding, skipped HOLD request\n")) << std::endl;
    }

  }

  void Autonomy::land() {

    // Print info
    std::cout << BOLD(GREEN(" >>> Landing...\n")) << std::endl;

    // Request landing if in_flight_
    if(in_flight_) {
      missionSequencerRequest(amaze_mission_sequencer::request::LAND);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is not flying, skipped LAND request\n")) << std::endl;
    }
  }

  void Autonomy::startAutonomy() {

    // Mission selection
    missionSelection();

    // Setting state to INITIALIZATION and initialize the system (watchdog)
    // This will trigger a state transition to either NOMINAL or FAILURE
    // depending by the result of the initialization
    // If the state is set to NOMINAL after succesfull initialization the data
    // recording will start if enabled and the waypoints for the selected mission will be loaded
    next_state_ = AutonomyState::INITIALIZATION;
    stateTransition();

    // Setting state to TAKEOFF
    // This will perform first the preflight checks if they are enabled,
    // it will send the takeoff command to the mission sequencer and then,
    // will wait for success responce from the mission sequencer to send waypoints
    next_state_ = AutonomyState::TAKEOFF;
    stateTransition();

    // Setting state to FLIGHT
    next_state_ = AutonomyState::FLIGHT;
    stateTransition();

  }

  void Autonomy::stateTransition() {

    // Execute predefined behavior on Exiting old state
    state_->onExit(*this);

    // Execute state transition
    switch(next_state_) {
      case AutonomyState::NOMINAL:
        state_ = &Nominal::Instance();
        break;
      case AutonomyState::HOLD:
        state_ = &Hold::Instance();
        break;
      case AutonomyState::FAILURE:
        state_ = &Failure::Instance();
        break;
      case AutonomyState::LAND:
        state_ = &Land::Instance();
        break;
      case AutonomyState::HOVER:
        state_ = &Hover::Instance();
        break;
      case AutonomyState::INITIALIZATION:
        state_ = &Initialization::Instance();
        break;
      case AutonomyState::UNDEFINED:
        state_ = &Undefined::Instance();
        break;
      case AutonomyState::TAKEOFF:
        state_ = &Takeoff::Instance();
        break;
    case AutonomyState::FLIGHT:
      state_ = &Flight::Instance();
      break;
      }

    // Execute predefined behavior on Entering new state
    state_->onEntry(*this);
  }

  void Autonomy::handleFailure() {

    // Stop data recording if data is getting recorded
    DataRecording(false);

    // Stop any timer
    watchdog_timer_->stopTimer();
    flight_timer_->stopTimer();
    for (const auto& it : pending_failures_) {
      it.second->stopTimer();
    }

    // Clear panding failure
    pending_failures_.clear();

    // Clear waypoints
    waypoints_.clear();

    // Send abort request to mission sequencer
    missionSequencerRequest(amaze_mission_sequencer::request::ABORT);

    // Unsubscribe from all the subscribed topics
    sub_watchdog_heartbeat_.shutdown();
    sub_watchdog_status_.shutdown();
    sub_landing_detection_.shutdown();
    sub_mission_sequencer_responce_.shutdown();

  }

} // namespace autonomy

// TODOS:
// - stop flight timer when mission done
// - Implement the possibility to decide to land or hover when the last waypoint is reached,
//   moreover implement the hover request onentry on hover state

// NOTES:
// Do we need to send the first waypoint at the takeoff (to have height reference)?


































//  bool Autonomy::setStatusMsgFromEntityTypeSubType(const Entity& entity, const Type& type, const subType& subtype, watchdog_msgs::Status& msg) {

//    // Set entity
//    if(!getEntityString(entity, msg.entity)) {
//      return false;
//    }

//    // Set status
//    switch(type) {
//    case Type::FIX:
//      msg.type = watchdog_msgs::Status::NOMINAL;
//      break;
//    case Type::FAILURE:
//      msg.type = watchdog_msgs::Status::ERROR;
//      break;
//    default:
//      msg.type = watchdog_msgs::Status::UNDEF;
//      break;
//    }


//    // Set type
//    switch (subtype) {
//    case subType::GLOBAL:
//      msg.type = watchdog_msgs::Status::GLOBAL;
//      break;
//    case subType::TOPIC:
//      msg.type = watchdog_msgs::Status::TOPIC;
//      break;
//    case subType::NODE:
//      msg.type = watchdog_msgs::Status::NODE;
//      break;
//    case subType::DRIVER:
//      msg.type = watchdog_msgs::Status::DRIVER;
//      break;
//    }

//    return true;
//  }








