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

#include <limits>

#include "autonomy_core/autonomy.h"
#include "state_machine/states/failure.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/initialization.h"
#include "state_machine/states/land.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/undefined.h"
#include "state_machine/states/preflight.h"
#include "state_machine/states/start_mission.h"
#include "state_machine/states/perform_mission.h"
#include "state_machine/states/end_mission.h"
#include "state_machine/states/termination.h"
#include "utils/colors.h"

namespace autonomy {

  Autonomy::Autonomy(ros::NodeHandle &nh) :
    nh_(nh) {

    // Parse parameters and options
    if(!parseParams()) {
        throw FailureException();
    }

    // Print option
    opts_->printAutonomyOptions();

    // Advertise watchdog service and action topic and instanciate timer
    if (opts_->activate_watchdog) {
      watchdog_start_service_client_ = nh_.serviceClient<watchdog_msgs::Start>(opts_->watchdog_start_service_name);
      pub_watchdog_action_ = nh.advertise<watchdog_msgs::ActionStamped>(opts_->watchdog_action_topic, 10);
      watchdog_timer_ = std::make_unique<Timer>(opts_->watchdog_timeout);
      watchdog_timer_->sh_.connect(boost::bind(&Autonomy::watchdogTimerOverflowHandler, this));
    }

    // Advertise estimator init service
    if (opts_->estimator_init_service) {
      estimator_init_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->estimator_init_service_name);
    }

    // Advertise estimator supervisor service
    if (opts_->perform_estimator_check) {
      estimator_supervisor_service_client_ = nh_.serviceClient<std_srvs::Trigger>(opts_->estimator_supervisor_service_name);
    }

    // Advertise takeoff service
    if (opts_->perform_takeoff_check) {
      takeoff_service_client_ = nh_.serviceClient<std_srvs::Trigger>(opts_->takeoff_service_name);
    }

    // Advertise data recording service
    if (opts_->activate_data_recording) {
      data_recording_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->data_recrding_service_name);
    }

    // Advertise mission sequencer request topic
    pub_mission_sequencer_request_ = nh.advertise<mission_sequencer::MissionRequest>(opts_->mission_sequencer_request_topic, 10);

    // Advertise mission sequencer waypoints topic
    pub_mission_sequencer_waypoints_ = nh.advertise<mission_sequencer::MissionWaypointArray>(opts_->mission_sequencer_waypoints_topic, 10);

    // Subscribe to mission sequencer responce
    sub_mission_sequencer_responce_ = nh_.subscribe(opts_->mission_sequencer_responce_topic, 100, &Autonomy::missionSequencerResponceCallback, this);

    // Instanciate flight timer and connect signal
    flight_timer_ = std::make_unique<Timer>(opts_->flight_timeout);
    flight_timer_->sh_.connect(boost::bind(&Autonomy::flightTimerOverflowHandler, this));

    // Instanciate waypoints_parser_ with categories {"x", "y", "z", "yaw", "holdtime"}
    waypoints_parser_ = std::make_unique<WaypointsParser>();

    // Setting state to UNDEFINED
    state_ = &Undefined::Instance();

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
    std::string mission_sequencer_waypoints_topic;
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
    bool hover_after_mission_completion;
    int mission_id_no_ui = -1;

    // Get Parmaters from ros param server
    if (!nh_.getParam("activate_user_interface", activate_user_interface)) {
      std::cout << BOLD(RED(" >>> [activate_user_interface] parameter not defined.\n")) << std::endl;
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
    if (!nh_.getParam("activate_landing_detection", activate_landing_detection)) {
      std::cout << BOLD(RED(" >>> [activate_takeoff_landing_detection] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("hover_after_mission_completion", hover_after_mission_completion)) {
      std::cout << BOLD(RED(" >>> [hover_after_mission_completion] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!activate_user_interface) {
      if (!nh_.getParam("mission_id_no_ui", mission_id_no_ui)) {
        std::cout << BOLD(RED(" >>> [mission_id_no_ui] parameter not defined.\n")) << std::endl;
        return false;
      }
    }
    if (activate_watchdog) {
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
    }
    if (!nh_.getParam("mission_sequencer_request_topic", mission_sequencer_request_topic)) {
      std::cout << BOLD(RED(" >>> [mission_sequencer_request_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("mission_sequencer_responce_topic", mission_sequencer_responce_topic)) {
      std::cout << BOLD(RED(" >>> [mission_sequencer_responce_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (!nh_.getParam("mission_sequencer_waypoints_topic", mission_sequencer_waypoints_topic)) {
      std::cout << BOLD(RED(" >>> [mission_sequencer_waypoints_topic] parameter not defined.\n")) << std::endl;
      return false;
    }
    if (activate_data_recording) {
      if (!nh_.getParam("data_recrding_service_name", data_recrding_service_name)) {
        std::cout << BOLD(RED(" >>> [data_recrding_service_name] parameter not defined.\n")) << std::endl;
        return false;
      }
    }
    if (estimator_init_service) {
      if (!nh_.getParam("estimator_init_service_name", estimator_init_service_name)) {
        std::cout << BOLD(RED(" >>> [estimator_init_service_name] parameter not defined.\n")) << std::endl;
        return false;
      }
    }
    if (perform_takeoff_check) {
      if (!nh_.getParam("takeoff_service_name", takeoff_service_name)) {
        std::cout << BOLD(RED(" >>> [takeoff_service_name] parameter not defined.\n")) << std::endl;
        return false;
      }
    }
    if (perform_estimator_check) {
      if (!nh_.getParam("estimator_supervisor_service_name", estimator_supervisor_service_name)) {
        std::cout << BOLD(RED(" >>> [estimator_supervisor_service_name] parameter not defined.\n")) << std::endl;
        return false;
      }
    }
    if (activate_landing_detection) {
      if (!nh_.getParam("landing_detection_topic", landing_detection_topic)) {
        std::cout << BOLD(RED(" >>> [landing_detection_topic] parameter not defined.\n")) << std::endl;
        return false;
      }
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
        std::string description, state;
        std::vector<std::string> filepaths;
        Entity entity;
        std::map<Entity, std::string> entity_state_map;

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
          for (int j = 0; j < XRV_filepaths.size(); ++j) {

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

                // Check the existance of predefined state for the given string action
                if (!checkStateFromString(std::string(XRV_entities_states[k][1]))) {
                  std::cout << std::endl << BOLD(RED(" >>> mission_" + std::to_string(i) + ": [action] wrongly defined in [entities_actions] list.\n")) << std::endl;
                  return false;
                } else {
                  state = std::string(XRV_entities_states[k][1]);
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
    opts_ = std::make_unique<autonomyOptions>(autonomyOptions({watchdog_heartbeat_topic,
                                                               watchdog_status_topic,
                                                               watchdog_action_topic,
                                                               mission_sequencer_request_topic,
                                                               mission_sequencer_responce_topic,
                                                               landing_detection_topic,
                                                               mission_sequencer_waypoints_topic,
                                                               watchdog_start_service_name,
                                                               estimator_supervisor_service_name,
                                                               data_recrding_service_name,
                                                               takeoff_service_name,
                                                               estimator_init_service_name,
                                                               watchdog_timeout_ms,
                                                               flight_timeout_ms,
                                                               fix_timeout_ms,
                                                               watchdog_startup_time_s,
                                                               activate_user_interface,
                                                               activate_watchdog,
                                                               activate_data_recording,
                                                               estimator_init_service,
                                                               perform_takeoff_check,
                                                               perform_estimator_check,
                                                               activate_landing_detection,
                                                               hover_after_mission_completion,
                                                               mission_id_no_ui}));

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
      if (missions_.at(mission_id_).getNextState(status.entity).compare("hold") == 0) {
        status.action = Action::FIX_NODE;
      } else {
        status.action = Action::NOTHING;
      }
      break;
    case watchdog_msgs::Status::DRIVER:
      status.type = Type::DRIVER;
      if (missions_.at(mission_id_).getNextState(status.entity).compare("hold") == 0) {
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
      if (missions_.at(mission_id_).getNextState(status.entity).compare("hold") == 0) {
        pending_failures_.emplace_back(std::make_pair(status, std::make_unique<Timer>(opts_->fix_timeout)));
        pending_failures_.back().second->sh_.connect(boost::bind(&Autonomy::failureTimerOverflowHandler, this));
        pending_failures_.back().second->resetTimer();
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

  bool Autonomy::getRequestfromMsg(const mission_sequencer::MissionRequest& msg, std::string& request_str) {

    // Get request
    switch(msg.request) {
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

          // Define auxilliary strings
          std::string entity;
          std::string type;

          if (status.event == Event::ENTITY_FAILURE) {

            // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromEntity
            if (!getStringFromEntity(status.entity, entity)) {
              std::cout << BOLD(RED(" >>> No string defined for required entity: " + std::to_string(status.entity) + "\n")) << std::endl;
            }

            // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
            if (!getStringFromType(status.type, type)) {
              std::cout << BOLD(RED(" >>> No string defined for required type: " + std::to_string(status.type) + "\n")) << std::endl;
            }

            std::cout << BOLD(YELLOW("-------------------------------------------------\n"));
            std::cout << BOLD(YELLOW(" >>> Sensor failure reported by the watchdog <<< \n"));
            std::cout << BOLD(YELLOW(" >>> Error code:  " + std::to_string(status.entity) + std::to_string(status.type) + std::to_string(status.event) + "\n"));
            std::cout << BOLD(YELLOW(" >>> Entity:      " + entity + "\n"));
            std::cout << BOLD(YELLOW(" >>> Type:        " + type + "\n"));
            std::cout << BOLD(YELLOW("-------------------------------------------------\n")) << std::endl;

            // Failure -- search an action (next state) on specific mission (mission id) [Next states strings can be: continue, hold, land, failure]
            // If the state string is not continue then, if in_flight_, call state transition otherwise, trigger a failure
            if (missions_.at(mission_id_).getNextState(status.entity).compare("continue") != 0) {
              if (!in_flight_) {
                stateTransition("failure");
              } else {
                stateTransition(missions_.at(mission_id_).getNextState(status.entity));
              }
            }

            // Always perform an action to react to the failure (the action can be NOTHING)
            watchdogActionRequest(status, it);

          } else if (status.event == Event::ENTITY_FIX) {

            // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromEntity
            if (!getStringFromEntity(status.entity, entity)) {
              std::cout << BOLD(RED(" >>> No string defined for required entity: " + std::to_string(status.entity) + "\n")) << std::endl;
            }

            // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
            if (!getStringFromType(status.type, type)) {
              std::cout << BOLD(RED(" >>> No string defined for required type: " + std::to_string(status.type) + "\n")) << std::endl;
            }

            std::cout << BOLD(GREEN("-------------------------------------------------\n"));
            std::cout << BOLD(GREEN(" >>> Sensor fix reported by the watchdog <<< \n"));
            std::cout << BOLD(GREEN(" >>> Entity:      " + entity + "\n"));
            std::cout << BOLD(GREEN(" >>> Type:        " + type + "\n"));
            std::cout << BOLD(GREEN("-------------------------------------------------\n")) << std::endl;

            // Fix -- At this stage the pending failure relative to the fix has already been removed.
            // Check if pending failure size == 0 then stop holding and resulme the mission otherwise keep holding
            if (pending_failures_.size() == 0) {

              // Redundant check, check if we are flying (we should not be here if we are not flying)
              if (in_flight_) {

                // Call state transition to PERFORM_MISSION
                stateTransition("perform_mission");

                // Resume mission
                missionSequencerRequest(mission_sequencer::MissionRequest::RESUME);

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

  void Autonomy::watchdogActionRequest(SensorStatus& status, const watchdog_msgs::Status& status_msg) {

    // Check existence of subscribers
    if (pub_watchdog_action_.getNumSubscribers() > 0) {

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
      std::cout << BOLD(YELLOW(" >>> Action communicated to the watchdog.\n")) << std::endl;
      pub_watchdog_action_.publish(action_msg);

    } else {

      // Print info
      std::cout << BOLD(YELLOW(" >>> No subscribers for watchdog action. Action will be ignored.\n")) << std::endl;

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

      // Call state transition to END MISSION
      stateTransition("end_mission");

    } else {

      // Print info
      std::cout << BOLD(RED(" >>> Unexpected land detected <<<\n")) << std::endl;

      // Call state transition to LAND
      stateTransition("land");
    }

  }

  void Autonomy::missionSequencerResponceCallback(const mission_sequencer::MissionResponseConstPtr& msg) {

    // Define request
    std::string req;

    // Get request and check
    if (getRequestfromMsg(msg->request, req)) {

      // Check pending request flag
      if (ms_request_pending_) {

        // Reset pending request flag
        ms_request_pending_ = false;

        // Check if mission sequencer request has been accepted
        if (msg->response && msg->request.request != mission_sequencer::MissionRequest::UNDEF) {
          std::cout << BOLD(GREEN(" >>> Request [" + req + "] accepted from Mission Sequencer.\n")) << std::endl;

          // Acts specifically based on the request
          switch (msg->request.request) {

          case mission_sequencer::MissionRequest::ARM:

            break;

          case mission_sequencer::MissionRequest::TAKEOFF:

            // Set in_flight_ and reset last_waypoint_reached_ flag
            in_flight_ = true;
            last_waypoint_reached_ = false;

            // Subscribe to landing after taking off if detection if active
            if (opts_->activate_landing_detection) {
              sub_landing_detection_ = nh_.subscribe(opts_->landing_detection_topic, 1, &Autonomy::landingDetectionCallback, this);
            }

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

            // If we are hovering and a lend is requested then set hovering_ flag
            if (hovering_) {
              hovering_ = false;
            }

            // Check if landing detection is active, if not transit to END_MISSION
            if (!opts_->activate_landing_detection && land_expected_) {
              stateTransition("end_mission");
            }

            break;

          case mission_sequencer::MissionRequest::HOVER:

            // Set hovering_ flag
            hovering_ = true;

            break;

          case mission_sequencer::MissionRequest::DISARM:

            // Reset armed_ flag
            armed_ = false;

            // Stop flight timer
            if (!multiple_touchdowns_ || (filepaths_cnt_ == static_cast<int>(missions_.at(mission_id_).getTouchdowns()))) {
              flight_timer_->stopTimer();
            }

            break;

          }
        }

        // Check if mission sequencer request has been rejected
        if (!msg->response && !msg->completed) {
          std::cout << BOLD(RED(" >>> Request [" + req + "] for mission ID: " + std::to_string(msg->request.id) + "  rejected from Mission Sequencer.\n")) << std::endl;
          stateTransition("failure");
        }

        // Check if mission has been compoleted (last waypoint reached)
      } else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::UNDEF) {

        std::cout << BOLD(GREEN(" >>> Mission ID: " + std::to_string(msg->request.id) + " succesfully reached last waypoint.\n")) << std::endl;

        // set last_waypoint_reached_ flag
        last_waypoint_reached_ = true;

        // Check parameter server
        bool hover_after_mission_completion;
        nh_.getParam("hover_after_mission_completion", hover_after_mission_completion);

        // Check if the parameter has changed from previously assigned value
        if (hover_after_mission_completion != opts_->hover_after_mission_completion) {
          // Assign new flag
          opts_->hover_after_mission_completion = hover_after_mission_completion;
          // Print info
          std::cout << BOLD(YELLOW(" >>> Changed behaviour at mission completion: Hover is now " + opts_->getStringfromBool(opts_->hover_after_mission_completion) + "\n")) << std::endl;
        }

        // Call state transition to LAND or request HOVER to mission sequencer based on param
        if (!opts_->hover_after_mission_completion) {
          stateTransition("land");
        } else {
          // Print info
          std::cout << BOLD(GREEN(" >>> Hovering...\n")) << std::endl;

          // Request hovering to the mission sequencer if not hovering
          if (!hovering_) {
            missionSequencerRequest(mission_sequencer::MissionRequest::HOVER);
          } else {
            std::cout << BOLD(YELLOW(" >>> the platform is already hovering, skipped HOVER request\n")) << std::endl;
          }
        }

        // Check if arm request has been completed
      } else if (!msg->response && msg->completed && msg->request.request == mission_sequencer::MissionRequest::ARM) {

        // Set armed_ flag
        armed_ = true;

        // Reset flight_timer only at first arming (if not active)
        if (!flight_timer_->isActive()) {
          flight_timer_->resetTimer();
        }

      } else {
        std::cout << BOLD(YELLOW(" >>> Received responce from mission sequencer without a prior request. Ignoring it.\n")) << std::endl;
      }
    } else {
      std::cout << BOLD(YELLOW(" >>> Received responce from mission sequencer to a unknown request. Ignoring it.\n")) << std::endl;
    }
  }

  void Autonomy::missionSequencerRequest(const int& request) {

    // Check existence of subscribers
    if (pub_mission_sequencer_request_.getNumSubscribers() > 0) {

      // Define request message to mission sequencer
      mission_sequencer::MissionRequest req;

      // Set mission id and request
      req.header.stamp = ros::Time::now();
      req.id = uint8_t(mission_id_);
      req.request = uint8_t(request);

      // publish mission start request
      pub_mission_sequencer_request_.publish(req);

      // Set flag
      ms_request_pending_ = true;

    } else {

      // Print info
      std::cout << BOLD(RED(" >>> No subscribers for mission sequencer request.\n")) << std::endl;

      // At this stage we are not flying yet, state transition to FAILURE
      stateTransition("failure");

    }

  }

  bool Autonomy::startWatchdog() {

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
        sub_watchdog_heartbeat_ = nh_.subscribe(opts_->watchdog_heartbeat_topic, 1, &Autonomy::watchdogHeartBeatCallback, this);

        // Subscribe to watchdog status changes
        sub_watchdog_status_ = nh_.subscribe(opts_->watchdog_status_topic, 1, &Autonomy::watchdogStatusCallback, this);

        // Success
        return true;

      } else {

        std::cout << BOLD(RED(" >>> FAILED TO START WATCHDOG --- Please perform a system hard restart <<< \n")) << std::endl;

        // Define status to get debug info
        SensorStatus status;

        // Get timestamp
        status.timestamp = watchdog_start.response.header.stamp.toSec();

        // print debug info
        if (getSensorStatusFromMsg(watchdog_start.response.status, status)) {

          // Define auxilliary strings
          std::string entity;
          std::string type;

          // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromEntity
          if (!getStringFromEntity(status.entity, entity)) {
            std::cout << BOLD(RED(" >>> No string defined for required entity: " + std::to_string(status.entity) + "\n")) << std::endl;
          }

          // Redundant check, this should never fail, if so not all the possible choices are coverd by getStringFromType
          if (!getStringFromType(status.type, type)) {
            std::cout << BOLD(RED(" >>> No string defined for required type: " + std::to_string(status.type) + "\n")) << std::endl;
          }

          std::cout << BOLD(RED("------------------------------------------------\n"));
          std::cout << BOLD(RED(" DEBUG INFORMATION\n"));
          std::cout << BOLD(RED(" - Entity:      " + entity + "\n"));
          std::cout << BOLD(RED(" - Type:        " + type + "\n"));
          std::cout << BOLD(RED(" - Debug name : " + status.debug_name + "\n"));
          std::cout << BOLD(RED(" - Debug info : " + status.debug_info + "\n"));
          std::cout << BOLD(RED("-------------------------------------------------\n")) << std::endl;
        }

        // Failure
        return false;
      }
    } else {

      // Failure
      return false;
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

    std::cout << BOLD(GREEN("\n >>> Loaded mission with ID: " << std::to_string(mission_id_) << "\n")) << std::endl;

    if (missions_.at(mission_id_).getTouchdowns() > 0) {
      std::cout << BOLD(YELLOW(" >>> Mission with Multiple touchdowns: " << std::to_string(missions_.at(mission_id_).getTouchdowns()) << " touchdown(s)\n")) << std::endl;
      multiple_touchdowns_ = true;
    }
  }

  bool Autonomy::preFlightChecks() {

    std::cout << BOLD(GREEN(" >>> Starting Pre-Flight Checks...\n")) << std::endl;

    if (opts_->perform_takeoff_check && !takeoffChecks()) {
      return false;
    }

    if (opts_->perform_estimator_check && !estimatorCheck()) {
      return false;
    }

    // Trigger State estimation initialization if a cascade of estimators are used
    if (opts_->estimator_init_service && !initializeStateEstimation()) {
      return false;
    }

    std::cout << BOLD(GREEN(" >>> Pre-Flight checks successed\n")) << std::endl;
    return true;

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

  void Autonomy::startAutonomy() {

    // Mission selection
    missionSelection();

    // Setting state to INITIALIZATION and initialize the system (watchdog)
    // This will trigger a state transition to either NOMINAL or FAILURE
    // depending by the result of the initialization
    // If the state is set to NOMINAL after succesfull initialization the data
    // recording will start if enabled and the waypoints for the selected mission will be loaded
    stateTransition("initialization");
  }

  void Autonomy::stateTransition(std::string str) {

    // Execute predefined behavior on Exiting old state
    state_->onExit(*this);

    // Execute state transition
    if (str.compare("undefined") == 0) {
      state_ = &Undefined::Instance();
    } else if (str.compare("initialization") == 0) {
      state_ = &Initialization::Instance();
    } else if (str.compare("nominal") == 0) {
      state_ = &Nominal::Instance();
    } else if (str.compare("failure") == 0) {
      state_ = &Failure::Instance();
    } else if (str.compare("preflight") == 0) {
      state_ = &Preflight::Instance();
    } else if (str.compare("start_mission") == 0) {
      state_ = &StartMission::Instance();
    } else if (str.compare("perform_mission") == 0) {
      state_ = &PerformMission::Instance();
    } else if (str.compare("end_mission") == 0) {
      state_ = &EndMission::Instance();
    } else if (str.compare("land") == 0) {
      state_ = &Land::Instance();
    } else if (str.compare("hold") == 0) {
      state_ = &Hold::Instance();
    } else if (str.compare("termination") == 0) {
      state_ = &Termination::Instance();
    } else {
      std::cout << BOLD(RED(" >>> Wrong state transition required.\n")) << std::endl;
      state_ = &Failure::Instance();
    }

    // Execute predefined behavior on Entering new state
    state_->onEntry(*this);
  }

} // namespace autonomy

// TODOS:
