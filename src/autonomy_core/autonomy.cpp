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

#include "autonomy_core/autonomy.h"
#include "utils/colors.h"

AmazeAutonomy::AmazeAutonomy(ros::NodeHandle &nh) :
  nh_(nh), reconfigure_cb_(boost::bind(&AmazeAutonomy::configCallback, this, _1, _2)) {

  // Parse parameters and options
  if(!parseParams()) {
      throw std::exception();
  }

  // Print option
  opts_->printAutonomyOptions();

  // Set dynamic reconfigure callback
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Advertise watchdog service
  watchdog_start_service_client_ = nh_.serviceClient<watchdog_msgs::Start>(opts_->watchdog_start_service_name);

  // Advertise takeoff service
  takeoff_service_client_ = nh_.serviceClient<std_srvs::Trigger>(opts_->takeoff_service_name);

  // Advertise data recording service
  data_recording_service_client_ = nh_.serviceClient<std_srvs::SetBool>(opts_->data_recrding_service_name);

  // Advertise watchdog action topic
  pub_watchdog_action_ = nh.advertise<watchdog_msgs::ActionStamped>(opts_->watchdog_action_topic, 10);

  // Advertise mission sequencer request topic
  pub_mission_sequencer_request_ = nh.advertise<amaze_mission_sequencer::request>(opts_->mission_sequencer_request_topic, 10);

  // Instanciate timeout timer and connect signal
  timer_ = std::make_shared<Timer>(opts_->timeout);
  timer_->sh_.connect(boost::bind(&AmazeAutonomy::watchdogTimerOverflowHandler, this));
}

bool AmazeAutonomy::parseParams() {

  // Define auxilliary variables
  std::string watchdog_start_service_name, watchdog_heartbeat_topic, watchdog_status_topic, watchdog_action_topic, mission_sequencer_request_topic, mission_sequencer_responce_topic, data_recrding_service_name, takeoff_service_name, landing_detection_topic, description;
  double watchdog_rate, watchdog_heartbeat_timeout_multiplier;
  int watchdog_startup_time_s, n_missions, watchdog_timeout_ms;
  std::map<size_t, std::string> missions;
  Entity entity;
  AutonomyState state;
  XmlRpc::XmlRpcValue entities_actions;
  std::vector<std::pair<size_t, std::pair<Entity, AutonomyState>>> entity_state_vector;

  // Get topics and service names
  if(!nh_.getParam("watchdog_start_service_name", watchdog_start_service_name)) {
    std::cout << std::endl << BOLD(RED(" >>> [watchdog_start_service_name] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("watchdog_heartbeat_topic", watchdog_heartbeat_topic)) {
    std::cout << std::endl << BOLD(RED(" >>> [watchdog_heartbeat_topic] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("watchdog_status_topic", watchdog_status_topic)) {
    std::cout << std::endl << BOLD(RED(" >>> [watchdog_status_topic] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("watchdog_action_topic", watchdog_action_topic)) {
    std::cout << std::endl << BOLD(RED(" >>> [watchdog_action_topic] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("mission_sequencer_request_topic", mission_sequencer_request_topic)) {
    std::cout << std::endl << BOLD(RED(" >>> [mission_sequencer_request_topic] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("mission_sequencer_responce_topic", mission_sequencer_responce_topic)) {
    std::cout << std::endl << BOLD(RED(" >>> [mission_sequencer_responce_topic] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("data_recrding_service_name", data_recrding_service_name)) {
    std::cout << std::endl << BOLD(RED(" >>> [data_recrding_service_name] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("takeoff_service_name", takeoff_service_name)) {
    std::cout << std::endl << BOLD(RED(" >>> [takeoff_service_name] parameter not defined")) << std::endl;
    return false;
  }
  if(!nh_.getParam("landing_detection_topic", landing_detection_topic)) {
    std::cout << std::endl << BOLD(RED(" >>> [landing_detection_topic] parameter not defined")) << std::endl;
    return false;
  }

  // Get watchdog rate
  if(!nh_.getParam("watchdog_rate_Hz", watchdog_rate)) {
     std::cout << std::endl << BOLD(RED(" >>> [watchdog_rate_Hz] parameter not defined. Please set it lower than 5Hz")) << std::endl;
     return false;
  }

  // Set watchdog timer timeout to be a given percentage of 1/watchdog_rate
  if(!nh_.getParam("watchdog_heartbeat_timeout_multiplier", watchdog_heartbeat_timeout_multiplier)) {
    std::cout << std::endl << BOLD(RED(" >>> [watchdog_heartbeat_timeot_multiplier] paramter not defined. This set the heartbeat timer timeout to be a scaled version of the period of the watchdog rate. Please set it higher than 1")) << std::endl;
    return false;
  } else {
    watchdog_timeout_ms = static_cast<int>(std::ceil((1000*watchdog_heartbeat_timeout_multiplier)/watchdog_rate));
  }

  // Get watchdog startup time
  if(!nh_.getParam("watchdog_startup_time_s", watchdog_startup_time_s)) {
    std::cout << std::endl << BOLD(RED(" >>> [watchdog_startup_time_s] paramter not defined. Please set it higher than 10")) << std::endl;
    return false;
  }

  // Get missions information
  if(!nh_.getParam("missions/number", n_missions)) {
    std::cout << std::endl << BOLD(RED(" >>> [missions/number] paramter not defined. Please set it on mission config file")) << std::endl;
    return false;
  }

  // Check whether missions are defined and parse them
  if (n_missions == 0) {
    std::cout << std::endl << BOLD(RED(" >>> No missions defined")) << std::endl;
    return false;
  } else {

    // Loop through missions
    for (int i = 1; i <= n_missions; ++i) {

      // get mission description
      if (!nh_.getParam("missions/mission_" + std::to_string(i) + "/description", description)) {
        std::cout << std::endl << BOLD(RED(" >>> Mission number " + std::to_string(i) + " description missing")) << std::endl;
        return false;
      }

      // Build Mission map <i, description>
      missions.insert({size_t(i), description});

      // get entities and actions
      if (!nh_.getParam("missions/mission_" + std::to_string(i) + "/entities_actions", entities_actions)) {
        std::cout << std::endl << BOLD(RED(" >>> Mission number " + std::to_string(i) + " entities_actions list missing")) << std::endl;
        return false;
      }

      // Check type to be array
      if (entities_actions.getType() ==  XmlRpc::XmlRpcValue::TypeArray) {

        // Loop through entities and actions
        for (int j = 0; j < entities_actions.size(); ++j) {

          // Check type to be array
          if (entities_actions[j].getType() ==  XmlRpc::XmlRpcValue::TypeArray) {

            // Check type to be string and get entity
            if (entities_actions[j][0].getType() == XmlRpc::XmlRpcValue::TypeString) {
              if (!getEntityFromString(std::string(entities_actions[j][0]), entity)) {
                std::cout << std::endl << BOLD(RED(" >>> Mission number " + std::to_string(i) + " wrong entity definition in entities_actions list")) << std::endl;
                return false;
              }
            }

            // Check type to be string and get action
            if (entities_actions[j][1].getType() == XmlRpc::XmlRpcValue::TypeString) {
              if (!getNextStateFromString(std::string(entities_actions[j][1]), state)) {
                std::cout << std::endl << BOLD(RED(" >>> Mission number " + std::to_string(i) + " wrong action definition in entities_actions list")) << std::endl;
                return false;
              }
            }

            // Build Entity Action vector
            entity_state_vector.emplace_back(std::make_pair(size_t(i), std::make_pair(entity, state)));

          } else {
            std::cout << std::endl << BOLD(RED(" >>> Mission number " + std::to_string(i) + " entities_actions list wrongly defined")) << std::endl;
            return false;
          }
        }
      } else {
        std::cout << std::endl << BOLD(RED(" >>> Mission number " + std::to_string(i) + " entities_actions list wrongly defined")) << std::endl;
        return false;
      }
    }
  }

  // Make options
  opts_ = std::make_shared<autonomyOptions>(autonomyOptions({watchdog_start_service_name, watchdog_heartbeat_topic, watchdog_status_topic, watchdog_action_topic, mission_sequencer_request_topic, mission_sequencer_responce_topic, data_recrding_service_name, takeoff_service_name, landing_detection_topic, watchdog_timeout_ms, watchdog_startup_time_s, missions, entity_state_vector}));

  // Success
  return true;
}

bool AmazeAutonomy::getEntityTypeSubTypeFromMsg(const watchdog_msgs::Status& msg, Entity& entity, Type& type, subType& subtype) {

  // Get entity
  if (!getEntityFromString(msg.entity, entity)) {
    return false;
  }

  // Get type
  switch(msg.status) {
  case watchdog_msgs::Status::NOMINAL:
    type = Type::FIX;
    break;
  case watchdog_msgs::Status::ERROR:
    type = Type::FAILURE;
    break;
  default:
    type = Type::OTHER;
    break;
  }

  // Get subtype
  switch (msg.type) {
  case watchdog_msgs::Status::GLOBAL:
    subtype = subType::GLOBAL;
    break;
  case watchdog_msgs::Status::TOPIC:
    subtype = subType::TOPIC;
    break;
  case watchdog_msgs::Status::NODE:
    subtype = subType::NODE;
    break;
  case watchdog_msgs::Status::DRIVER:
    subtype = subType::DRIVER;
    break;
  }

  return true;
}

bool AmazeAutonomy::getEntityFromString(const std::string entity_str, Entity& entity) {

  // Check if it is either 0 or 1, this means that either the string
  // match or there is a char more that could be \n
  if ((entity_str.compare("px4_imu") >> 1) == 0) {
    entity = Entity::PX4_IMU;
  } else if ((entity_str.compare("px4_gps") >> 1) == 0) {
    entity = Entity::PX4_GPS;
  } else if ((entity_str.compare("px4_bar") >> 1) == 0) {
    entity = Entity::PX4_BAR;
  } else if ((entity_str.compare("px4_mag") >> 1) == 0) {
    entity = Entity::PX4_MAG;
  } else if ((entity_str.compare("mission_cam") >> 1) == 0) {
    entity = Entity::MISSION_CAM;
  } else if ((entity_str.compare("realsense") >> 1) == 0) {
    entity = Entity::REALSENSE;
  } else if ((entity_str.compare("lsm9ds1") >> 1) == 0) {
    entity = Entity::LSM9DS1;
  } else if ((entity_str.compare("lrf") >> 1) == 0) {
    entity = Entity::LRF;
  } else if ((entity_str.compare("rtk_gps_1") >> 1) == 0) {
    entity = Entity::RTK_GPS_1;
  } else if ((entity_str.compare("rtk_gps_2") >> 1) == 0) {
    entity = Entity::RTK_GPS_2;
  } else if (entity_str.compare("") == 0) {
    entity = Entity::UNKNOWN;
  } else {
    return false;
  }

  return true;
}

bool AmazeAutonomy::getNextStateFromString(const std::string action_str,  AutonomyState& state) {

  if (action_str.compare("continue") == 0) {
    state = AutonomyState::NOMINAL;
  } else if (action_str.compare("hold") == 0) {
    state = AutonomyState::HOLD;
  } else if (action_str.compare("manual") == 0) {
    state = AutonomyState::MANUAL;
  } else {
    return false;
  }

  return true;
}

bool AmazeAutonomy::getEntityString(const Entity& entity, std::string entity_str) {

  switch (entity) {
  case Entity::UNKNOWN:
    entity_str = "";
    break;
  case Entity::PX4_IMU:
    entity_str = "px4_imu";
    break;
  case Entity::PX4_GPS:
    entity_str = "px4_gps";
    break;
  case Entity::PX4_BAR:
    entity_str = "px4_bar";
    break;
  case Entity::PX4_MAG:
    entity_str = "px4_mag";
    break;
  case Entity::MISSION_CAM:
    entity_str = "mission_cam";
    break;
  case Entity::REALSENSE:
    entity_str = "realsense";
    break;
  case Entity::LSM9DS1:
    entity_str = "lsm9ds1";
    break;
  case Entity::LRF:
    entity_str = "lrf";
    break;
  case Entity::RTK_GPS_1:
    entity_str = "rtk_gps_1";
    break;
  case Entity::RTK_GPS_2:
    entity_str = "rtk_gps_2";
    break;
  }

  return true;

}

bool AmazeAutonomy::setStatusMsgFromEntityTypeSubType(const Entity& entity, const Type& type, const subType& subtype, watchdog_msgs::Status& msg) {

  // Set entity
  if(!getEntityString(entity, msg.entity)) {
    return false;
  }

  // Set status
  switch(type) {
  case Type::FIX:
    msg.type = watchdog_msgs::Status::NOMINAL;
    break;
  case Type::FAILURE:
    msg.type = watchdog_msgs::Status::ERROR;
    break;
  default:
    msg.type = watchdog_msgs::Status::UNDEF;
    break;
  }


  // Set type
  switch (subtype) {
  case subType::GLOBAL:
    msg.type = watchdog_msgs::Status::GLOBAL;
    break;
  case subType::TOPIC:
    msg.type = watchdog_msgs::Status::TOPIC;
    break;
  case subType::NODE:
    msg.type = watchdog_msgs::Status::NODE;
    break;
  case subType::DRIVER:
    msg.type = watchdog_msgs::Status::DRIVER;
    break;
  }

  return true;
}

bool AmazeAutonomy::setActionMsg(const Action& action, const EntityEvent entityevent, watchdog_msgs::Action& msg) {

  // Assign action
  switch (action) {
  case Action::NOTHING:
    msg.action =  watchdog_msgs::Action::NOTHING;
    break;
  case Action::RESTART_NODE:
    msg.action =  watchdog_msgs::Action::RESTART_NODE;
    break;
  case Action::RESTART_DRIVER:
    msg.action =  watchdog_msgs::Action::RESTART_DRIVER;
    break;
  case Action::KILL_NODE:
    msg.action =  watchdog_msgs::Action::KILL_NODE;
    break;
  }

  // Assign status
  if (!setStatusMsgFromEntityTypeSubType(entityevent.getEntity(), entityevent.getType(), entityevent.getSubType(), msg.entity)) {
    return false;
  }

  return true;
}

void AmazeAutonomy::watchdogHeartBeatCallback(const watchdog_msgs::StatusStampedConstPtr&) {

  // Restart timeout timer
  timer_->resetTimer();
}

void AmazeAutonomy::watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStampedConstPtr& msg) {

  // Loop through all the chenges with respect to last iteration
  for (const auto &it : msg->data.changes) {

    // Define EntityEvent
    Entity entity;
    Type type;
    subType subtype;
    AutonomyState nextstate;
    EntityEvent event;

    // Parse information of status changes
    if (getEntityTypeSubTypeFromMsg(it, entity, type, subtype)) {

      if (type != Type::OTHER) {

        // Search on entity_action_vector the next state for the given status change
        auto it = find_if(opts_->entity_action_vector.begin(), opts_->entity_action_vector.end(), [this, &entity](const std::pair<size_t, std::pair<Entity, AutonomyState>>& entity_nextstate){ return entity_nextstate.first == mission_id_ && entity_nextstate.second.first == entity;});

        // Check if a correspondence has been found
        if (it != opts_->entity_action_vector.end()) {

          // Set event
          nextstate = it->second.second;
          event.setEvent(entity, type, subtype, nextstate);

          // Call state transition
          // It return false if the required transition is not handled (e.g., FIX without a previous FAILURE)
          if (state_.stateTransition(event)) {

            // Get actual state and send request to mission sequencer
            if (!holding_ && state_.getState() == AutonomyState::HOLD) {
              missionSequencerRequest(amaze_mission_sequencer::request::HOLD);
              holding_ = true;
            } else if (holding_ && state_.getState() == AutonomyState::NOMINAL) {
              missionSequencerRequest(amaze_mission_sequencer::request::RESUME);
              holding_ = false;
            } else if (state_.getState() == AutonomyState::MANUAL) {
              missionSequencerRequest(amaze_mission_sequencer::request::ABORT);
              throw std::exception();
            }

            // Get action <Action, EntityEvent> to be performed from the state machine
            auto action = state_.getAction();

            // Check wether an action has to be performed and perform it
            if (action.first != Action::NOTHING) {
              watchdogActionRequest(action);
            }
          } else {
            std::cout << std::endl << BOLD(RED(" >>> Wrong state transition required.")) << std::endl;
          }
        } else {
          std::cout << std::endl << BOLD(RED(" >>> No Entity-Action defined on current mission for the current status changes.")) << std::endl;
        }
      } else {
        if (it.status == watchdog_msgs::Status::DEFECT) {
          std::cout << std::endl << BOLD(YELLOW(" >>> Defect detected. No action needed.")) << std::endl;
        }
      }
    } else {
      std::cout << std::endl << BOLD(RED(" >>> Wrong message received from watchdog.")) << std::endl;
    }
  }
}

void AmazeAutonomy::watchdogTimerOverflowHandler() {

  // print message of watchdog timer overflow
  std::cout << std::endl << BOLD(RED(" >>> Timeout overflow -- no heartbeat from system watchdog")) << std::endl;
}

void AmazeAutonomy::configCallback(amaze_autonomy::autonomyConfig&, uint32_t) {
}

void AmazeAutonomy::landingDetectionCallback(const std_msgs::BoolConstPtr& msg) {

  if (msg) {
    std::cout << std::endl << BOLD(GREEN(" >>> Flat land detected.")) << std::endl;
  } else {
    std::cout << std::endl << BOLD(YELLOW(" >>> Non flat land detected.")) << std::endl;
  }

  // Stop data recording after landing in case mission has been succesfully completed
  if (last_waypoint_reached_) {
    DataRecording(false);
  } else {
    // [TODO]
    std::cout << std::endl << BOLD(YELLOW(" >>> Unexpected land detected. What should we do?")) << std::endl;
  }



}

void AmazeAutonomy::missionSequencerResponceCallback(const amaze_mission_sequencer::responseConstPtr& msg) {

  // Check if mission sequencer request has been accepted or if mission has ended
  if (msg->response) {
    std::cout << std::endl << BOLD(GREEN(" >>> Mission ID: " + std::to_string(msg->id) + " accepted from Mission Sequencer")) << std::endl;
  } else {
    std::cout << std::endl << BOLD(RED(" >>> Mission ID: " + std::to_string(msg->id) + "  rejected from Mission Sequencer")) << std::endl;
  }

  if (msg->completed) {
    std::cout << std::endl << BOLD(GREEN(" >>> Mission ID: " + std::to_string(msg->id) + " succesfully reached last waypoint")) << std::endl;
    last_waypoint_reached_ = true;
  }
}

void AmazeAutonomy::startWatchdog() {

  // Define service request
  watchdog_msgs::Start watchdog_start;
  watchdog_start.request.header.stamp = ros::Time::now();
  watchdog_start.request.startup_time = opts_->watchdog_startup_time;

  std::cout << std::endl << BOLD(GREEN(" >>> Starting Watchdog... Please wait")) << std::endl;

  // Call service request
  if (watchdog_start_service_client_.call(watchdog_start)) {

    // Check responce
    if(watchdog_start.response.successful) {

      std::cout << std::endl << BOLD(GREEN(" >>> Watchdog is running")) << std::endl << std::endl;

      // Subscriber to watchdog (system status) heartbeat
      sub_watchdog_heartbeat_ = nh_.subscribe("/watchdog/status", 1, &AmazeAutonomy::watchdogHeartBeatCallback, this);

      // Subscribe to watchdog status changes
      sub_watchdog_status_ = nh_.subscribe("/watchdog/log", 1, &AmazeAutonomy::watchdogStatusCallback, this);

      // Setting state to nominal
      state_.nominal();
    }
  } else {
    watchdog_start.response.successful = false;
  }

  if (!watchdog_start.response.successful) {

    std::cout << std::endl << BOLD(RED("----------- FAILED TO START WATCHDOG ------------")) << std::endl << std::endl;
    std::cout << BOLD(RED(" Please perform a system hard restart  ")) << std::endl;
    std::cout << BOLD(RED(" If you get the same problem after the ")) << std::endl;
    std::cout << BOLD(RED(" hard restart, shutdown the system and ")) << std::endl;
    std::cout << BOLD(RED(" abort the mission. ")) << std::endl << std::endl;
    std::cout << BOLD(RED("-------------------------------------------------")) << std::endl;

    // Define variables for debug
    Entity entity;
    std::string entity_str;
    Type type;
    subType subtype;

    // Print debug information
    if (getEntityTypeSubTypeFromMsg(watchdog_start.response.status, entity, type, subtype)) {
      if (getEntityString(entity, entity_str)) {
        std::cout << std::endl << BOLD(RED(" >>> Failed Entity: " + entity_str + "")) << std::endl;
      }
      std::cout << BOLD(RED(" >>> Failed Type: " + std::to_string(type) + "")) << std::endl;
      std::cout << BOLD(RED(" >>> Failed subType: " + std::to_string(subtype) + "")) << std::endl;
    } else {
      std::cout << std::endl << BOLD(RED(" >>> Unable to get debug information from watchdog")) << std::endl;
    }

    throw std::exception();
  }

}

void AmazeAutonomy::missionSelection() {

  // Print missions
  std::cout << std::endl << BOLD(GREEN(" >>> Please select one of the following mission by inserting the mission ID")) << std::endl << std::endl;
  for (auto &it : opts_->missions) {
    std::cout << BOLD(GREEN("      - ID: ")) << it.first << BOLD(GREEN(" DESCRIPTION: ")) << it.second << std::endl;
  }

  // Get ID of mission being executed
  std::cout << std::endl << BOLD(GREEN(" >>> Mission ID: "));
  std::cin >> mission_id_;

  // Check validity of mission id
  if (mission_id_ == 0 || mission_id_ > opts_->missions.size()) {
    std::cout << std::endl << BOLD(RED(" >>> Wrong mission ID chosen")) << std::endl;
    throw std::exception();
  }

  std::cout << std::endl << BOLD(GREEN("      - Selected mission with ID: ")) << mission_id_ << std::endl;
}

void AmazeAutonomy::preFlightChecks() {

  std::cout << std::endl << BOLD(GREEN(" >>> Starting Pre-Flight Checks... Please wait")) << std::endl;

  // if (!check1() && !check2() && ...) {
  if (!takeoffChecks()) {
    throw std::exception();
  }

  std::cout << std::endl << BOLD(GREEN(" >>> Pre-Flight checks successed")) << std::endl;

}

bool AmazeAutonomy::takeoffChecks() {

  // Define takeoff request
  std_srvs::Trigger takeoff;

  // service call to check if we are ready to takeoff
  if (takeoff_service_client_.call(takeoff)) {

    // Check responce
    if(takeoff.response.success) {
      std::cout << std::endl << BOLD(GREEN(" >>> Vehicle flat on the ground")) << std::endl;
      std::cout << std::endl << BOLD(GREEN(" >>> Takeoff checks successed")) << std::endl << std::endl;
    }
  } else {
    takeoff.response.success = false;
  }

  if (!takeoff.response.success) {
    std::cout << std::endl << BOLD(RED(" >>> Takeoff checks failed")) << std::endl << std::endl;
    return false;
  }

  return true;
}

void AmazeAutonomy::DataRecording(const bool& start_stop) {

  // Define data recording
  std_srvs::SetBool data_rec;

  // Set data recording start request
  data_rec.request.data = start_stop;

  // service call to check if we are ready to takeoff
  if (data_recording_service_client_.call(data_rec)) {

    // Check responce
    if (data_rec.response.success) {
      if (data_rec.request.data) {
        std::cout << std::endl << BOLD(GREEN(" >>> Data recorded started succesfully")) << std::endl;
      } else {
        std::cout << std::endl << BOLD(GREEN(" >>> Data recorded stopped succesfully")) << std::endl;
      }
    }
  } else {
    if (data_rec.request.data) {
      std::cout << std::endl << BOLD(RED(" >>> Data recorded start failure")) << std::endl;
    } else {
      std::cout << std::endl << BOLD(RED(" >>> Data recorded stop failure")) << std::endl;
    }
    throw std::exception();
  }
}

void AmazeAutonomy::missionSequencerRequest(const int& request) {

  // Define request message to mission sequencer
  amaze_mission_sequencer::request mission_start;

  // Set mission id and request
  mission_start.header.stamp = ros::Time::now();
  mission_start.id = uint8_t(mission_id_);
  mission_start.request = uint8_t(request);

  // publish mission start request
  pub_mission_sequencer_request_.publish(mission_start);

  // Subscribe to mission sequencer responce
  sub_mission_sequencer_responce_ = nh_.subscribe(opts_->mission_sequencer_responce_topic, 1, &AmazeAutonomy::missionSequencerResponceCallback, this);

  // Subscribe to landing detection
  sub_landing_detection_ = nh_.subscribe(opts_->landing_detection_topic, 1, &AmazeAutonomy::landingDetectionCallback, this);
}

void AmazeAutonomy::watchdogActionRequest(const std::pair<Action, EntityEvent>& action) {

  // Define action and action message
  watchdog_msgs::ActionStamped action_msg;

  // Fill action message
  action_msg.header.stamp = ros::Time::now();

  // Get Action, Entity, Type and Status from <Action, EntityEvent>
  // and convert them into an action message
  if (setActionMsg(action.first, action.second, action_msg.action)) {

    // Publish action message
    std::cout << std::endl << BOLD(YELLOW(" >>> Publishing action message.")) << std::endl;
    pub_watchdog_action_.publish(action_msg);

  } else {
    std::cout << std::endl << BOLD(RED(" >>> Error on generating action message.")) << std::endl;
  }
}

void AmazeAutonomy::startAutonomy() {

  // Mission selection
  missionSelection();

  // Start watchdog
  startWatchdog();

  // Subscriber to watchdog (system status) heartbeat
  sub_watchdog_heartbeat_ = nh_.subscribe("/watchdog/status", 1, &AmazeAutonomy::watchdogHeartBeatCallback, this);

  // Subscribe to watchdog status changes
  sub_watchdog_status_ = nh_.subscribe("/watchdog/log", 1, &AmazeAutonomy::watchdogStatusCallback, this);

  // Setting state to nominal
  state_.nominal();

  // Run pre flight checks
  preFlightChecks();

  // Start data recording
  // DataRecording(true);

  // Start mission
  missionSequencerRequest(amaze_mission_sequencer::request::START);

}
