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

#ifndef AMAZEAUTONOMY_H
#define AMAZEAUTONOMY_H

#include <ros/ros.h>
#include <watchdog_msgs/Start.h>
#include <watchdog_msgs/StatusStamped.h>
#include <watchdog_msgs/StatusChangesArrayStamped.h>
#include <watchdog_msgs/ActionStamped.h>
#include <amaze_mission_sequencer/request.h>
#include <amaze_mission_sequencer/response.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <string>
#include <xmlrpcpp/XmlRpcValue.h>

#include "autonomy_core/autonomy_options.h"
#include "timer/timer.h"
#include "state_machine/state.h"
#include "utils/except.h"

class AmazeAutonomy {

  public:

    /**
     * @brief Autonomy constructor
     * @param Ros NodeHandle
     */
    AmazeAutonomy(ros::NodeHandle &nh);

    /**
     * @brief Function that starts the autonomy and the interface with the user.
     */
    void startAutonomy();

  private:

    /**
     * @brief Watchdog start service call
     */
    void startWatchdog();

    /**
     * @brief Selection of the mission form the user
     */
    void missionSelection();

    /**
     * @brief Run preflight checks
     */
    void preFlightChecks();

    /**
     * @brief Send mission request to mission sequencer
     * @param const reference to int request
     */
    void missionSequencerRequest(const int& request);

    /**
     * @brief Send action to watchdog
     * @param const reference to std::pair<Action, EntityEvent>
     */
    void watchdogActionRequest(const std::pair<Action, EntityEvent>& action);

    /**
     * @brief Start/stop data recording service call
     * @param const boolean, true if data recording should be started, false if data recording should be stopped
     */
    void DataRecording(const bool& start_stop);

    /**
     * @brief Run takeoff checks
     * @return boolean true in case of success, false in case of failure
     */
    [[nodiscard]] bool takeoffChecks();

    /**
     * @brief Run vio checks
     * @return boolean true in case of success, false in case of failure
     */
    [[nodiscard]] bool vioChecks();

    /**
     * @brief Load and parse paramters and options
     * @return boolean true in case of success, false in case of failure
     */
    [[nodiscard]] bool parseParams();

    /**
     * @brief Watchdog (system status) heartbeat callback
     */
    void watchdogHeartBeatCallback(const watchdog_msgs::StatusStampedConstPtr& msg);

    /**
     * @brief Watchdog system status changes callback
     */
    void watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStampedConstPtr& msg);

    /**
     * @brief Landing detection callback
     */
    void landingDetectionCallback(const std_msgs::BoolConstPtr& msg);

    /**
     * @brief Mission sequencer responce callback
     */
    void missionSequencerResponceCallback(const amaze_mission_sequencer::responseConstPtr& msg);

    /**
     * @brief Callback method called when a watchdog timer overflow occurs
     */
    void watchdogTimerOverflowHandler();

    /**
     * @brief Get Entity, Type and subType from watchdog_msgs::Status
     * @param const reference to watchdog_msgs::Status
     * @param reference to Entity
     * @param reference to Type
     * @param reference to subType
     * @return boolean
     */
    [[nodiscard]] bool getEntityTypeSubTypeFromMsg(const watchdog_msgs::Status& msg, Entity& entity, Type& type, subType& subtype);

    /**
     * @brief Set watchdog_msgs::Status from Entity, Type and subType
     * @param const reference to Entity
     * @param const reference to Type
     * @param const reference to subType
     * @param reference to watchdog_msgs::Status
     * @return boolean
     */
    [[nodiscard]] bool setStatusMsgFromEntityTypeSubType(const Entity& entity, const Type& type, const subType& subtype, watchdog_msgs::Status& msg);

    /**
     * @brief Get Entity from string
     * @param const string
     * @param reference to Entity
     * @return boolean
     */
    [[nodiscard]] bool getEntityFromString(const std::string entity_str, Entity& entity);

    /**
     * @brief Get next state (AutonomyState) from string
     * @param const string
     * @param reference to Action
     * @return boolean
     */
    [[nodiscard]] bool getNextStateFromString(const std::string action_str,  AutonomyState& action);

    /**
     * @brief Get string from Entity
     * @param const reference to Entity
     * @param string
     * @return boolean
     */
    [[nodiscard]] bool getEntityString(const Entity& entity, std::string entity_str);

    /**
     * @brief Get Action from string
     * @param const reference to EntityEvent
     * @param const reference to Action
     * @param reference to watchdog_msgs::Action
     * @return boolean
     */
    [[nodiscard]] bool setActionMsg(const Action& action, const EntityEvent entityevent, watchdog_msgs::Action& msg);


    /**
     * @brief Callback to handle failure
     */
    [[noreturn]] void handleFailure();

    /**
     * @brief Callback to handle manual mode
     */
    [[noreturn]] void handleManual();

    /// Nodehandler
    ros::NodeHandle nh_;

    /// Autonomy options
    std::shared_ptr<autonomyOptions> opts_;

    /// Subscribers
    ros::Subscriber sub_watchdog_heartbeat_;
    ros::Subscriber sub_watchdog_status_;
    ros::Subscriber sub_landing_detection_;
    ros::Subscriber sub_mission_sequencer_responce_;

    /// Publishers
    ros::Publisher pub_watchdog_action_;
    ros::Publisher pub_mission_sequencer_request_;

    /// Service clients
    ros::ServiceClient watchdog_start_service_client_;
    ros::ServiceClient takeoff_service_client_;
    ros::ServiceClient data_recording_service_client_;
    ros::ServiceClient estimator_supervisor_service_client_;

    /// Timeout timer
    std::shared_ptr<Timer> timer_;

    /// Selected mission ID
    size_t mission_id_ = 0;

    /// State
    State state_;

    /// boolean to check if we are holding
    bool holding_ = false;

    /// Bboolean to check if last waypoint got reached
    bool last_waypoint_reached_ = false;

};

#endif  // AMAZEAUTONOMY_H
