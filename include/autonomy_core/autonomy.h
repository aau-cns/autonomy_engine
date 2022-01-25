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

#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <ros/ros.h>
#include <watchdog_msgs/Start.h>
#include <watchdog_msgs/StatusStamped.h>
#include <watchdog_msgs/StatusChangesArrayStamped.h>
#include <watchdog_msgs/ActionStamped.h>
#include <mission_sequencer/MissionRequest.h>
#include <mission_sequencer/MissionResponse.h>
#include <mission_sequencer/MissionWaypointArray.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <string>
#include <xmlrpcpp/XmlRpcValue.h>
#include <chrono>
#include <limits>

#include "autonomy_core/autonomy_options.h"
#include "autonomy_core/mission.h"
#include "timer/timer.h"
#include "utils/except.h"
#include "waypoints_parser/waypoints_parser.h"
#include "utils/logger.h"

namespace autonomy {

  class State;

  class Autonomy {

  public:

    /**
     * @brief Autonomy constructor
     * @param Ros NodeHandle
     */
    Autonomy(ros::NodeHandle &nh);

    /**
     * @brief Start the autonomy.
     */
    void startAutonomy();

    /// Autonomy options
    std::unique_ptr<autonomyOptions> opts_;

  private:

    /**
     * @brief Selection of the mission form the user
     */
    void missionSelection();

    /**
     * @brief Watchdog start service call
     * @return Boolean
     */
    [[nodiscard]] bool startWatchdog();

    /**
     * @brief Start/stop data recording service call
     * @param const boolean, true if data recording should be started, false if data recording should be stopped
     */
    void DataRecording(const bool& start_stop);

    /**
     * @brief Run preflight checks
     * @return Boolean
     */
    [[nodiscard]] bool preFlightChecks();

    /**
     * @brief Send action to watchdog
     * @param reference to SensorStatus
     * @param const reference to watchdog_msgs::Status
     */
    void watchdogActionRequest(SensorStatus& status, const watchdog_msgs::Status& status_msg);

    /**
     * @brief Run takeoff checks
     * @return boolean true in case of success, false in case of failure
     */
    [[nodiscard]] bool takeoffChecks();

    /**
     * @brief Run estimator check
     * @return boolean true in case of success, false in case of failure
     */
    [[nodiscard]] bool estimatorCheck();

    /**
     * @brief Load and parse paramters and options
     * @return boolean true in case of success, false in case of failure
     */
    [[nodiscard]] bool parseParams();

    /**
     * @brief Perform service call to initialize estimator
     * @return boolean
     */
    [[nodiscard]] bool initializeStateEstimation();

    /**
     * @brief Perform service call to start in flight sensor initialization
     * @return boolean
     */
    [[nodiscard]] bool InFlightSensorInit();

    /**
     * @brief Watchdog system status changes callback
     */
    void watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStampedConstPtr& msg);

    /**
     * @brief Landing detection callback
     */
    void landingDetectionCallback(const std_msgs::BoolConstPtr& msg);

    /**
     * @brief Send mission request to mission sequencer
     * @param const reference to int request
     */
    void missionSequencerRequest(const int& request);

    /**
     * @brief Mission sequencer responce callback
     */
    void missionSequencerResponceCallback(const mission_sequencer::MissionResponseConstPtr& msg);

     /**
     * @brief Get SensorStatus from watchdog_msgs::Status, this function will also
     * increase and decrease the pending_failures_ dependeng on weather we got a
     * failure or a fix
     *
     * @param const reference to watchdog_msgs::Status
     * @param reference to SensorStatus
     * @return Boolean
     */
    [[nodiscard]] bool getSensorStatusFromMsg(const watchdog_msgs::Status& msg, SensorStatus& status);

    /**
     * @brief Get string from mission_sequencer::MissionRequest
     * @param const reference to mission_sequencer::MissionRequest
     * @param Reference to std::string
     * @return Boolean
     */
    [[nodiscard]] bool getRequestfromMsg(const mission_sequencer::MissionRequest& msg, std::string& request_str);

    /**
     * @brief Get Action from string
     * @param const reference to Action
     * @param const reference to watchdog_msgs::Status
     * @param reference to watchdog_msgs::Action
     */
    void setActionMsg(const Action& action, const watchdog_msgs::Status& status_msg, watchdog_msgs::Action& action_msg);

    /**
     * @brief Call/Perform a state transition
     * @param std::string (next state string)
     */
    void stateTransition(std::string str);

    /**
     * @brief Polling with ros spin
     * @param sleeap time for polling in ms (int)
     */
    inline void polling(int ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      ros::spinOnce();
    }

    /**
     * @brief Watchdog (system status) heartbeat callback, reset timer
     */
    inline void watchdogHeartBeatCallback(const watchdog_msgs::StatusStampedConstPtr&) {
      watchdog_timer_->resetTimer();
    }

    /**
     * @brief Callback method called when a watchdog timer overflow occurs, print infos and unsubscribe
     */
    inline void watchdogTimerOverflowHandler() {
      std::cout << BOLD(YELLOW(" >>> Timeout overflow -- no heartbeat from system watchdog.\n")) << std::endl;
      sub_watchdog_heartbeat_.shutdown();
      sub_watchdog_status_.shutdown();
      stateTransition("land");
    }

    /**
     * @brief Callback method called when a flight timer overflow occurs, triggers a land
     */
    inline void flightTimerOverflowHandler() {
      std::cout << BOLD(YELLOW(" >>> Timeout overflow -- maximum flight time achieved -- the platform will land.\n")) << std::endl;
      stateTransition("land");
    }

    /**
     * @brief Callback method called when a failure timer overflow occurs, triggers a land
     */
    inline void failureTimerOverflowHandler() {
      std::cout << BOLD(YELLOW(" >>> Timeout overflow -- maximum waiting time for a sensor fix achieved -- the platform will land.\n")) << std::endl;
      stateTransition("land");
    }

    /// Nodehandler
    ros::NodeHandle nh_;

    /// Subscribers
    ros::Subscriber sub_watchdog_heartbeat_;
    ros::Subscriber sub_watchdog_status_;
    ros::Subscriber sub_landing_detection_;
    ros::Subscriber sub_mission_sequencer_responce_;

    /// Publishers
    ros::Publisher pub_watchdog_action_;
    ros::Publisher pub_mission_sequencer_request_;
    ros::Publisher pub_mission_sequencer_waypoints_;

    /// Service clients
    ros::ServiceClient takeoff_service_client_;
    ros::ServiceClient watchdog_start_service_client_;
    ros::ServiceClient data_recording_service_client_;
    ros::ServiceClient estimator_supervisor_service_client_;
    ros::ServiceClient estimator_init_service_client_;
    std::vector<ros::ServiceClient> inflight_sensor_init_service_client_;

    /// Timeout timers
    std::unique_ptr<Timer> watchdog_timer_;
    std::unique_ptr<Timer> flight_timer_;

    /// Loaded missions, mapped by their id
    std::map<int, Mission> missions_;

    /// Pending failures (sensor status and relative timers) reported by the watchdog
    std::vector<std::pair<SensorStatus, std::unique_ptr<Timer>>> pending_failures_;

    /// Waypoint parser
    std::unique_ptr<WaypointsParser> waypoints_parser_;

    /// Loaded waypoints
    std::vector<WaypointsParser::Waypoint> waypoints_;

    /// Selected mission ID (>= 1)
    int mission_id_ = -1;

    /// Pointer to State
    State* state_;

    /// Boolean to check if data is getting recorded
    bool is_recording_ = false;

    /// Boolean to check if the platform is armed
    bool armed_ = false;

    /// Boolean to check if we are in flight
    bool in_flight_ = false;

    /// Boolean to check if we are currently in takeoff
    bool in_takeoff_ = false;

    /// boolean to check if we are holding
    bool holding_ = false;

    /// boolean to check if we are hovering
    bool hovering_ = false;

    /// Boolean to determine weather a mission with multiple touchdown is loaded
    bool multiple_touchdowns_ = false;

    /// filepaths counter, used in case of multiple touchdown to keep track of how many filepaths have been loaded
    int filepaths_cnt_ = 0;

    /// Boolean to check if succesfully completed a mission
    bool last_waypoint_reached_= false;

    /// Boolean to check if we expect a land
    bool land_expected_ = false;

    /// Boolean to determine weather there is a pending request to mission sequencer
    bool ms_request_pending_ = false;

    /// Friend classes (able to access private data members)
    friend class Failure;
    friend class Hold;
    friend class Hover;
    friend class Initialization;
    friend class Land;
    friend class Nominal;
    friend class Undefined;
    friend class Preflight;
    friend class StartMission;
    friend class PerformMission;
    friend class EndMission;
    friend class Termination;

  }; // class Autonomy

} // namespace autonomy

#endif  // AUTONOMY_H
