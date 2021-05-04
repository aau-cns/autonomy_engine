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
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include <amaze_autonomy/autonomyConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <string>
#include <xmlrpcpp/XmlRpcValue.h>

#include "autonomy_core/autonomy_options.h"
#include "timer/timer.h"
#include "state_machine/state.h"

class AmazeAutonomy {

  public:

    /**
     * @brief Autonomy constructor
     * @param Ros NodeHandle
     */
    AmazeAutonomy(ros::NodeHandle &nh);

    /**
     * @brief Watchdog start service call
     */
    void startWatchdog();

    /**
     * @brief Function that defines the interface with the user.
     */
    void userInterface();

  private:

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
    void watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStamped& msg);

    /**
     * @brief Callback method called when a watchdog timer overflow occurs
     */
    void watchdogTimerOverflowHandler();

    /**
     * @brief Configuration callback for dynamic reconfigure
     */
    void configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level);

    /**
     * @brief Run preflight checks
     * @return boolean
     */
    [[nodiscard]] bool preFlightChecks();

    /**
     * @brief Get Entity-Action from XmlRpc::XmlRpcValue
     * @param reference to XmlRpc::XmlRpcValue
     * @param reference to Entity
     * @param reference to NextState (Action)
     * @return boolean
     */
    [[nodiscard]] bool getEntityAction(const XmlRpc::XmlRpcValue& entity_action, Entity& entity, NextState& action);

    /// Nodehandler
    ros::NodeHandle nh_;

    /// Autonomy options
    std::shared_ptr<autonomyOptions> opts_;

    /// Dynamic reconfigure server and callback
    dynamic_reconfigure::Server<amaze_autonomy::autonomyConfig> reconfigure_srv_;
    dynamic_reconfigure::Server<amaze_autonomy::autonomyConfig>::CallbackType reconfigure_cb_;

    /// Subscribers
    ros::Subscriber sub_watchdog_heartbeat_;
    ros::Subscriber sub_watchdog_status_;

    /// Publishers

    /// Watchdog service
    ros::ServiceClient service_client_;
    watchdog_msgs::Start service_;

    /// Timeout timer
    std::shared_ptr<Timer> timer_;

    /// Selected mission ID
    size_t mission_id_ = 0;

    /// State
    State state_();

};

#endif  // AMAZEAUTONOMY_H
