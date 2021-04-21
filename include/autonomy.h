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

#include <autonomy_msgs/SystemStatus.h>
#include <ros/ros.h>
#include <ros_watchdog/wderror.h>
#include <ros_watchdog/wdstart.h>
#include <std_srvs/Trigger.h>
#include <amaze_autonomy/autonomyConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/bind/bind.hpp>

#include "timer.h"

class AmazeAutonomy
{

  public:

    /**
     * @brief Autonomy constructor
     * @param Ros NodeHandle
     * @param Reference to boost io service
     * @param timeout in milliseconds
     */
    AmazeAutonomy(ros::NodeHandle nh, boost::asio::io_service &io, int timeout_ms);

  private:

    /**
     * @brief Nodehandler
     */
    ros::NodeHandle nh_;

    /**
     * @brief Dynamic reconfigure server and callback
     */
    dynamic_reconfigure::Server<amaze_autonomy::autonomyConfig> reconfigure_srv_;
    dynamic_reconfigure::Server<amaze_autonomy::autonomyConfig>::CallbackType reconfigure_cb_;

    /**
     * @brief Subscribers
     */
    ros::Subscriber sub_safety_node_heartbeat_;

    /**
     * @brief Publisher
     */

    /**
     * @brief Services
     */
    ros::ServiceServer safety_srv;
    bool SafetyNodeCallback(ros_watchdog::wderror::Request& request, ros_watchdog::wderror::Response& response);

    /**
     * @brief Timeout timer
     */
    std::shared_ptr<Timer> timer_;

    /**
     * @brief Watchdog (system status) heartbeat callback
     */
    void WatchdogHeartBeatCallback(const autonomy_msgs::SystemStatusConstPtr& meas);

    /**
     * @brief Configuration callback for dynamic reconfigure
     */
    void configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level);

};

#endif  // AMAZEAUTONOMY_H
