// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, Universitaet Klagenfurt, Austria
// You can contact the author at <christian.brommer@ieee.org>
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

class AmazeAutonomy
{
public:
  AmazeAutonomy(ros::NodeHandle nh);

  // Dynamic reconfigure components
  dynamic_reconfigure::Server<amaze_autonomy::autonomyConfig> reconfigure_srv_;
  dynamic_reconfigure::Server<amaze_autonomy::autonomyConfig>::CallbackType reconfigure_cb_;
  void configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level);

  // Subscriber
  ros::Subscriber sub_safety_node_heartbeat_;

  // Service Server
  ros::ServiceServer safety_srv;
  bool SafetyNodeCallback(ros_watchdog::wderror::Request& request, ros_watchdog::wderror::Response& response);

  // Service Clients
  ros::ServiceClient safety_takeoff_ready;

  // Sensor Callbacks
  void SafetyNodeHeartBeatCallback(const autonomy_msgs::SystemStatusConstPtr& meas);

  // Publisher
  // ros::Publisher pub_some_topic_;
};

#endif  // AMAZEAUTONOMY_H
