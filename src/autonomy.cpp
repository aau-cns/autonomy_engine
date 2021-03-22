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

#include "autonomy.h"

#include <ros/ros.h>
#include <iostream>
#include <string>

AmazeAutonomy::AmazeAutonomy(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&AmazeAutonomy::configCallback, this, _1, _2))
{
  // Reconfigure
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Service Server
  safety_srv = nh.advertiseService("/safety_srv_in", &AmazeAutonomy::SafetyNodeCallback, this);

  // Service Clients
  safety_takeoff_ready = nh.serviceClient<ros_watchdog::wdstart>("/watchdog/start");

  // Subscriber
  sub_safety_node_heartbeat_ = nh.subscribe("/watchdog/status", 10, &AmazeAutonomy::SafetyNodeHeartBeatCallback, this);

  // Publisher
  // pub_some_topic_ = nh.advertise<std_msgs::Empty>("autonomy_out", 5);
}

bool AmazeAutonomy::SafetyNodeCallback(ros_watchdog::wderror::Request& request,
                                       ros_watchdog::wderror::Response& response)
{
  std::cout << "Got Service Request with some Error Status" << std::endl;
  return true;
}

void AmazeAutonomy::SafetyNodeHeartBeatCallback(const autonomy_msgs::SystemStatusConstPtr& meas)
{
  std::cout << "Got Safety Node Heartbeat" << std::endl;
}

void AmazeAutonomy::configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level)
{
  if (config.option_a)
  {
    // std::cout << "Option A was choosen in the Reconfigure GUI" << std::endl;

    ros_watchdog::wdstart srv;
    srv.request.source = "Autonomy";
    srv.request.startup_time = 5;  // Timeout in seconds

    if (safety_takeoff_ready.call(srv))
    {
      ROS_INFO("Succesfull %i", srv.response.successful);
    }
    else
    {
      ROS_ERROR("Failed to Call Safety Node with Takeoff Ready Service");
    }

    config.option_a = false;
  }
}
