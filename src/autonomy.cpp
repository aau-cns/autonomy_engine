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
#include <std_srvs/Trigger.h>
#include <iostream>
#include <string>

AmazeAutonomy::AmazeAutonomy(ros::NodeHandle nh)
  : reconfigure_cb_(boost::bind(&AmazeAutonomy::configCallback, this, _1, _2))
{
  // Reconfigure
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Services
  safety_srv = nh.advertiseService("safety_node_srv", &AmazeAutonomy::SafetyNodeCallback, this);

  // Subscriber
  sub_safety_node_heartbeat_ =
      nh.subscribe("safety_node_heartbeat_in", 10, &AmazeAutonomy::SafetyNodeHeartBeatCallback, this);

  // Publisher
  // pub_some_topic_ = nh.advertise<std_msgs::Empty>("autonomy_out", 5);
}

bool AmazeAutonomy::SafetyNodeCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  std::cout << "Got Service Request" << std::endl;
  return true;
}

void AmazeAutonomy::SafetyNodeHeartBeatCallback(const std_msgs::EmptyConstPtr& meas)
{
  std::cout << "Got Safety Node Heartbeat" << std::endl;
}

void AmazeAutonomy::configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level)
{
  if (config.option_a)
  {
    std::cout << "Option A was choosen in the Reconfigure GUI" << std::endl;
  }
}
