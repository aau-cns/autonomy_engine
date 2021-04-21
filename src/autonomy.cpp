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

#include "autonomy.h"
#include "colors.h"

#include <ros/ros.h>
#include <iostream>
#include <string>

AmazeAutonomy::AmazeAutonomy(ros::NodeHandle nh, boost::asio::io_service &io, int timeout_ms) :
  nh_(nh), reconfigure_cb_(boost::bind(&AmazeAutonomy::configCallback, this, _1, _2))
{
  // Reconfigure
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Service Server
  safety_srv = nh_.advertiseService("/safety_srv_in", &AmazeAutonomy::SafetyNodeCallback, this);

  // Subscriber to watchdog (system status) heartbeat
  sub_safety_node_heartbeat_ = nh_.subscribe("/watchdog/status", 10, &AmazeAutonomy::WatchdogHeartBeatCallback, this);

  // Instanciate timeout timer
  std::make_shared<Timer>(io, timeout_ms);
}

bool AmazeAutonomy::SafetyNodeCallback(ros_watchdog::wderror::Request& request, ros_watchdog::wderror::Response& response)
{
  std::cout << "Got Service Request with some Error Status" << std::endl;
  return true;
}

void AmazeAutonomy::WatchdogHeartBeatCallback(const autonomy_msgs::SystemStatusConstPtr& meas)
{
  // Restart timeout timer
  try {

  } catch (const std::exception&) {
    std::cout << BOLD(RED("Timeout overflow -- no heartbeat from system watchdog")) << std::endl;

    // Take action here ...

  }
}

void AmazeAutonomy::configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level)
{
  if (config.option_a)
  {
    std::cout << "Option A was choosen in the Reconfigure GUI" << std::endl;
    config.option_a = false;
  }
}
