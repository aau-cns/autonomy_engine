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

AmazeAutonomy::AmazeAutonomy(ros::NodeHandle &nh, boost::asio::io_service &io) :
  nh_(nh), reconfigure_cb_(boost::bind(&AmazeAutonomy::configCallback, this, _1, _2))
{

  // Parse options
  parseRosParams(nh_, opts_);

  // Print options
  opts_.printAutonomyOptions();

  // Set dynamic reconfigure callback
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Advertise watchdog service
  safety_srv = nh_.advertiseService("/safety_srv_in", &AmazeAutonomy::WatchdogCallback, this);

  // Subscriber to watchdog (system status) heartbeat
  sub_safety_node_heartbeat_ = nh_.subscribe("/watchdog/status", 10, &AmazeAutonomy::WatchdogHeartBeatCallback, this);

  // Instanciate timeout timer
  timer_ = std::make_shared<Timer>(io, opts_.timeout);
}

void AmazeAutonomy::parseRosParams(ros::NodeHandle &nh, autonomyOptions &opts)
{
  // get params
  nh.param<int>("watchdog_timeout_ms", opts.timeout, opts.timeout);
  nh.param<int>("sensor_readings_window", opts.sensor_readings_window, opts.sensor_readings_window);
  nh.param<float>("angle_threshold", opts.angle_threshold, opts.angle_threshold);
}

bool AmazeAutonomy::WatchdogCallback(ros_watchdog::wderror::Request& request, ros_watchdog::wderror::Response& response)
{
  std::cout << "Got Service Request with some Error Status" << std::endl;
  return true;
}

void AmazeAutonomy::WatchdogHeartBeatCallback(const autonomy_msgs::SystemStatusConstPtr& meas)
{
  // Restart timeout timer
  try {
    timer_->restartTimer();
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


// The idea is to subscribe to imu when we check the "flatness" of the platform
// We will listen to imu measurement for a fixed (configurable) window of time
// and then we will compute the roll and the pitch of the IMU
// We will then compare the computed roll and pitch with pre-defined
// thresholds (configurable) ... work in progress
void AmazeAutonomy::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  // Parse incoming message and fill out specified data structure
  imuData meas;
  meas.timestamp = msg->header.stamp.toSec();
  meas.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  meas.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // Push measurement into buffer
  imu_data_buffer_.emplace_back(meas);
}
