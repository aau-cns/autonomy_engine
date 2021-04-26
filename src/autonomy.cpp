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

AmazeAutonomy::AmazeAutonomy(ros::NodeHandle &nh) :
  nh_(nh), reconfigure_cb_(boost::bind(&AmazeAutonomy::configCallback, this, _1, _2)) {

  // Parse options
  if(!parseRosParams())
  {
      throw std::exception();
  }

  // Print option
  opts_->printAutonomyOptions();

  // Set dynamic reconfigure callback
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Advertise watchdog service
  safety_srv = nh_.advertiseService("/safety_srv_in", &AmazeAutonomy::watchdogCallback, this);

  // Subscriber to watchdog (system status) heartbeat
  sub_watchdog_heartbeat_ = nh_.subscribe("/watchdog/status", 10, &AmazeAutonomy::watchdogHeartBeatCallback, this);

  // Instanciate timeout timer and connect signal
  timer_ = std::make_shared<Timer>(opts_->timeout);
  timer_->sh_.connect(boost::bind(&AmazeAutonomy::watchdogTimerOverflowHandler, this));
}

bool AmazeAutonomy::parseRosParams() {

  // Define auxilliary variables and default values
  int watchdog_timeout_ms = 100;
  double sensor_readings_window_s = 1.0;
  double angle_threshold_deg = 10.0;
  int n_missions = 0;
  std::vector<double> R_IP = {1,0,0,0,1,0,0,0,1};
  std::string imu_topic, description, filepath;
  std::map<int, std::pair<std::string, std::string>> missions;

  // Get watchdog timer timeout
  nh_.param<int>("watchdog_timeout_ms", watchdog_timeout_ms, watchdog_timeout_ms);

  // Get sensor reading window
  nh_.param<double>("sensor_readings_window", sensor_readings_window_s, sensor_readings_window_s);

  // Get roll and pitch angle threshold for platform flatness check
  nh_.param<double>("angle_threshold", angle_threshold_deg, angle_threshold_deg);

  // Get IMU-Platform rotation
  nh_.param<std::vector<double>>("R_IP", R_IP, R_IP);

  // Get imu topic
  if(!nh_.getParam("imu_topic", imu_topic)) {
    std::cout << std::endl << BOLD(RED("No IMU topic defined")) << std::endl;
    return false;
  }

  // Get missions information
  nh_.param<int>("missions/number", n_missions, n_missions);
  if (n_missions == 0) {
    std::cout << std::endl << BOLD(RED("No missions defined")) << std::endl;
    return false;
  } else {
    for (int i = 1; i <= n_missions; ++i) {
      if(!nh_.getParam("missions/mission_" + std::to_string(i) + "/description", description) && !nh_.getParam("missions/mission_" + std::to_string(i) + "/filepath", filepath)) {
        std::cout << std::endl << BOLD(RED("Mission number " + std::to_string(i) + " is not correctly defined")) << std::endl;
        return false;
      }
      missions.insert({i,std::make_pair(description,filepath)});
    }
  }

  // Make options
  opts_ = std::make_shared<autonomyOptions>(autonomyOptions({watchdog_timeout_ms, imu_topic, sensor_readings_window_s, angle_threshold_deg, R_IP, missions}));

  // Success
  return true;
}

bool AmazeAutonomy::watchdogCallback(ros_watchdog::wderror::Request& request, ros_watchdog::wderror::Response& response) {
  std::cout << std::endl << BOLD(RED("Got Service Request with some Error Status")) << std::endl;
  return true;
}

void AmazeAutonomy::watchdogHeartBeatCallback(const autonomy_msgs::SystemStatusConstPtr& meas) {

  // Restart timeout timer
  timer_->resetTimer();

  // Do something here...

}

void AmazeAutonomy::watchdogTimerOverflowHandler() {

  // print message of watchdog timer overflow
  std::cout << std::endl << BOLD(RED("Timeout overflow -- no heartbeat from system watchdog")) << std::endl;

  // Do something here ...

}

void AmazeAutonomy::configCallback(amaze_autonomy::autonomyConfig& config, uint32_t level) {
  if (config.option_a) {
    std::cout << "Option A was choosen in the Reconfigure GUI" << std::endl;
    config.option_a = false;
  }
}

void AmazeAutonomy::userInterface() {

  // Print missions
  std::cout << std::endl << BOLD(GREEN("Please select one of the following mission by inputting the mission ID")) << std::endl << std::endl;
  for (auto &it : opts_->missions) {
    std::cout << BOLD(GREEN(" - ID: ")) << it.first << BOLD(GREEN(" DESCRIPTION: ")) << it.second.first << std::endl;
  }

  // Get ID of mission being executed
  std::cout << std::endl << BOLD(GREEN(">>> "));
  std::cin >> mission_id_;

  // Check validity of mission id
  if (mission_id_ == 0 || mission_id_ > opts_->missions.size()) {
    std::cout << std::endl << BOLD(RED("Wrong mission ID chosen")) << std::endl;
    throw std::exception();
  } else {
    std::cout << std::endl << BOLD(GREEN(" - Selected mission with ID: ")) << mission_id_ << std::endl;
  }

  // check result of preflight checks
  std::cout << std::endl << BOLD(GREEN("Start Pre-Flight Checks ...")) << std::endl;
  if (!AmazeAutonomy::preFlightChecks()) {
    std::cout << std::endl << BOLD(RED("Pre-Flight checks failure")) << std::endl;
    throw std::exception();
  }

}

void AmazeAutonomy::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {

  // Parse incoming message and fill out specified data structure
  imuData meas;
  meas.timestamp = msg->header.stamp.toSec();
  meas.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  meas.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

  // Push measurement into buffer
  imu_data_buffer_.emplace_back(meas);

  // Remove oldest if sensor reading window width is reached
  if ((meas.timestamp - imu_data_buffer_.begin()->timestamp) > opts_->sensor_readings_window) {

    // Since we check everytime a new measurement is added to the buffer it would
    // be sufficient to simply remove the first element, however it is more robust
    // to check everytime how many elements we should remove.

    double Dt = meas.timestamp - opts_->sensor_readings_window;

    // Get iterator to first element that has to be kept (timestamp > Dt (meas.timestamp - timestamp < window))
    auto it = std::find_if(imu_data_buffer_.begin(), imu_data_buffer_.end(), [&Dt](imuData meas){return meas.timestamp >= Dt;});

    // Remove all 1elements starting from the beginning until the first element that has to be kept (excluded)
    imu_data_buffer_.erase(imu_data_buffer_.begin(), it-1);
  }

}

bool AmazeAutonomy::checkFlatness() {

  // Return if buffer is empty
  if (imu_data_buffer_.empty()) {
      std::cout << std::endl << BOLD(RED("No IMU measurements available to check platform flatness")) << std::endl;
      return false;
  }

  // Return if minimum window is not reached
  if ((imu_data_buffer_.end()->timestamp - imu_data_buffer_.begin()->timestamp) < opts_->sensor_readings_window) {
    std::cout << std::endl << BOLD(RED("Not enough IMU measurements available to check platform flatness")) << std::endl;
    return false;
  }

  // Define mean acceleration and mean angular velocity
  Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d ang_mean = Eigen::Vector3d::Zero();

  // Calculate the mean acceleration and the mean angular velocity
  for (auto &it : imu_data_buffer_) {
    acc_mean += it.am;
    ang_mean += it.wm;
  }
  acc_mean = acc_mean/imu_data_buffer_.size();
  ang_mean = ang_mean/imu_data_buffer_.size();

  // As further check we could eventually compute
  // the sample variance to check if there have
  // been to much excitation and return false

  // Get z axis aligned with gravity direction
  Eigen::Vector3d z_axis = acc_mean/acc_mean.norm();

  // Make x axis perpendicular to z axis
  Eigen::Vector3d e_1(1,0,0);
  Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
  x_axis= x_axis/x_axis.norm();

  // Get y axis from the cross product of these two
  Eigen::Vector3d y_axis = skew(z_axis)*x_axis;

  // Get rotation of the imu using axes as columns
  // R_GI defines the rotation matrix that rotates
  // vector in IMU frame (I_x) to vector in global
  // inertial gravity-aligned frame (G_x = R_GI * I_x)
  Eigen::Matrix3d R_GI;
  R_GI.block(0,0,3,1) = x_axis;
  R_GI.block(0,1,3,1) = y_axis;
  R_GI.block(0,2,3,1) = z_axis;

  // Apply Rotation between the imu and the platform
  Eigen::Matrix3d R_GP = R_GI*Rot(opts_->R_IP);

  // Convert Rotation matrix to euler angles
  Eigen::Vector3d eul_ang = eul(R_GP);

  // Compare roll and pitch with thresholds
  if (eul_ang(0) > opts_->angle_threshold || eul_ang(1) > opts_->angle_threshold) {
    std::cout << std::endl << BOLD(RED("Platform not flat! roll reading: ")) << eul_ang(0) << BOLD(RED(" pitch reading: ")) << eul_ang(1) << std::endl;
    return false;
  } else {
    std::cout << std::endl << BOLD(GREEN("Platform is flat! roll reading: ")) << eul_ang(0) << BOLD(GREEN(" pitch reading: ")) << eul_ang(1) << std::endl;
  }

  // test passed
  return true;
}

bool AmazeAutonomy::preFlightChecks() {

  // Subscribe to IMU
  sub_imu_ = nh_.subscribe(opts_->imu_topic, 999, &AmazeAutonomy::imuCallback, this);

  // Sleep (at least twice sensor reading window) to get imu measurements
  usleep(static_cast<__useconds_t>(opts_->sensor_readings_window*2e6));

  // Check flatness
  if(!checkFlatness()) {
    return false;
  }

   // Shutdown IMU subscriber
   sub_imu_.shutdown();

  return true;

}
