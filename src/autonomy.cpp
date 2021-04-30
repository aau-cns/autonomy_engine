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
  if(!parseRosParams()) {
      throw std::exception();
  }

  // Print option
  opts_->printAutonomyOptions();

  // Set dynamic reconfigure callback
  reconfigure_srv_.setCallback(reconfigure_cb_);

  // Advertise watchdog service
  service_client_ = nh_.serviceClient<watchdog_msgs::Start>("/watchdog/service/start");

  // Subscriber to watchdog (system status) heartbeat
  sub_watchdog_heartbeat_ = nh_.subscribe("/watchdog/status", 1, &AmazeAutonomy::watchdogHeartBeatCallback, this);

  // Subscribe to watchdog status changes
  sub_watchdog_status_ = nh.subscribe("/watchdog/log", 1, &AmazeAutonomy::watchdogStatusCallback, this);

  // Instanciate timeout timer and connect signal
  timer_ = std::make_shared<Timer>(opts_->timeout);
  timer_->sh_.connect(boost::bind(&AmazeAutonomy::watchdogTimerOverflowHandler, this));
}

bool AmazeAutonomy::parseRosParams() {

  // Define auxilliary variables and default values
  int watchdog_timeout_ms = 100;
  int watchdog_startup_time_s = 5;
  int n_missions = 0;
  std::string description;
  std::map<int, std::string> missions;
  std::vector<std::string> critical_entities;
  std::map<int, std::pair<Entity, Action>> entity_action;

  // Get watchdog timer timeout
  nh_.param<int>("watchdog_timeout_ms", watchdog_timeout_ms, watchdog_timeout_ms);

  // Get watchdog startup time
  nh_.param<int>("watchdog_startup_time_s", watchdog_startup_time_s, watchdog_startup_time_s);

  // Get missions information
  nh_.param<int>("missions/number", n_missions, n_missions);
  if (n_missions == 0) {
    std::cout << std::endl << BOLD(RED("No missions defined")) << std::endl;
    return false;
  } else {
    for (int i = 1; i <= n_missions; ++i) {
      if(!nh_.getParam("missions/mission_" + std::to_string(i) + "/description", description) && !nh_.getParam("missions/mission_" + std::to_string(i) + "/critical_entities", critical_entities)) {
        std::cout << std::endl << BOLD(RED("Mission number " + std::to_string(i) + " is not correctly defined")) << std::endl;
        return false;
      }

      // switch to check entities and set it
      // entity_action.insert({i, std::make_pair(critical_entities)});

      missions.insert({i, description});
    }
  }

  // Make options
  opts_ = std::make_shared<autonomyOptions>(autonomyOptions({watchdog_timeout_ms, watchdog_startup_time_s, missions}));

  // Success
  return true;
}

void AmazeAutonomy::startWatchdog() {

  // Define service request
  service_.request.header.stamp = ros::Time::now();
  service_.request.startup_time = opts_->watchdog_startup_time;

  // Call service request
  if (service_client_.call(service_)) {

    // Check responce
    if(service_.response.successful) {

      std::cout << std::endl << BOLD(GREEN("--------- WATCHDOG IS RUNNING ---------")) << std::endl << std::endl;

//      // Initialize entities
//      for (const auto &it : service_.response.entities) {
//        entities_.emplace_back(std::make_pair(it.name, it.id));
//        std::cout << std::endl << BOLD(GREEN(" - Entity: " + it.name + " | ID: " + std::to_string(it.id))) << std::endl;
//      }

      // System status --> nominal

      std::cout << std::endl << BOLD(GREEN("---------------------------------------")) << std::endl;

    } else {
       std::cout << std::endl << BOLD(RED("------ FAILED TO START WATCHDOG -------")) << std::endl << std::endl;
       std::cout << BOLD(RED(" Please perform a system hard restart  ")) << std::endl;
       std::cout << BOLD(RED(" If you get the same problem after the ")) << std::endl;
       std::cout << BOLD(RED(" hard restart, shutdown the system and ")) << std::endl;
       std::cout << BOLD(RED(" abort the mission. ")) << std::endl << std::endl;
       std::cout << BOLD(RED("---------------------------------------")) << std::endl;

       // Eventually include debug information here

       throw std::exception();
    }
  } else {
    std::cout << std::endl << BOLD(RED("------- FAILED TO CALL SERVICE --------")) << std::endl << std::endl;
    std::cout << BOLD(RED(" Please perform a system hard restart  ")) << std::endl;
    std::cout << BOLD(RED(" If you get the same problem after the ")) << std::endl;
    std::cout << BOLD(RED(" hard restart, shutdown the system and ")) << std::endl;
    std::cout << BOLD(RED(" abort the mission. ")) << std::endl;
    std::cout << BOLD(RED("---------------------------------------")) << std::endl;
    throw std::exception();
  }
}

void AmazeAutonomy::watchdogHeartBeatCallback(const watchdog_msgs::StatusStampedConstPtr& msg) {

  // Restart timeout timer
  timer_->resetTimer();
}

void AmazeAutonomy::watchdogStatusCallback(const watchdog_msgs::StatusChangesArrayStamped& msg) {

  // Parse the message
  msg.data.changes.end();

  // changes is the changes wrt the last change

}

void AmazeAutonomy::watchdogTimerOverflowHandler() {

  // print message of watchdog timer overflow
  std::cout << std::endl << BOLD(RED("Timeout overflow -- no heartbeat from system watchdog")) << std::endl;
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
    std::cout << BOLD(GREEN(" - ID: ")) << it.first << BOLD(GREEN(" DESCRIPTION: ")) << it.second << std::endl;
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

bool AmazeAutonomy::preFlightChecks() {

  // service call to check if we are ready to takeoff

  return true;
}
