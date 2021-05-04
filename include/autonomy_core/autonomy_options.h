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

#ifndef AUTONOMY_OPTIONS_H
#define AUTONOMY_OPTIONS_H

#include <iostream>
#include <vector>
#include <map>

#include "utils/colors.h"
#include "utils/utilities.h"
#include "autonomy_core/entity_event.h"

/**
 * @brief Struct for autonomy options.
 *
 * This struct contains information mainly got from the
 * parameter server that are used by the autonomy.
 * Matricies are defined as vectors in Row-Major order
 */
struct autonomyOptions {

  /// Timeout in milliseconds for watchdog heartbeat
  const int timeout;

  /// Watchdog startup time in seconds needed to check entities
  const int watchdog_startup_time;

  /// Mission map <Mission ID, Mission description>
  const std::map<int, std::string> missions;

  /// Entity Action map <Mission ID <Entity, Action>>
  const std::vector<std::pair<int, std::pair<Entity, NextState>>> entity_action_vector;

  /// Print function
  void printAutonomyOptions() {

    std::cout << std::endl << BOLD(YELLOW("---------- LOADED PARAMETERS ----------")) << std::endl << std::endl;
    std::cout << BOLD(YELLOW(" - Watchdog heartbeat timeout: " + std::to_string(timeout) + " ms")) << std::endl;
    std::cout << BOLD(YELLOW(" - Watchdog startup time: " + std::to_string(watchdog_startup_time) + " s")) << std::endl;
    std::cout << BOLD(YELLOW(" - Number of missions: " + std::to_string(missions.size()) + "")) << std::endl;

//    std::cout << << std::endl << BOLD(YELLOW(" - Loaded missions: ")) << std::endl;
//    for (const auto &it : missions) {
//      std::cout << BOLD(YELLOW("   - Mission ID: " + std::to_string(it.first) + " | Mission Description: " + it.second + "")) << std::endl;
//    }

//    std::cout << BOLD(YELLOW(" - Entity-Action Transtion Table: ")) << std::endl;
//    for (const auto &it : entity_action_vector) {

//      std::string entity, action;

//      switch (it.second.first) {
//      case Entity::PX4_GPS:
//        entity = "PX4 GPS";
//        break;
//      case Entity::PX4_BAR:
//        entity = "PX4 BAROMETER";
//        break;
//      case Entity::PX4_MAG:
//        entity = "PX4 MAGNETOMETER";
//        break;
//      case Entity::MISSION_CAM:
//        entity = "MISSION CAMERA";
//        break;
//      case Entity::REALSENSE:
//        entity = "REALSENSE";
//        break;
//      case Entity::LSM9DS1:
//        entity = "LSM9DS1 IMU";
//        break;
//      case Entity::LRF:
//        entity = "LASER RANGE FINDER";
//        break;
//      case Entity::RTK_GPS_1:
//        entity = "RTK GPS 1";
//        break;
//      case Entity::RTK_GPS_2:
//        entity = "RTK GPS 2";
//        break;
//      default:
//        break;
//      }

//      switch (it.second.second) {
//      case NextState::NOMINAL:
//        action = "CONTINUE";
//        break;
//      case NextState::HOLD:
//        action = "HOLD";
//        break;
//      case NextState::MANUAL:
//        action = "MANUAL";
//        break;
//      }

//      std::cout << BOLD(YELLOW("   - Mission ID: " + std::to_string(it.first) + " | " + entity + " => " + action + "")) << std::endl;
//    }

    std::cout << std::endl << BOLD(YELLOW("---------------------------------------")) << std::endl;
  }

};

#endif  // AUTONOMY_OPTIONS_H
