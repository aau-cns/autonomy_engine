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

#include "colors.h"
#include "utilities.h"

/**
 * @brief Entities
 */
enum Entity {PX4_GPS, PX4_IMU, PX4_MAG, PX4_BAR, MISSION_CAM, REALSENSE, LSM9DS1, LRF, RTK_GPS_1, RTK_GPS_2};

/**
 * @brief Actions to be performed when a entity failure is identified
 */
enum Action {CONTINUE, HOLD, MANUAL};

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

  /// Mission map <Mission ID Mission description>
  const std::map<int, std::string> missions;

  /// Entities actions map <Mission ID <Entity, Action>>
  const std::map<int, std::pair<Entity, Action>> entity_action;

  /// Print function
  void printAutonomyOptions() {

    std::cout << std::endl << BOLD(YELLOW("---------- LOADED PARAMETERS ----------")) << std::endl << std::endl;
    std::cout << BOLD(YELLOW(" - Watchdog heartbeat timeout: " + std::to_string(timeout) + " ms")) << std::endl;
    std::cout << BOLD(YELLOW(" - Watchdog startup time: " + std::to_string(watchdog_startup_time) + " s")) << std::endl;
    std::cout << BOLD(YELLOW(" - Number of missions: " + std::to_string(missions.size()) + "")) << std::endl;
    std::cout << std::endl << BOLD(YELLOW("---------------------------------------")) << std::endl;
  }

};

#endif  // AUTONOMY_OPTIONS_H
