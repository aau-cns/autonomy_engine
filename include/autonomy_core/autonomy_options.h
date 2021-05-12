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
 * @brief Immutable Struct for autonomy options.
 *
 * This struct contains information mainly got from the
 * parameter server that are used by the autonomy.
 */
struct autonomyOptions {

  /// Timeout in milliseconds for watchdog heartbeat
  const int timeout;

  /// Watchdog startup time in seconds needed to check entities
  const int watchdog_startup_time;

  /// Mission map <Mission ID, Mission description>
  const std::map<size_t, std::string> missions;

  /// Entity Action map <Mission ID <Entity, Action>>
  const std::vector<std::pair<size_t, std::pair<Entity, NextState>>> entity_action_vector;

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
