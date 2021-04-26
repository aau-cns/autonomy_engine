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
 * @brief Struct for autonomy options.
 *
 * This struct contains information mainly got from the
 * parameter server that are used by the autonomy.
 * Matricies are defined as vectors in Row-Major order
 */
struct autonomyOptions {

  /// Timeout in milliseconds for watchdog heartbeat (100ms default)
  int timeout = 100;

  /// IMU topic
  std::string imu_topic;

  /// Window of time in seconds for sensor readings during pre-flight checks (1s default)
  double sensor_readings_window = 1;

  /// pitch-roll angle threshold in degree (10.0deg default)
  double angle_threshold = 10.0;

  /// Rotation matrix that rotates vector in the platform frame (P_x) to vector
  /// in the IMU frame (I_x = R_IP * P_x) (Identity default)
  std::vector<double> R_IP = {1,0,0,0,1,0,0,0,1};

  /// Number of missions to be loaded
  int number_missions = 0;

  /// Mission map <Mission ID <Mission description, Mission filepath>>
  std::map<int, std::pair<std::string, std::string>> missions;

  /// Print function
  void printAutonomyOptions() {

    std::cout << std::endl << BOLD(YELLOW("---------- LOADED PARAMETERS ----------")) << std::endl << std::endl;
    std::cout << BOLD(YELLOW(" - Watchdog heartbeat timeout: ")) << timeout << "ms" << std::endl;
    std::cout << BOLD(YELLOW(" - IMU topic: ")) << imu_topic << std::endl;
    std::cout << BOLD(YELLOW(" - Sensor reading time window during pre-flight checks: ")) << sensor_readings_window << "s" << std::endl;
    std::cout << BOLD(YELLOW(" - Pitch-Roll angle threshold: ")) << angle_threshold << "deg" << std::endl;
    std::cout << BOLD(YELLOW(" - IMU-Platform Rotation R_IP: ")) << R_IP << std::endl;
    std::cout << BOLD(YELLOW(" - Number of missions: ")) << number_missions << std::endl;
    std::cout << std::endl << BOLD(YELLOW("---------------------------------------")) << std::endl;
  }

};

#endif  // AUTONOMY_OPTIONS_H
