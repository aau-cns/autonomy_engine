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

#include <ros/ros.h>
#include "colors.h"

/**
 * @brief Struct for autonomy options
 */
struct autonomyOptions
{

  /// Timeout in milliseconds for watchdog heartbeat (100ms default)
  int timeout;

  /// Window of time in seconds for sensor readings during pre-flight checks (1s default)
  int sensor_readings_window = 1;

  /// pitch-roll angle threshold in degree (5.0deg default)
  float angle_threshold = 5.0;

  /// Print function
  void printAutonomyOptions()
  {
    std::cout << YELLOW("Watchdog heartbeat timeout: ") << timeout << YELLOW("ms") << std::endl;
    std::cout << YELLOW("Sensor reading time window during pre-flight checks: ") << sensor_readings_window << YELLOW("s") << std::endl;
    std::cout << YELLOW("Pitch-Roll angle threshold: ") << angle_threshold << YELLOW("deg") << std::endl;
  }

};

#endif  // AUTONOMY_OPTIONS_H
