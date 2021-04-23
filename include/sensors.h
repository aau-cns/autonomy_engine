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

#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <Eigen/Eigen>

/**
 * @brief Struct for imu data
 */
struct imuData
{

    /// Timestamp of the reading (s)
    double timestamp;

    /// Gyroscope reading, angular velocity (rad/s)
    Eigen::Matrix<double, 3, 1> wm;

    /// Accelerometer reading, linear acceleration (m/s^2)
    Eigen::Matrix<double, 3, 1> am;

    /// Sort function to allow for using of STL containers
    bool operator<(const imuData& other) const {
        return timestamp < other.timestamp;
    }

};

#endif  // SENSORS_H
