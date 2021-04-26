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

#ifndef MATHEMATICS_H
#define MATHEMATICS_H

#include <vector>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <math.h>

/**
 * @brief Skew-symmetric matrix from a given 3x1 vector
 * @param 3x1 vector
 * @return 3x3 skew-symmetric matrix
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d &w) {

  Eigen::Matrix3d skew;

  skew << 0, -w(2), w(1),
          w(2), 0, -w(0),
          -w(1), w(0), 0;

  return skew;
}

/**
 * @brief Rotation matrix from a given 9x1 vector in row-major order
 * @param 9x1 vector in row-major order
 * @return 3x3 Rotation matrix
 */
inline Eigen::Matrix3d Rot(const std::vector<double> &x) {

  Eigen::Matrix3d R;

  R << x.at(0),x.at(1),x.at(2),
       x.at(3),x.at(4),x.at(5),
       x.at(6),x.at(7),x.at(8);

  return R;
}

/**
 * @brief Conversion from radians to degrees.
 */
inline double rad2deg(const double rad) {
  return rad * 180 / M_PI;
}

/**
 * @brief Rotation matrix to euler angles conversion.
 *
 * roll = rotation about x
 * pitch = rotation about y
 * yaw = rotation abot z
 *
 * @param 3x3 Rotation matrix
 * @return 3x1 vector of angles in degree
 */
inline Eigen::Vector3d eul(const Eigen::Matrix3d &R) {
  double eps = 1.0e-3;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  Eigen::Vector3d eul;

  if ((abs(R(2,0)) - 1) < eps) {
    pitch = -asin(R(2,0));
    roll = atan2(R(2,1)/cos(roll), R(2,2)/cos(roll));
    yaw = atan2(R(1,0)/cos(roll), R(0,0)/cos(roll));
  } else {
    yaw = 0.0;
    if(abs(R(2,0) + 1) < eps) {
      pitch = M_PI/2;
      roll = atan2(R(0,1), R(0,2));
    } else {
      pitch = -M_PI/2;
      roll = atan2(-R(0,1), -R(0,2));
    }
  }
  eul << rad2deg(roll), rad2deg(pitch), rad2deg(yaw);

  return eul;
}

#endif  // MATHEMATICS_H
