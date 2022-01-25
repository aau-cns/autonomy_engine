// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
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

#ifndef AUTONOMY_DEFS_H
#define AUTONOMY_DEFS_H

#include <string>
#include <vector>

namespace autonomy {

  /**
   * @brief Entity
   */
  enum Entity {PX4_GPS = 0, PX4_IMU = 1, PX4_MAG = 2, PX4_BAR = 3, MISSION_CAM = 4, REALSENSE = 5, LSM9DS1 = 6, LRF = 7, RTK_GPS_1 = 8, RTK_GPS_2 = 9, UWB = 10, OPTITRACK = 11};

  /**
   * @brief Event regarding an entity.
   */
  enum Event {ENTITY_FAILURE = 0, ENTITY_FIX = 1, ENTITY_OTHER = 2};

  /**
   * @brief Type that trigger an event of failure/fix
   */
  enum Type {GLOBAL = 0, TOPIC = 1, NODE = 2, DRIVER = 3};

  /**
   * @brief Action to be performed by the watchdog to react to an entity event
   */
  enum Action {NOTHING = 0, FIX_NODE = 1, FIX_DRIVER = 2};

  /**
   * @brief Single sensor status
   */
  struct SensorStatus {

    /// Timestamp of status report
    double timestamp;

    /// Entity
    Entity entity;

    /// Event and reported msg.status
    Event event;

    /// Type
    Type type;

    /// Action
    Action action;

    /// Debug name and infos
    std::string debug_name;
    std::string debug_info;

    /// Sort function to allow for using of STL containers
    bool operator<(const SensorStatus& other) const {
      return timestamp < other.timestamp;
    }

    /// Comparison operator overloading
    bool operator==(const SensorStatus& ss) const {
      if (timestamp == ss.timestamp && entity == ss.entity && event == ss.event && type == ss.type) {
        return true;
      }
      return false;
    }

    /// Equal operator, that checks for everything except timestamps
    bool isEqual(const SensorStatus& ss) const {
      if (entity == ss.entity && type == ss.type) {
        return true;
      }
      return false;
    }
  };

  /**
   * @brief Get Entity from String
   * @param const reference to string
   * @param reference to Entity
   */
  [[nodiscard]] inline bool getEntityFromString(const std::string& str, Entity& entity) {

    if (str.compare("px4_imu") == 0) {
      entity = Entity::PX4_IMU;
    } else if (str.compare("px4_gps") == 0) {
      entity = Entity::PX4_GPS;
    } else if (str.compare("px4_bar") == 0) {
      entity = Entity::PX4_BAR;
    } else if (str.compare("px4_mag") == 0) {
      entity = Entity::PX4_MAG;
    } else if (str.compare("mission_cam") == 0) {
      entity = Entity::MISSION_CAM;
    } else if (str.compare("realsense") == 0) {
      entity = Entity::REALSENSE;
    } else if (str.compare("lsm9ds1") == 0) {
      entity = Entity::LSM9DS1;
    } else if (str.compare("lrf") == 0) {
      entity = Entity::LRF;
    } else if (str.compare("uwb") == 0) {
      entity = Entity::UWB;
    } else if (str.compare("rtk_gps_1") == 0) {
      entity = Entity::RTK_GPS_1;
    } else if (str.compare("rtk_gps_2") == 0) {
      entity = Entity::RTK_GPS_2;
    } else if (str.compare("optitrack") == 0) {
      entity = Entity::OPTITRACK;
    } else {
      return false;
    }
    return true;
  }

  /**
   * @brief Get String from Entity
   * @param const reference to Entity
   * @param reference to String
   * @return boolean
   */
  [[nodiscard]] inline bool getStringFromEntity(const Entity& entity, std::string& str) {

    switch (entity) {
    case Entity::PX4_IMU:
      str = "px4_imu";
      break;
    case Entity::PX4_GPS:
      str = "px4_gps";
      break;
    case Entity::PX4_BAR:
      str = "px4_bar";
      break;
    case Entity::PX4_MAG:
      str = "px4_mag";
      break;
    case Entity::MISSION_CAM:
      str = "mission_cam";
      break;
    case Entity::REALSENSE:
      str = "realsense";
      break;
    case Entity::LSM9DS1:
      str = "lsm9ds1";
      break;
    case Entity::LRF:
      str = "lrf";
      break;
    case Entity::UWB:
      str = "uwb";
      break;
    case Entity::RTK_GPS_1:
      str = "rtk_gps_1";
      break;
    case Entity::RTK_GPS_2:
      str = "rtk_gps_2";
      break;
    case Entity::OPTITRACK:
      str = "optitrack";
      break;
    default:
      return false;
    }
    return true;
  }

  /**
   * @brief Get String from Entity
   * @param const reference to Entity
   * @param reference to String
   * @return boolean
   */
  [[nodiscard]] inline bool getStringFromType(const Type& type, std::string& str) {

    switch (type) {
    case Type::GLOBAL:
      str = "Global";
      break;
    case Type::NODE:
      str = "Node";
      break;
    case Type::DRIVER:
      str = "Driver";
      break;
    case Type::TOPIC:
      str = "Topic";
      break;
    default:
      return false;
    }
    return true;
  }

  /**
   * @brief Check the existance of a state from String
   * @param const reference to String
   * @return Bool
   */
  [[nodiscard]] inline bool checkStateFromString(const std::string& str) {

    if (str.compare("continue") == 0 ||
        str.compare("hold") == 0     ||
        str.compare("failure") == 0  ||
        str.compare("land") == 0) {
      return true;
    } else {
      return false;
    }
  }

} // namespace autonomy

#endif  // AUTONOMY_DEFS_H
