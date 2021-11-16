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
  enum Entity {PX4_GPS, PX4_IMU, PX4_MAG, PX4_BAR, MISSION_CAM, REALSENSE, LSM9DS1, LRF, RTK_GPS_1, RTK_GPS_2};

  /**
   * @brief Event regarding an entity.
   */
  enum Event {ENTITY_FAILURE, ENTITY_FIX, ENTITY_OTHER};

  /**
   * @brief Type that trigger an event of failure/fix
   */
  enum Type {GLOBAL, TOPIC, NODE, DRIVER};

  /**
   * @brief Atonomy state
   */
  enum AutonomyState {UNDEFINED, INITIALIZATION, NOMINAL, HOLD, FAILURE, LAND, HOVER, TAKEOFF, FLIGHT};

  /**
   * @brief Action to be performed by the watchdog to react to an entity event
   */
  enum Action {NOTHING, FIX_NODE, FIX_DRIVER};

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
  };

  /**
   * @brief Get Entity from String
   * @param const reference to string
   * @param reference to Entity
   */
  [[nodiscard]] inline bool getEntityFromString(const std::string& str, Entity& entity) {

    // Check if it is either 0 or 1, this means that either the string
    // match or there is a char more that could be \n
    if ((str.compare("px4_imu") >> 1) == 0) {
      entity = Entity::PX4_IMU;
    } else if ((str.compare("px4_gps") >> 1) == 0) {
      entity = Entity::PX4_GPS;
    } else if ((str.compare("px4_bar") >> 1) == 0) {
      entity = Entity::PX4_BAR;
    } else if ((str.compare("px4_mag") >> 1) == 0) {
      entity = Entity::PX4_MAG;
    } else if ((str.compare("mission_cam") >> 1) == 0) {
      entity = Entity::MISSION_CAM;
    } else if ((str.compare("realsense") >> 1) == 0) {
      entity = Entity::REALSENSE;
    } else if ((str.compare("lsm9ds1") >> 1) == 0) {
      entity = Entity::LSM9DS1;
    } else if ((str.compare("lrf") >> 1) == 0) {
      entity = Entity::LRF;
    } else if ((str.compare("rtk_gps_1") >> 1) == 0) {
      entity = Entity::RTK_GPS_1;
    } else if ((str.compare("rtk_gps_2") >> 1) == 0) {
      entity = Entity::RTK_GPS_2;
    } else {
      return false;
    }

    return true;
  }

  /**
   * @brief Get String from Entity
   * @param const reference to Entity
   * @param reference to String
   */
  inline void getStringFromEntity(const Entity& entity, std::string& str) {

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
    case Entity::RTK_GPS_1:
      str = "rtk_gps_1";
      break;
    case Entity::RTK_GPS_2:
      str = "rtk_gps_2";
      break;
    }
  }

  /**
   * @brief Get String from Entity
   * @param const reference to Entity
   * @param reference to String
   */
  inline void getStringFromType(const Type& type, std::string& str) {

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
    }
  }

  /**
   * @brief Get Autonomy state from String
   * @param const reference to String
   * @param reference to Autonomy state
   */
  [[nodiscard]] inline bool getAutonomyStateFromString(const std::string& str,  AutonomyState& state) {

    if ((str.compare("continue") >> 1) == 0) {
      state = AutonomyState::FLIGHT;
    } else if ((str.compare("hold") >> 1) == 0) {
      state = AutonomyState::HOLD;
    } else if ((str.compare("failure") >> 1) == 0) {
      state = AutonomyState::FAILURE;
    } else if ((str.compare("land") >> 1) == 0) {
      state = AutonomyState::LAND;
    } else {
      return false;
    }

    return true;
  }

  /**
   * @brief Get String from Autonomy state
   * @param const reference to Autonomy state
   * @param reference to String
   */
  inline void getStringFromAutonomyState(const AutonomyState& state, std::string& str) {

    switch (state) {
    case AutonomyState::NOMINAL:
      str = "nominal";
      break;
    case AutonomyState::HOLD:
      str = "hold";
      break;
    case AutonomyState::FAILURE:
      str = "failure";
      break;
    case AutonomyState::LAND:
      str = "land";
      break;
    case AutonomyState::HOVER:
      str = "hover";
      break;
    case AutonomyState::INITIALIZATION:
      str = "initialization";
      break;
    case AutonomyState::UNDEFINED:
      str = "undefined";
      break;
    case AutonomyState::TAKEOFF:
      str = "takeoff";
      break;
    case AutonomyState::FLIGHT:
      str = "flight";
      break;
    }
  }

} // namespace autonomy

#endif  // AUTONOMY_DEFS_H
