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

#ifndef ENTITY_EVENT_H
#define ENTITY_EVENT_H

/**
 * @brief Entity
 */
enum Entity {UNKNOWN, PX4_GPS, PX4_IMU, PX4_MAG, PX4_BAR, MISSION_CAM, REALSENSE, LSM9DS1, LRF, RTK_GPS_1, RTK_GPS_2};

/**
 * @brief Type of entity event.
 *         - Failure in case of entity failure
 *         - Fix in case a past failure gets fixed
 *         - Other
 */
enum Type {FAILURE, FIX, OTHER};

/**
 * @brief Type of failure/fix
 */
enum subType {GLOBAL, TOPIC, NODE, DRIVER};

/**
 * @brief next state to be set when there is an entity event
 */
enum AutonomyState {NOMINAL, HOLD, MANUAL};

/**
 * @brief Action to be performed to react to an entity event
 */
enum Action {NOTHING, RESTART_NODE, RESTART_DRIVER, KILL_NODE};

/**
 * @brief Class that define a failure by means of Entity, Type, subType and AutonomyState
 */
class EntityEvent {

public:

  /**
   * @brief EntityEvent default constructor
   */
  EntityEvent();

  /**
   * @brief EntityEvent default constructor
   */
  EntityEvent(const Entity& entity, const Type& type, const subType& subtype, const AutonomyState& next_state);


  /**
   * @brief Get entity that caused the event
   * @return Const reference to Entity
   */
  const Entity& getEntity() const;

  /**
   * @brief Get type of the entity that caused the event
   * @return Const reference to Type
   */
  const Type& getType() const;

  /**
   * @brief Get subtype of the entity that caused the event
   * @return Const reference to subType
   */
  const subType& getSubType() const;

  /**
   * @brief Get next state
   * @return Const reference to AutonomyState
   */
  const AutonomyState& getNextState() const;

  /**
   * @brief Get has_failed boolean
   * @return Const reference to has_failed
   */
  const bool& is_init() const;

  /**
   * @brief Set Entity and type that caused the event and next state
   * @param entity
   * @param type
   * @param subtype
   * @param AutonomyState
   */
  void setEvent(const Entity& entity, const Type& type, const subType& subtype, const AutonomyState& next_state);

  /**
   * @brief Comparison operator overloading
   */
  bool operator==(const EntityEvent& e) const {
    if (init_ == true && e.init_ == true) {
      if (entity_ == e.entity_ && type_ == e.type_ && sub_type_ == e.sub_type_ && next_state_ == e.next_state_) {
        return true;
      }
    }
    return false;
  }

private:

  /**
   * @brief Boolean that define if an event has happend (initilized)
   */
  bool init_ = false;

  /**
   * @brief Entity that caused the event
   */
  Entity entity_;

  /**
   * @brief Type that caused the event
   */
  Type type_;

  /**
   * @brief subType that caused the event
   */
  subType sub_type_;

  /**
   * @brief Next state to be triggerd by the event
   */
  AutonomyState next_state_;

};

#endif  // ENTITY_EVENT_H
