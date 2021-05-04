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

#include <string>
#include <iostream>

#include "state_machine/abstract_state.h"
#include "utils/colors.h"

AbstractState::~AbstractState() {}

void AbstractState::stateTransition(State*, EntityEvent&) {}

void AbstractState::setState(State* state, AbstractState& abstract_state) {
  state->setState(abstract_state);
}



void AbstractState::getEntityString(EntityEvent& event, std::string& entity, std::string& subtype) {

  // Check if there is a valid event
  if(event.is_init()) {
    switch (event.getEntity()) {
    case Entity::PX4_GPS:
      entity = "PX4 GPS";
      break;
    case Entity::PX4_BAR:
      entity = "PX4 BAROMETER";
      break;
    case Entity::PX4_MAG:
      entity = "PX4 MAGNETOMETER";
      break;
    case Entity::MISSION_CAM:
      entity = "MISSION CAMERA";
      break;
    case Entity::REALSENSE:
      entity = "REALSENSE";
      break;
    case Entity::LSM9DS1:
      entity = "LSM9DS1 IMU";
      break;
    case Entity::LRF:
      entity = "LASER RANGE FINDER";
      break;
    case Entity::RTK_GPS_1:
      entity = "RTK GPS 1";
      break;
    case Entity::RTK_GPS_2:
      entity = "RTK GPS 2";
      break;
    default:
      break;
    }
  }

    switch (event.getSubType()) {
    case subType::TOPIC:
      subtype = "TOPIC";
      break;
    case subType::NODE:
      subtype = "NODE";
      break;
    case subType::DRIVER:
      subtype = "DRIVER";
      break;
    }
}
