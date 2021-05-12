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

#ifndef ABSTRACT_STATE_H
#define ABSTRACT_STATE_H

#include <string>

#include "state_machine/state.h"

class AbstractState {

public:

  /**
   * @brief Destructor
   */
  virtual ~AbstractState();

  /**
   * @brief Trigger a state transition
   * @param Pointer to State
   * @param reference to failure that triggered the state changes
   */
  virtual void stateTransition(State* state, EntityEvent& event) = 0;

  /**
   * @brief Trigger a state transition from undefined state to nominal state
   * @param Pointer to State
   */
  virtual void nominal(State* state) = 0;

  /**
   * @brief Action to be performed when exiting a state
   * @param Pointer to State
   * @param Event that triggered the exit from the state
   */
  virtual void onExit(State* state, EntityEvent& event) = 0;

  /**
   * @brief Action to be performed when entering a state
   * @param Pointer to State
   * @param Event that triggered the entry to the state
   */
  virtual void onEntry(State* state, EntityEvent& event) = 0;

protected:

  /**
   * @brief State transition
   * @param Pointer to State
   * @param Reference to AbstractState
   */
  void setState(State* state, AbstractState& abstract_state);

  /**
   * @brief Set action to be performed
   * @param Pointer to State
   * @param const reference to Action
   * @param const reference to EntityEvent
   */
  void setAction(State* state, const Action& action_id, const EntityEvent& event);

  /**
   * @brief Get entity and subtype strings
   * @param Reference to event
   * @param Reference to string
   * @param Reference to string
   */
  void getEntityString(EntityEvent& event, std::string& entity, std::string& subtype);

};

#endif  // ABSTRACT_STATE_H
