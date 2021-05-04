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

#ifndef STATE_H
#define STATE_H

#include <vector>

#include "autonomy_core/entity_event.h"

class AbstractState;

/**
 * @brief State class
 */
class State {

public:

  /**
   * @brief State constructor
   */
  State();

  /**
   * @brief Trigger a state transition
   * @param reference to entity event
   */
  void stateTransition(EntityEvent& event);

  /**
   * @brief Get current state
   * @return const pointer to current state
   */
  const AbstractState* getCurrentState() const;

  /**
   * @brief Get pending failures
   * @return const reference to pending failures vector
   */
  const std::vector<EntityEvent>& getPendingFailures() const;

  /**
   * @brief Get actual (last registered) event
   * @return const reference to EntityEvent
   */
  const EntityEvent& getEntityEvent() const;

private:

  /**
   * @brief Friend AbstractState (can access private part of State)
   */
  friend class AbstractState;

  /**
   * @brief Set state
   * @param reference to AbstractState
   */
  void setState(AbstractState& abstract_state);

private:

  // Pointer to AbstractState
  AbstractState* abstract_state_;  

  // Actual (last registered) event
  EntityEvent event_;

  // History of failures
  std::vector<EntityEvent> pending_failures_;

};

#endif  // STATE_H
