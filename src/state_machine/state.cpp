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

#include <algorithm>

#include "state_machine/state.h"
#include "state_machine/states/undefined.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/manual.h"

State::State() {

  // Initialized state to be Undefined
  abstract_state_ = &Undefined::Instance();
}

void State::nominal() {
  abstract_state_->nominal(this);
}

bool State::stateTransition(EntityEvent& event) {

  // Register new event
  event_ = event;

  // If event is a failure push failure into pending failures
  // otherwise remove fixed failure from pending failures
  if (event.getType() == Type::FAILURE) {
    pending_failures_.emplace_back(event);

  } else if (pending_failures_.size() > 0 && event.getType() == Type::FIX) {  

    // Define temporary event replacing FIX type with FAILURE type
    EntityEvent e(event.getEntity(), Type::FAILURE, event.getSubType(), event.getNextState());

    // remove fixed failure from pending failures by shift
    // It does not change the size and return iterator to the new last element
    // which position is < then the end of the vector
    const auto &it = std::remove_if(pending_failures_.begin(), pending_failures_.end(), [&e](EntityEvent const& failure){return failure == e;});

    if (it != pending_failures_.end()) {
      // erase "removed" elements
      pending_failures_.erase(it, pending_failures_.end());
    } else {
      return false;
    }
  } else {
    // handle case of a FIX without the respective FAILURE
    return false;
  }

  // State transition
  abstract_state_->stateTransition(this, event_);

  return true;
}

void State::setState(AbstractState& abstract_state) {

  // Perform action before exiting old state
  abstract_state_->onExit(this, event_);

  // Change state
  abstract_state_ = &abstract_state;
  state_ = event_.getNextState();

  // Perform action when entering new state
  abstract_state_->onEntry(this, event_);
}

void State::setAction(const Action& action, const EntityEvent& event) {
  action_.first = action;
  action_.second = event;
}

const EntityEvent& State::getEntityEvent() const {
  return event_;
}

const std::vector<EntityEvent>& State::getPendingFailures() const {
  return pending_failures_;
}

const std::pair<Action, EntityEvent>& State::getAction() const {
  return action_;
}

const AutonomyState& State::getState() const {
  return state_;
}
