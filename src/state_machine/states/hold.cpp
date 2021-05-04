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

#include <iostream>
#include <algorithm>

#include "state_machine/states/hold.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/manual.h"
#include "utils/colors.h"

Hold::Hold() {};

AbstractState& Hold::Instance() {
  static Hold singleton;
  return singleton;
}

void Hold::stateTransition(State* state, EntityEvent& event) {

  // Switch to next state
  switch (event.getNextState()) {

  case NextState::NOMINAL:
    // Check wheter the event report a failre or a fix to a previous failre
    if (event.getType() == Type::FAILURE) {
      // In case of a failure the HOLD state must be kept untill all pending failures that
      // require the HOLD state get solved. Thus the next state is set to HOLD
      event.setFailure(event.getEntity(), event.getType(), event.getSubType(), NextState::HOLD);
      setState(state, Hold::Instance());
    } else if (event.getType() == Type::FIX) {
      // The fixed failure has already been removed from the pending failures
      // check if there are still pending failures that require HOLD state (NextState = HOLD)
      const auto &it = find_if(state->getPendingFailures().begin(), state->getPendingFailures().end(), [](const EntityEvent& failure){ return failure.getNextState() == NextState::HOLD;});
      if (it != state->getPendingFailures().end()) {
        // In case of a failure the HOLD state must be kept untill all pending failures that
        // require the HOLD state get solved. Thus the next state is set to HOLD
        event.setFailure(event.getEntity(), event.getType(), event.getSubType(), NextState::HOLD);
        setState(state, Hold::Instance());
      } else {
        setState(state, Nominal::Instance());
      }
    }
    break;

  case NextState::MANUAL:
    setState(state, Manual::Instance());
    break;

  case NextState::HOLD:
    setState(state, Hold::Instance());
    break;
  }
}

void Hold::onExit(State*, EntityEvent& event) {

  std::string entity, subtype;

  // get string corresponding to entity, subtype
  getEntityString(event, entity, subtype);

  // Print info
  switch (event.getType()) {
  case Type::FAILURE:
    if (event.getNextState() == NextState::NOMINAL) {
      std::cout << std::endl << BOLD(GREEN("---------------------------------------")) << std::endl << std::endl;
      std::cout << BOLD(GREEN(" Detected " + entity + " FAILURE")) << std::endl;
      std::cout << BOLD(GREEN(" Type: " + subtype + " FAILURE")) << std::endl;
      std::cout << BOLD(GREEN(" NON-CRITICAL FAILURE >>> Next state: NOMINAL")) << std::endl;
    } else if (event.getNextState() == NextState::HOLD) {
      std::cout << std::endl << BOLD(YELLOW("---------------------------------------")) << std::endl << std::endl;
      std::cout << BOLD(YELLOW(" Detected " + entity + " FAILURE")) << std::endl;
      std::cout << BOLD(YELLOW(" Type: " + subtype + " FAILURE")) << std::endl;
      std::cout << BOLD(YELLOW(" INCONVENIENT FAILURE >>> Next state: HOLD")) << std::endl;
      std::cout << BOLD(YELLOW("---------------------------------------")) << std::endl;
    } else if (event.getNextState() == NextState::MANUAL) {
      std::cout << std::endl << BOLD(RED("---------------------------------------")) << std::endl << std::endl;
      std::cout << BOLD(RED(" Detected " + entity + " FAILURE")) << std::endl;
      std::cout << BOLD(RED(" Type: " + subtype + " FAILURE")) << std::endl;
      std::cout << BOLD(RED(" CRITICAL FAILURE >>> Next state: MANUAL")) << std::endl;
      std::cout << BOLD(RED("---------------------------------------")) << std::endl;
    }
    break;
  case Type::FIX:
    std::cout << std::endl << BOLD(GREEN("---------------------------------------")) << std::endl << std::endl;
    std::cout << BOLD(GREEN(" Detected " + entity + " FIX")) << std::endl;
    std::cout << BOLD(GREEN(" Type: " + subtype + " FIX")) << std::endl;
    if (event.getNextState() == NextState::NOMINAL) {
      std::cout << BOLD(GREEN(" FIX >>> Next state: NOMINAL")) << std::endl;
    } else if (event.getNextState() == NextState::HOLD) {
      std::cout << BOLD(GREEN(" Existing pending failures >>> Next state: HOLD")) << std::endl;
    }
    std::cout << BOLD(GREEN("---------------------------------------")) << std::endl;
    break;
  }
}

void Hold::onEntry(State*, EntityEvent&) {

  // print info
  std::cout << std::endl << BOLD(YELLOW("---------------------------------------")) << std::endl << std::endl;
  std::cout << BOLD(YELLOW(" >>> System state: HOLD <<< ")) << std::endl;
  std::cout << BOLD(YELLOW("---------------------------------------")) << std::endl;

  // Take an ection to react to the failure
}

