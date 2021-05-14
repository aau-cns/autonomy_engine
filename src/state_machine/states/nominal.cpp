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

#include "state_machine/states/nominal.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/manual.h"
#include "utils/colors.h"

Nominal::Nominal() {};

AbstractState& Nominal::Instance() {
  static Nominal singleton;
  return singleton;
}

void Nominal::stateTransition(State* state, EntityEvent& event) {

  // Switch to next state
  switch (event.getNextState()) {
  case AutonomyState::HOLD:
    setState(state, Hold::Instance());
    break;
  case AutonomyState::MANUAL:
    setState(state, Manual::Instance());
    break;
  case AutonomyState::NOMINAL:
    setState(state, Nominal::Instance());
    break;
  }
}

void Nominal::onExit(State*, EntityEvent& event) {

  std::string entity, subtype;

  // get string corresponding to entity, subtype
  getEntityString(event, entity, subtype);

  // Print info
  switch (event.getType()) {
  case Type::FAILURE:
    if (event.getNextState() == AutonomyState::NOMINAL) {
      std::cout << std::endl << BOLD(GREEN("-------------------------------------------------")) << std::endl << std::endl;
      std::cout << BOLD(GREEN(" Detected " + entity + " FAILURE")) << std::endl;
      std::cout << BOLD(GREEN(" Type: " + subtype + " FAILURE")) << std::endl;
      std::cout << BOLD(GREEN(" NON-CRITICAL FAILURE >>> Next state: NOMINAL")) << std::endl;
      std::cout << std::endl << BOLD(GREEN("-------------------------------------------------")) << std::endl;
    } else if (event.getNextState() == AutonomyState::HOLD) {
      std::cout << std::endl << BOLD(YELLOW("------------------------------------------------")) << std::endl << std::endl;
      std::cout << BOLD(YELLOW(" Detected " + entity + " FAILURE")) << std::endl;
      std::cout << BOLD(YELLOW(" Type: " + subtype + " FAILURE")) << std::endl;
      std::cout << BOLD(YELLOW(" INCONVENIENT FAILURE >>> Next state: HOLD")) << std::endl;
      std::cout << std::endl << BOLD(YELLOW("-------------------------------------------------")) << std::endl;
    } else if (event.getNextState() == AutonomyState::MANUAL) {
      std::cout << std::endl << BOLD(RED("-------------------------------------------------")) << std::endl << std::endl;
      std::cout << BOLD(RED(" Detected " + entity + " FAILURE")) << std::endl;
      std::cout << BOLD(RED(" Type: " + subtype + " FAILURE")) << std::endl;
      std::cout << BOLD(RED(" CRITICAL FAILURE >>> Next state: MANUAL")) << std::endl;
      std::cout << std::endl << BOLD(RED("-------------------------------------------------")) << std::endl;
    }
    break;
  case Type::FIX:
    std::cout << std::endl << BOLD(GREEN("-------------------------------------------------")) << std::endl << std::endl;
    std::cout << BOLD(GREEN(" Detected " + entity + " FIX")) << std::endl;
    std::cout << BOLD(GREEN(" Type: " + subtype + " FIX")) << std::endl;
    std::cout << BOLD(GREEN(" FIX >>> Next state: NOMINAL")) << std::endl;
    std::cout << std::endl << BOLD(GREEN("-------------------------------------------------")) << std::endl;
    break;
  default:
    break;
  }

}

void Nominal::onEntry(State* state, EntityEvent& event) {

  // print info
  std::cout << std::endl << BOLD(GREEN("-------------------------------------------------")) << std::endl << std::endl;
  std::cout << BOLD(GREEN(" >>> System state: NOMINAL <<< ")) << std::endl;
  std::cout << std::endl << BOLD(GREEN("-------------------------------------------------")) << std::endl;

  // Set no action
  setAction(state, Action::NOTHING, event);
}

void Nominal::nominal(State*) {}
