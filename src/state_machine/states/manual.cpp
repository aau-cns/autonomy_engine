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

#include "state_machine/states/manual.h"
#include "utils/colors.h"

Manual::Manual() {};

AbstractState& Manual::Instance() {
  static Manual singleton;
  return singleton;
}

void Manual::stateTransition(State*, EntityEvent&) {}

void Manual::onExit(State*, EntityEvent&) {}

void Manual::onEntry(State* state, EntityEvent& event) {

  // Print info
  std::cout << std::endl << BOLD(RED("-------------------------------------------------")) << std::endl << std::endl;
  std::cout << BOLD(RED(" >>> PLEASE TAKE MANUAL CONTROL OF <<< ")) << std::endl;
  std::cout << BOLD(RED(" >>> THE PLATFORM AND LAND SAFELY  <<< ")) << std::endl;
  std::cout << std::endl << BOLD(RED("-------------------------------------------------")) << std::endl;

  // Set no action
  setAction(state, Action::NOTHING, event);

}

void Manual::nominal(State*) {}
