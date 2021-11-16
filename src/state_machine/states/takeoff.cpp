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
#include <amaze_mission_sequencer/request.h>
#include <amaze_mission_sequencer/response.h>

#include "state_machine/states/takeoff.h"
#include "utils/colors.h"

namespace autonomy {

  Takeoff::Takeoff() {};

  State& Takeoff::Instance() {
    static Takeoff singleton;
    return singleton;
  }

  void Takeoff::onEntry(Autonomy& autonomy) {

    // print info
    std::cout << BOLD(YELLOW("-------------------------------------------------\n"));
    std::cout << BOLD(YELLOW(" >>> System state: TAKEOFF <<< \n"));
    std::cout << BOLD(YELLOW("-------------------------------------------------\n")) << std::endl;

    // Perform preflight checks
    autonomy.preFlightChecks();

    // Send arm command to mission sequencer
    autonomy.arm();

  }

  void Takeoff::onExit(Autonomy& autonomy) {

    // Send waypoints
    autonomy.sendWaypoints();

  }

}
