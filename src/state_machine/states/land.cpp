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

#include <iostream>

#include "state_machine/states/land.h"
#include "utils/colors.h"

namespace autonomy {

  Land::Land() {};

  State& Land::Instance() {
    static Land singleton;
    return singleton;
  }

  void Land::onEntry(Autonomy& autonomy) {

    // print info
    std::cout << BOLD(YELLOW("-------------------------------------------------\n"));
    std::cout << BOLD(YELLOW(" >>> System state: LAND <<< \n"));
    std::cout << BOLD(YELLOW("-------------------------------------------------\n")) << std::endl;

    // Print info
    std::cout << BOLD(GREEN(" >>> Landing...\n")) << std::endl;

    // Request landing to the mission sequencer if flying
    if(autonomy.in_flight_) {
      autonomy.missionSequencerRequest(amaze_mission_sequencer::request::LAND);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is not flying, skipped LAND request\n")) << std::endl;
    }

  }

  void Land::onExit(Autonomy&) {}

}
