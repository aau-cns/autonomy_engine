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

#include "state_machine/states/hold.h"

namespace autonomy {

  Hold::Hold() {};

  State& Hold::Instance() {
    static Hold singleton;
    return singleton;
  }

  void Hold::onEntry(Autonomy& autonomy) {

    // print info
    std::cout << BOLD(YELLOW("-------------------------------------------------\n"));
    std::cout << BOLD(YELLOW(" >>> System state: HOLD <<< \n"));
    std::cout << BOLD(YELLOW("-------------------------------------------------\n")) << std::endl;

    // Print info
    std::cout << BOLD(YELLOW(" >>> Holding...\n")) << std::endl;

    // Request holding to the mission sequencer if not holding
    if(!autonomy.holding_) {
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::HOLD);
    } else {
      std::cout << BOLD(YELLOW(" >>> the platform is already holding, skipped HOLD request\n")) << std::endl;
    }

  }

  void Hold::onExit(Autonomy&) {}

}
