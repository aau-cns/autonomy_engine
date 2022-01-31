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

#include "state_machine/states/land.h"

namespace autonomy {

  Land::Land() {};

  State& Land::Instance() {
    static Land singleton;
    return singleton;
  }

  void Land::onEntry(Autonomy& autonomy) {

    // print info
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> System state: LAND <<< \n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")) << std::endl);

    // Print info
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> Landing...\n")) << std::endl);

    // Request landing to the mission sequencer if flying
    if(autonomy.in_flight_) {
      autonomy.land_expected_ = true;
      autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::LAND);
    } else {
      AUTONOMY_UI_STREAM(BOLD(YELLOW(" >>> the platform is not flying, skipped LAND request\n")) << std::endl);
    }

  }

  void Land::onExit(Autonomy&) {}

}
