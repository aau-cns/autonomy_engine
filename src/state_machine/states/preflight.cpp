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

#include "state_machine/states/preflight.h"

namespace autonomy {

  Preflight::Preflight() {};

  State& Preflight::Instance() {
    static Preflight singleton;
    return singleton;
  }

  void Preflight::onEntry(Autonomy& autonomy) {

    // print info
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN(" >>> System state: PREFLIGHT <<< \n")));
    AUTONOMY_UI_STREAM(BOLD(GREEN("-------------------------------------------------\n")) << std::endl);

    // Perform preflight checks and state transition
    if (autonomy.preFlightChecks()) {
      autonomy.stateTransition("start_mission");
    } else {
      autonomy.stateTransition("failure");
    }

  }

  void Preflight::onExit(Autonomy&) {}

}
