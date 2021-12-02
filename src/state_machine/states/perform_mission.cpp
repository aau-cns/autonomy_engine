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

#include "state_machine/states/perform_mission.h"

namespace autonomy {

  PerformMission::PerformMission() {};

  State& PerformMission::Instance() {
    static PerformMission singleton;
    return singleton;
  }

  void PerformMission::onEntry(Autonomy&) {

    // print info
    std::cout << BOLD(GREEN("-------------------------------------------------\n"));
    std::cout << BOLD(GREEN(" >>> System state: MISSION (PERFORMING) <<< \n"));
    std::cout << BOLD(GREEN("-------------------------------------------------\n")) << std::endl;

  }

  void PerformMission::onExit(Autonomy&) {}

}
