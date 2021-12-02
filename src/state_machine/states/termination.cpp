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

#include "state_machine/states/termination.h"

namespace autonomy {

  Termination::Termination() {};

  State& Termination::Instance() {
    static Termination singleton;
    return singleton;
  }

  void Termination::onEntry(Autonomy& autonomy) {

    // print info
    std::cout << BOLD(MAGENTA("-------------------------------------------------\n"));
    std::cout << BOLD(MAGENTA(" >>> System state: TERMINATION <<< \n"));
    std::cout << BOLD(MAGENTA("-------------------------------------------------\n")) << std::endl;

    // Wait
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Shoutdown
    autonomy.nh_.shutdown();
    std::cout << BOLD(MAGENTA(" >>> Press CTRL-C to terminate the autonomy <<< \n")) << std::endl;

  }

  void Termination::onExit(Autonomy&) {}

}
