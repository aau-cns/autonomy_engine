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

#ifndef START_MISSION_H
#define START_MISSION_H

#include "state_machine/state.h"

namespace autonomy {

  /**
   * @brief Land state.
   */
  class StartMission : public State {

  public:

    /**
     * @brief Instance of State (Singleton)
     */
    static State& Instance();

    /**
     * @brief Action to be performed when exiting a state
     * @param Reference to Autonomy
     */
    void onExit(Autonomy& autonomy) override;

    /**
     * @brief Action to be performed when entering a state
     * @param  Reference to Autonomy
     */
    void onEntry(Autonomy& autonomy) override;

    /**
     * @brief Return a String relative to the state
     */
    const std::string getStringFromState() override {
      return "start_mission";
    }

  private:

    /**
     * @brief Private constructor and copy-constructor
     */
    StartMission();
    StartMission(const StartMission& other);

    /**
     * @brief Assognment operator
     */
    StartMission& operator=(const StartMission& other);

  }; // calss StartMission

} // namespace autonomy

#endif  // START_MISSION_H
