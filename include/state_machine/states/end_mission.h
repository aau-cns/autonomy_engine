// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <alessandro.fornasier@ieee.org>
#ifndef END_MISSION_H
#define END_MISSION_H

#include "state_machine/state.h"

namespace autonomy
{
/**
 * @brief Land state.
 */
class EndMission : public State
{
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
  const std::string getStringFromState() override
  {
    return "end_mission";
  }

private:
  /**
   * @brief Private constructor and copy-constructor
   */
  EndMission();
  EndMission(const EndMission& other);

  /**
   * @brief Assognment operator
   */
  EndMission& operator=(const EndMission& other);

  /**
   * @brief Perform last steps anc call termination
   */
  void Terminate(Autonomy& autonomy);

};  // calss EndMission

}  // namespace autonomy

#endif  // END_MISSION_H
