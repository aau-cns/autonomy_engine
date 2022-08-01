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

#ifndef HOLD_H
#define HOLD_H

#include "state_machine/state.h"

namespace autonomy
{
/**
 * @brief Hold state.
 */
class Hold : public State
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
    return "hold";
  }

private:
  /**
   * @brief Private constructor and copy-constructor
   */
  Hold();
  Hold(const Hold& other);

  /**
   * @brief Assognment operator
   */
  Hold& operator=(const Hold& other);

};  // calss Hold

}  // namespace autonomy

#endif  // HOLD_H
