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

#ifndef NOMINAL_H
#define NOMINAL_H

#include "state_machine/state.h"

namespace autonomy
{
/**
 * @brief Land state.
 */
class Nominal : public State
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
    return "nominal";
  }

private:
  /**
   * @brief Private constructor and copy-constructor
   */
  Nominal();
  Nominal(const Nominal& other);

  /**
   * @brief Assognment operator
   */
  Nominal& operator=(const Nominal& other);

};  // calss Nominal

}  // namespace autonomy

#endif  // NOMINAL_H
