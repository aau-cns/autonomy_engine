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

#ifndef UNDEFINED_H
#define UNDEFINED_H

#include "state_machine/state.h"

namespace autonomy
{
/**
 * @brief Undefined state at initialization
 */
class Undefined : public State
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
    return "undefined";
  }

private:
  /**
   * @brief Private constructor and copy-constructor
   */
  Undefined();
  Undefined(const Undefined& other);

  /**
   * @brief Assognment operator
   */
  Undefined& operator=(const Undefined& other);

};  // calss Undefined

}  // namespace autonomy

#endif  // UNDEFINED_H
