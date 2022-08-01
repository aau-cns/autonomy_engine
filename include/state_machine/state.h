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

#ifndef STATE_H
#define STATE_H

#include <iostream>

#include "autonomy_core/autonomy.h"
#include "utils/colors.h"
#include "utils/format.h"

namespace autonomy
{
/**
 * @brief State class
 */
class State
{
public:
  /**
   * @brief Destructor
   */
  virtual ~State();

  /**
   * @brief Action to be performed when exiting a state
   * @param Reference to Autonomy
   */
  virtual void onExit(Autonomy& autonomy) = 0;

  /**
   * @brief Action to be performed when entering a state
   * @param Reference to Autonomy
   */
  virtual void onEntry(Autonomy& autonomy) = 0;

  /**
   * @brief Return a String relative to the state
   */
  virtual const std::string getStringFromState() = 0;

};  // calss State

}  // namespace autonomy

#endif  // STATE_H
