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

#include "state_machine/states/initialization.h"

namespace autonomy
{
Initialization::Initialization()
{
}

State& Initialization::Instance()
{
  static Initialization singleton;
  return singleton;
}

void Initialization::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatStateEntry("INITIALIZATION"));

  // Perform initialization of the watchdog and RC aux registration
  if (autonomy.opts_->activate_watchdog)
  {
    if (!autonomy.startWatchdog())
    {
      autonomy.stateTransition("failure");
    }
  }

  if (autonomy.opts_->register_aux)
  {
    if (!autonomy.registerRCAux())
    {
      autonomy.stateTransition("failure");
    }
  }

  // Transition to nominal
  autonomy.stateTransition("nominal");
}

void Initialization::onExit(Autonomy&)
{
}

}  // namespace autonomy
