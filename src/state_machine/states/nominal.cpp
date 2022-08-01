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

#include "state_machine/states/nominal.h"

namespace autonomy
{
Nominal::Nominal(){};

State& Nominal::Instance()
{
  static Nominal singleton;
  return singleton;
}

void Nominal::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE), formatStateEntry("NOMINAL (IDLE)"));

  // Start data recording if enabled
  if (autonomy.opts_->activate_data_recording && !autonomy.is_recording_)
  {
    autonomy.DataRecording(true);
  }

  // Setting state to PREFLIGHT
  // This will perform first the preflight checks if they are enabled
  autonomy.stateTransition("preflight");
}

void Nominal::onExit(Autonomy&)
{
}

}  // namespace autonomy
