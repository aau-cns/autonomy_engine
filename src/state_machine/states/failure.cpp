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

#include "state_machine/states/failure.h"

namespace autonomy
{
Failure::Failure(){};

State& Failure::Instance()
{
  static Failure singleton;
  return singleton;
}

void Failure::onEntry(Autonomy& autonomy)
{
  // Print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, RED_ESCAPE), formatStateEntry("FAILURE"));

  // If flying send an abort request to mission sequencer
  if (autonomy.in_flight_)
  {
    autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::ABORT);
  }

  // Call state transition to TERMINATION
  autonomy.stateTransition("termination");
}

void Failure::onExit(Autonomy&)
{
}

}  // namespace autonomy
