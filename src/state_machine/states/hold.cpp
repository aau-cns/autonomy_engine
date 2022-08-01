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

#include "state_machine/states/hold.h"

namespace autonomy
{
Hold::Hold(){};

State& Hold::Instance()
{
  static Hold singleton;
  return singleton;
}

void Hold::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), formatStateEntry("HOLD"));
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE), formatMsg("Holding...", 2));

  // Request holding to the mission sequencer if not holding
  if (!autonomy.holding_)
  {
    autonomy.missionSequencerRequest(mission_sequencer::MissionRequest::HOLD);
  }
  else
  {
    autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, YELLOW_ESCAPE),
                           formatMsg("The platform is already holding, skipped HOLD request", 2));
  }
}

void Hold::onExit(Autonomy&)
{
}

}  // namespace autonomy
