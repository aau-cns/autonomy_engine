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

#include "state_machine/states/termination.h"

namespace autonomy
{
Termination::Termination(){};

State& Termination::Instance()
{
  static Termination singleton;
  return singleton;
}

void Termination::onEntry(Autonomy& autonomy)
{
  // print info
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, MAGENTA_ESCAPE), formatStateEntry("TERMINATION"));

  // Terminate remaining timers
  autonomy.watchdog_timer_->stopTimer();
  for (const auto& it : autonomy.pending_failures_)
  {
    it.second->stopTimer();
  }

  // Wait
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Shoutdown
  autonomy.nh_.shutdown();
  autonomy.logger_.logUI(getStringFromState(), ESCAPE(BOLD_ESCAPE, MAGENTA_ESCAPE),
                         formatMsg("Press CTRL-C to terminate the autonomy"));
}

void Termination::onExit(Autonomy&)
{
}

}  // namespace autonomy
