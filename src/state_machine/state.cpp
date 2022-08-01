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

#include "state_machine/states/end_mission.h"
#include "state_machine/states/failure.h"
#include "state_machine/states/hold.h"
#include "state_machine/states/initialization.h"
#include "state_machine/states/land.h"
#include "state_machine/states/nominal.h"
#include "state_machine/states/perform_mission.h"
#include "state_machine/states/preflight.h"
#include "state_machine/states/start_mission.h"
#include "state_machine/states/termination.h"
#include "state_machine/states/undefined.h"

namespace autonomy
{
State::~State()
{
}

}  // namespace autonomy
