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

#include "state_machine/states/undefined.h"

namespace autonomy
{
Undefined::Undefined()
{
}

State& Undefined::Instance()
{
  static Undefined singleton;
  return singleton;
}

void Undefined::onEntry(Autonomy&)
{
}

void Undefined::onExit(Autonomy&)
{
}

}  // namespace autonomy
