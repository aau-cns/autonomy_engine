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

#include <climits>

#include "autonomy_core/mission.h"
#include "utils/except.h"

namespace autonomy
{
Mission::Mission(const int& id, const std::string& description, const std::vector<std::string>& filepaths,
                 const std::map<Entity, std::string>& entity_state_map)
  : id_(id), description_(description), filepaths_(filepaths), entity_state_map_(entity_state_map)
{
  if (filepaths_.size() > INT_MAX)
  {
    throw DataOverflowException();
  }
  else
  {
    // Number of touchdown equal the number of sub-missions - 1
    number_of_touchdown_ = filepaths_.size() - 1;
  }
}

}  // namespace autonomy
