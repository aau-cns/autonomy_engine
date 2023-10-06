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

#include "autonomy_core/mission.h"

#include <limits>

#include "utils/except.h"

namespace autonomy
{
Mission::Mission(const int& id, const std::string& description, const std::vector<std::string>& filepaths,
                 const std::map<Entity, std::string>& entity_state_map, const float& instances)
  : id_(id)
  , description_(description)
  , filepaths_(filepaths)
  , entity_state_map_(entity_state_map)
  , instances_(instances)
{
  // Number of touchdown equal the number of (sub-missions * instances) - 1
  instances_ == std::numeric_limits<float>::infinity() ?
      number_of_touchdown_ = std::numeric_limits<size_t>::max() :
      number_of_touchdown_ = (filepaths_.size() * static_cast<size_t>(instances_) - 1);
  ;
}

}  // namespace autonomy
