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

#ifndef MISSION_H
#define MISSION_H

#include <map>
#include <string>
#include <vector>

#include "autonomy_core/autonomy_defs.h"

namespace autonomy
{
/**
 * @brief Class that define a mission including the filepaths
 * and the differnet actions to be tacken when
 * a failure for a specific entity is triggered
 */
class Mission
{
public:
  /**
   * @brief EntityEvent default constructor
   */
  Mission(const int& id, const std::string& description, const std::vector<std::string>& filepaths,
          const std::map<Entity, std::string>& entity_state_map);

  /**
   * @brief Get next state string for a given entity
   * @param const reference to Entity
   * @return reference to std::string
   */
  inline const std::string& getNextState(const Entity& entity) const
  {
    return entity_state_map_.at(entity);
  }

  /**
   * @brief Get description
   * @return reference to std::string
   */
  inline const std::string& getDescription() const
  {
    return description_;
  }

  /**
   * @brief Get filepaths
   * @return reference to std::vector<std::string>
   */
  inline const std::vector<std::string>& getFilepaths() const
  {
    return filepaths_;
  }

  /**
   * @brief Get number of touchdowns
   * @return reference to int
   */
  inline const size_t& getTouchdowns() const
  {
    return number_of_touchdown_;
  }

private:
  /// Mission id
  int id_;

  /// Mission description
  std::string description_;

  /// Mission filepaths
  std::vector<std::string> filepaths_;

  /// Entity - Next state map
  std::map<Entity, std::string> entity_state_map_;

  /// Number of touchdown (number of filepaths - 1)
  size_t number_of_touchdown_;
};

}  // namespace autonomy

#endif  // MISSION_H
