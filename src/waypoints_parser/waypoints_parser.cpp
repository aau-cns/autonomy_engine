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

#include "waypoints_parser/waypoints_parser.h"

namespace autonomy
{
WaypointsParser::WaypointsParser()
{
}

WaypointsParser::WaypointsParser(std::string& filename, std::vector<std::string>& categories)
  : filename_(filename), categories_(categories)
{
}

bool WaypointsParser::getIndices(const std::vector<std::string>& header, std::vector<int>& indices)
{
  // Temporary index
  int idx;

  // Get indices
  for (const auto& it : categories_)
  {
    // Get index
    if (!getIndex(header, it, idx))
    {
      return false;
    }
    indices.push_back(idx);
  }

  return true;
}

void WaypointsParser::parseLine(std::string& line, std::vector<std::string>& data)
{
  // Create a stringstream of the current line
  std::stringstream ss(line);

  // Temporary string
  std::string tmp;

  // Extract each cell
  while (std::getline(ss, tmp, ','))
  {
    data.push_back(tmp);
  }
}

void WaypointsParser::readParseCsv()
{
  std::ifstream file(filename_);

  if (!file)
  {
    throw std::runtime_error("Error opening file \"" + filename_ + "\". Exit programm.");
  }

  // std::cout << "----------------------------------------\nFile: " << filename_ << " successfully open.\n";

  // Line, header and data
  std::string line;
  std::vector<std::string> header;
  std::vector<std::vector<double>> data;

  // Indices (indices of the header corresponding to the defined convention)
  std::vector<int> indices;

  // rows counter
  int rows_cnt = 0;

  // Read the column names
  if (file.good())
  {
    // Extract the header (supposed to be the first line) in the file
    std::getline(file, line);

    // Parse the header
    parseLine(line, header);

    // Read data, line by line
    while (std::getline(file, line))
    {
      std::vector<double> tmp;
      parseLine(line, tmp);
      data.push_back(tmp);
      ++rows_cnt;
    }

    // Get association (indices) based on the defined convention
    if (!getIndices(header, indices))
    {
      throw std::runtime_error("Required data missing. Exit programm.");
    }

    // clear data structure from previous data
    data_.clear();

    // Loop through the data (lines) and fill the data structure
    for (const auto& it : data)
    {
      // Temporary waypoint data structure
      Waypoint tmp;

      // Fill out temporary waypoint data structure. If holdtime is not defined
      // within the categories then and if a fifth index is not present then set it to 0
      tmp.x = it.at(indices.at(0));
      tmp.y = it.at(indices.at(1));
      tmp.z = it.at(indices.at(2));
      tmp.yaw = it.at(indices.at(3));
      if (indices.size() > 4)
      {
        tmp.holdtime = it.at(indices.at(4));
      }
      else
      {
        tmp.holdtime = 0;
      }
      data_.push_back(tmp);
    }
  }

  file.close();
  // std::cout << "File read successfully.\n----------------------------------------\n" << std::endl;
}

bool WaypointsParser::fileSanityCheck()
{
  // Check filename_ is a file
  std::ifstream file(filename_);
  if (!file)
  {
    return false;
  }

  // Check filname header and data
  std::regex regex("(-?)(\\d+)(\\.\\d+)?");
  std::vector<int> indices;
  std::string line;
  std::vector<std::string> header;
  if (file.good())
  {
    std::getline(file, line);
    parseLine(line, header);
    if (!getIndices(header, indices))
    {
      return false;
    }
    while (std::getline(file, line))
    {
      std::vector<std::string> tmp;
      parseLine(line, tmp);
      if (tmp.size() != 0 && tmp.size() != indices.size())
      {
        return false;
      }
      for (const auto& it : tmp)
      {
        if (!std::regex_match(it, regex))
        {
          return false;
        }
      }
    }
  }

  return true;
}
}  // namespace autonomy
