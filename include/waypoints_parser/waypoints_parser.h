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

#ifndef WAYPOINTPARSER_H
#define WAYPOINTPARSER_H

#include <stdlib.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace autonomy
{
/**
 * @brief Input data parser forwaypoints.
 *
 * This class has a series of functions that allows to generate
 * data starting from a waypoint file for which convention is that
 * the first row contains the header defining what data is at each column.
 * Default convention --> x,y,z,yaw
 *
 */

class WaypointsParser
{
public:
  /**
   * @brief Input struct to parse a single line of a .csv file
   *
   * x
   * y
   * z
   * yaw
   */
  struct Waypoint
  {
    double x;
    double y;
    double z;
    double yaw;
    double holdtime;
  };

  /**
   * @brief constructor
   */
  WaypointsParser();

  /**
   * @brief constructor
   */
  WaypointsParser(std::string& filename, std::vector<std::string>& categories);

  /**
   * @brief set the filename
   */
  inline void setFilename(const std::string& filename)
  {
    filename_ = filename;
  }

  /**
   * @brief set the categoreis
   */
  inline void setCategories(const std::vector<std::string>& categories)
  {
    categories_ = categories;
  }

  /**
   * @brief Clear actual data, read a new .csv file and convert to a matrix (vector of vectors)
   */
  void readParseCsv();

  /**
   * @brief Get Data red from data structure
   */
  inline const std::vector<Waypoint>& getData() const
  {
    if (!data_.empty())
    {
      return data_;
    }
    else
    {
      throw std::runtime_error("Trying to get data from empty structure, something went wrong when parsing .csv input "
                               "file...");
    }
  }

  /**
   * @brief Sanity check for filename_. This method checks that the given filename_ is indeed a file.
   * Moreover it checks that it contains all the fileds related to categories_, and that at least N
   * values are given per line, where N is the number of categories.
   * Finally it checks if each value is a number (either int or float) via regex
   */
  [[nodiscard]] bool fileSanityCheck();

private:
  /**
   * @brief Filename of file containing waypoints
   */
  std::string filename_;

  /**
   * @brief Raw data from a .csv file converted to a matrix (vector of inputs)
   */
  std::vector<Waypoint> data_;

  /**
   * @brief vector of strings in header ordered based on defined convention -- x,y,z,yaw,holdtime --
   */
  std::vector<std::string> categories_ = { "x", "y", "z", "yaw", "holdtime" };

  /**
   * @brief Parse a single line of the .csv file
   *
   * overloaded function to parse a single line of a .csv
   * file with a comma as delimeter.
   * This function is overloaded to include either string values
   * (usually the case for headers) or numerical values
   */
  void parseLine(std::string& line, std::vector<std::string>& data);

  template <typename T>
  void parseLine(std::string& line, std::vector<T>& data)
  {
    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Temporary value
    T tmp;

    // Extract each cell
    while (ss >> tmp)
    {
      data.push_back(tmp);

      // skip commas
      if (ss.peek() == ',')
        ss.ignore();
    }
  }

  /**
   * @brief Find association between input file and defined convention
   *
   * The defined convention of the Input structure is -- x,y,z,yaw,holdtime --
   * This function find the indices of the columns of the input file based
   * on its header in order to correctly associate input data with the
   * Input structure allowing inpput files with shuffled columns or even
   * more columns than the onse that are necessary
   */
  [[nodiscard]] bool getIndices(const std::vector<std::string>& header, std::vector<int>& indices);

  /**
   * @brief Find the index of token within the given vector
   */
  template <typename T>
  bool getIndex(const std::vector<T>& data, const T& token, int& index)
  {
    // Iterator
    auto it = find(data.begin(), data.end(), token);

    // Check if element was found
    if (it != data.end())
    {
      // Get the index
      index = it - data.begin();
    }
    else
    {
      return false;
    }

    return true;
  }

};  // class WaypointsParser

}  // namespace autonomy
#endif  // WAYPOINTPARSER_H
