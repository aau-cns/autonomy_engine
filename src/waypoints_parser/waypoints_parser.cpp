// Copyright (C) 2021 Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <alessandro.fornasier@ieee.org>
//
// All rights reserved.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "waypoints_parser/waypoints_parser.h"

namespace autonomy {

  WaypointsParser::WaypointsParser() {}

  WaypointsParser::WaypointsParser(std::string &filename, std::vector<std::string> &categories):
    filename_(filename), categories_(categories) {}

  void WaypointsParser::getIndices(const std::vector<std::string> &header, std::vector<int> &indices) {

    // Temporary index
    int idx;

    // Get indices
    for (const auto &it : categories_) {

      // Get index
      getIndex(header, it, idx);
      indices.push_back(idx);
    }
  }

  void WaypointsParser::parseLine(std::string &line, std::vector<std::string> &data) {

    // Create a stringstream of the current line
    std::stringstream ss(line);

    // Temporary string
    std::string tmp;

    // Extract each cell
    while (std::getline(ss, tmp, ',')) {
      data.push_back(tmp);
    }
  }

  void WaypointsParser::readParseCsv() {

    std::ifstream file(filename_);

    if (!file) {
      throw std::runtime_error("Error opening file \"" + filename_ + "\". Exit programm.");
    }

    //std::cout << "----------------------------------------\nFile: " << filename_ << " successfully open.\n";

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
      while (std::getline(file, line)) {
        std::vector<double> tmp;
        parseLine(line, tmp);
        data.push_back(tmp);
        ++rows_cnt;
      }

      // Get association (indices) based on the defined convention
      getIndices(header, indices);

      // clear data structure from previous data
      data_.clear();

      // Loop through the data (lines) and fill the data structure
      for (const auto &it : data) {

        // Temporary waypoint data structure
        Waypoint tmp;

        // Fill out temporary waypoint data structure, if x,y,z,yaw are mandatory data while waiting_time not.
        // If waiting time is not present then set it to 0
        tmp.x = it.at(indices.at(0));
        tmp.y = it.at(indices.at(1));
        tmp.z = it.at(indices.at(2));
        tmp.yaw = it.at(indices.at(3));
        if (indices.size() > 4) {
          tmp.holdtime = it.at(indices.at(4));
        } else {
          tmp.holdtime = 0;
        }
        data_.push_back(tmp);
      }
    }

    file.close();
    //std::cout << "File read successfully.\n----------------------------------------\n" << std::endl;
  }

} // namespace autonomy
