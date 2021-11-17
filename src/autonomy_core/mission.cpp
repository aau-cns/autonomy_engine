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

#include <climits>

#include "autonomy_core/mission.h"
#include "utils/except.h"

namespace autonomy {

  Mission::Mission(const int& id, const std::string& description, const std::vector<std::string>& filepaths, const std::map<Entity, AutonomyState>& entity_state_map) :
    id_(id), description_(description), filepaths_(filepaths), entity_state_map_(entity_state_map) {

    if (filepaths_.size() > INT_MAX) {
      throw DataOverflowException();
    } else {
      // Number of touchdown equal the number of sub-missions - 1
      number_of_touchdown_ = filepaths_.size() - 1;
    }
  }

} // namespace autonomy
