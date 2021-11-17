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

#ifndef STATE_H
#define STATE_H

#include "autonomy_core/autonomy.h"

namespace autonomy {

  /**
   * @brief State class
   */
  class State {

  public:

    /**
     * @brief Destructor
     */
    virtual ~State();

    /**
     * @brief Action to be performed when exiting a state
     * @param Reference to Autonomy
     */
    virtual void onExit(Autonomy& autonomy) = 0;

    /**
     * @brief Action to be performed when entering a state
     * @param Reference to Autonomy
     */
    virtual void onEntry(Autonomy& autonomy) = 0;

  protected:

    /// ROS node handler
    ros::NodeHandle nh_;

  }; // calss State

} // namespace autonomy

#endif  // STATE_H
