// Copyright (C) 2021 Christian Brommer and Alessandro Fornasier,
// Control of Networked Systems, Universitaet Klagenfurt, Austria
//
// You can contact the author at <christian.brommer@ieee.org>
// and <alessandro.fornasier@ieee.org>
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

#include "autonomy_core/autonomy.h"
#include <ros/ros.h>
#include <limits>

int main(int argc, char* argv[])
{
  // Init ros node with private nodehandler
  ros::init(argc, argv, "amaze_autonomy");
  ros::NodeHandle nh("~");

  // Load tcpNoDelay option (false by default)
  bool tcp_no_delay = false;
  nh.param<bool>("tcp_no_delay", tcp_no_delay, tcp_no_delay);

  // Provide a reliable low-latency communication if tcp_no_delay flag is true.
  // Note that, the tcpNoDelay option this affect negatively the bandwidth
  if (tcp_no_delay) {
    ros::TransportHints().tcpNoDelay();
  }

  ROS_INFO("Starting the AMAZE Autonomy\n");

  // Start asynch (multi-threading) spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  try {

    // Instanciate AMAZE autonomy
    autonomy::Autonomy autonomy(nh);

    std::cout << BOLD(GREEN(" >>> Press [ENTER] to start the AMAZE Autonomy"));
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << std::endl;

    // Start the autonomy
    autonomy.startAutonomy();

    // Wait for the node to be killed
    ros::waitForShutdown();

  } catch (const FailureException&) {

    // Stop the spinner
    spinner.stop();

    // Print info
    std::cout << BOLD(RED(" >>> An error occured. Shutting down Autonomy <<<\n")) << std::endl;

  }

  return EXIT_SUCCESS;
}
