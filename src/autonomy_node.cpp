// Copyright (C) 2021 Christian Brommer, Control of Networked Systems, Universitaet Klagenfurt, Austria
// You can contact the author at <christian.brommer@ieee.org>
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

#include "autonomy.h"
#include <ros/ros.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "amaze_autonomy");
  ros::NodeHandle nh;

  ros::TransportHints().tcpNoDelay();

  ROS_INFO("Starting the AMAZE Autonomy");

  AmazeAutonomy autonomy(nh);

  ros::spin();
}
