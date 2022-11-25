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

#include <ros/ros.h>
#include <signal.h>
#include <functional>
#include <limits>

#include "autonomy_core/autonomy.h"

std::function<void(void)> sigintHandler;
void sigHandler(int)
{
  sigintHandler();
}

int main(int argc, char* argv[])
{
  // Init ros node with private nodehandler
  ros::init(argc, argv, "cns_flight_autonomy");
  ros::NodeHandle nh("~");

  // Load tcpNoDelay option (false by default)
  bool tcp_no_delay = false;
  nh.param<bool>("tcp_no_delay", tcp_no_delay, tcp_no_delay);

  // Provide a reliable low-latency communication if tcp_no_delay flag is true.
  // Note that, the tcpNoDelay option this affect negatively the bandwidth
  if (tcp_no_delay)
  {
    ros::TransportHints().tcpNoDelay();
  }

  // Start asynch (multi-threading) spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  try
  {
    // Instanciate autonomy
    autonomy::Autonomy autonomy(nh);

    // Sigaction to handle CTRL-C
    //    sigintHandler = std::bind(&autonomy::Autonomy::sigintHandler, &autonomy);
    sigintHandler = [&autonomy]() { autonomy.sigintHandler(); };
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    autonomy.logger_.logUI("undefined", ESCAPE(BOLD_ESCAPE, GREEN_ESCAPE),
                           " >>> Press [ENTER] to start the CNS-FLIGHT Autonomy\n");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Start the autonomy
    autonomy.startAutonomy();

    // Wait for the node to be killed
    ros::waitForShutdown();
  }
  catch (const FailureException&)
  {
    // Stop the spinner
    spinner.stop();

    // Print info
    std::cout << BOLD(RED(" >>> An unexpected error occured. Shutting down Autonomy <<<\n")) << std::endl;

    // Shoutdown
    ros::shutdown();
  }

  return EXIT_SUCCESS;
}
