/* ----------------------------------------------------------------------------
 * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "predictor.hpp"

int main(int argc, char **argv)
{
  std::cout << "Going to initialize predictor node" << std::endl;
  ros::init(argc, argv, "predictor");
  ros::NodeHandle nh("~");

  std::cout << "Creating Predictor object" << std::endl;
  Predictor Predictor(nh);

  while (ros::ok())
  {
    ros::spinOnce();  // spin the normal queue
  }

  ros::waitForShutdown();
  return 0;
}