/**
 * @file trajectory_generator_node.cpp
 * @brief Trajectory Generator node
 * @author Aleix Paris
 * @date 2020-01-08
 */

#include <ros/ros.h>
#include "TrajectoryGenerator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  trajectory_generator::TrajectoryGenerator trajectoryGenerator(nhtopics, nhparams);
  ros::spin();
  return 0;
}
