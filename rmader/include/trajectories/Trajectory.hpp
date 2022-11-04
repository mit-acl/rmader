/**
 * @file Trajectory.hpp
 * @brief Trajectory class
 * @author Aleix Paris
 * @date 2020-02-18
 */

#pragma once

#include <vector>
#include <unordered_map>
#include <Eigen/Core>

#include <ros/ros.h>
#include <snapstack_msgs/Goal.h>

namespace trajectory_generator
{
/*!
 * Abstract class that represents a trajectory. All trajectories inherit from it.
 */
class Trajectory
{
public:
  Trajectory(double dt) : dt_(dt)
  {
  }
  virtual ~Trajectory()
  {
  }

  // Generate the trajectory to be followed
  // It should only be called at init bc it can exit the program
  virtual void generateTraj(std::vector<snapstack_msgs::Goal>& goals,
                            std::unordered_map<int, std::string>& index_msgs) = 0;

  // Generate a stopping (braking) trajectory
  virtual void generateStopTraj(std::vector<snapstack_msgs::Goal>& goals,
                                std::unordered_map<int, std::string>& index_msgs, int& pub_index) = 0;

  // Return if the trajectory params conflict with the room bounds
  virtual bool trajectoryInsideBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) = 0;
  // TODO: add const to all const functions
protected:
  static bool isPointInsideBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax,
                                  Eigen::Vector3d point)
  {
    if (point.x() < xmin or point.x() > xmax)
      return false;
    if (point.y() < ymin or point.y() > ymax)
      return false;
    if (point.z() < zmin or point.z() > zmax)
      return false;
    return true;
  }

  static constexpr double GRAVITY = 9.81;  // m/s^2
  double dt_;                              // goal publication period [s]
};

}  // namespace trajectory_generator
