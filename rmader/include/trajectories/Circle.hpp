/**
 * @file Circle.hpp
 * @brief Circle class
 * @author Aleix Paris
 * @date 2020-02-18
 */

#pragma once

#include "trajectories/Trajectory.hpp"

namespace trajectory_generator
{
/*!
 * Class that represents a circular trajectory
 */
class Circle : public Trajectory
{
public:
  Circle(double alt, double r, double cx, double cy, std::vector<double> v_goals, double t_traj, double accel,
         double dt)
    : alt_(alt), r_(r), cx_(cx), cy_(cy), v_goals_(v_goals), t_traj_(t_traj), accel_(accel), Trajectory(dt)
  {
  }

  virtual ~Circle()
  {
  }

  void generateTraj(std::vector<snapstack_msgs::Goal>& goals,
                    std::unordered_map<int, std::string>& index_msgs) override;  // generates the circles

  snapstack_msgs::Goal createCircleGoal(double v, double accel, double theta) const;

  void generateStopTraj(std::vector<snapstack_msgs::Goal>& goals, std::unordered_map<int, std::string>& index_msgs,
                        int& pub_index) override;

  bool trajectoryInsideBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax) override;

private:
  double alt_;      // altitude in m
  double r_;        // radius in m
  double cx_, cy_;  // circle center in m

  std::vector<double> v_goals_;  // norm of goal velocities
  double t_traj_;                // time to follow each trajectory, s
  double accel_;                 // max acceleration allowed
};

}  // namespace trajectory_generator
