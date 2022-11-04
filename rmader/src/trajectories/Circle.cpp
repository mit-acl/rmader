/**
 * @file Circle.cpp
 * @brief Circle class
 * @author Aleix Paris
 * @date 2020-02-18
 */

#include "trajectories/Circle.hpp"

namespace trajectory_generator
{
void Circle::generateTraj(std::vector<snapstack_msgs::Goal>& goals, std::unordered_map<int, std::string>& index_msgs)
{
  ros::Time tstart = ros::Time::now();
  // init pos
  double theta = 0;
  double v = 0;

  goals.push_back(createCircleGoal(v, 0, theta));

  for (int i = 0; i < v_goals_.size(); ++i)
  {
    double v_goal = v_goals_[i];
    index_msgs[goals.size() - 1] = "Circle traj: accelerating to " + std::to_string(v_goal) + " m/s";
    // accelerate to the goal velocity
    while (v < v_goal)
    {
      // generate points in the circle with *increasing* velocity
      v = std::min(v + accel_ * dt_, v_goal);
      double omega = v / r_;
      theta += omega * dt_;

      goals.push_back(createCircleGoal(v, accel_, theta));
    }

    // we reached v_goal, continue the traj for t_traj_ seconds
    if (fabs(v - v_goal) > 0.001)
    {
      ROS_WARN("Vels are not in increasing order, ignoring vels from the first to decrease...");
    }

    index_msgs[goals.size() - 1] = "Circle traj: reached " + std::to_string(v_goal) + " m/s, keeping constant v for " +
                                   std::to_string(t_traj_) + " s";
    double current_t_traj_ = 0;
    while (current_t_traj_ < t_traj_)
    {
      // generate points in the circle with *constant* velocity v == v_goal
      double omega = v / r_;
      theta += omega * dt_;

      goals.push_back(createCircleGoal(v, 0, theta));
      current_t_traj_ += dt_;
    }
  }
  // now decelerate to 0
  index_msgs[goals.size() - 1] = "Circle traj: decelerating to 0 m/s";
  while (v > 0)
  {
    // generate points in the circle with *decreasing* velocity
    v = std::max(v - accel_ * dt_, 0.0);
    double omega = v / r_;
    theta += omega * dt_;

    goals.push_back(createCircleGoal(v, -accel_, theta));
  }

  // we reached zero velocity
  if (fabs(v) > 0.001)
  {
    ROS_ERROR("Error: final velocity is not zero");
    exit(1);
  }
  index_msgs[goals.size() - 1] = "Circle traj: stopped";

  ROS_INFO("Time to calculate the traj (s): %f", (ros::Time::now() - tstart).toSec());
  ROS_INFO("Goal vector size = %lu", goals.size());
}

snapstack_msgs::Goal Circle::createCircleGoal(double v, double accel, double theta) const
{
  // TODO: return ref to goal to avoid copy? Same for the simpleInterpolation function
  double s = sin(theta);
  double c = cos(theta);
  double v2r = pow(v, 2) / r_;
  double v3r2 = pow(v, 3) / pow(r_, 2);
  double v4r3 = pow(v, 4) / pow(r_, 3);
  double omega = v / r_;

  snapstack_msgs::Goal goal;
  goal.header.frame_id = "world";
  goal.p.x = cx_ + r_ * c;
  goal.p.y = cy_ + r_ * s;
  goal.p.z = alt_;
  goal.v.x = -v * s;
  goal.v.y = v * c;
  goal.v.z = 0;
  goal.a.x = -v2r * c;  //+ accel*c; //this accel term makes a.x discontinuous
  goal.a.y = -v2r * s;  //+ accel*s; //this accel term makes a.y discontinuous
  goal.a.z = 0;
  goal.j.x = v3r2 * s;
  goal.j.y = -v3r2 * c;
  goal.j.z = 0;
  goal.j.x = v3r2 * s;
  goal.j.y = -v3r2 * c;
  goal.j.z = 0;
  // goal.s.x  = v4r3*c;
  // goal.s.y  = v4r3*s;
  // goal.s.z  = 0;
  goal.psi = theta + M_PI_2;
  goal.dpsi = omega;  // dyaw has to be in world frame, the outer loop already converts it to body
  goal.power = true;

  return goal;
}

void Circle::generateStopTraj(std::vector<snapstack_msgs::Goal>& goals,
                              std::unordered_map<int, std::string>& index_msgs, int& pub_index)
{
  ros::Time tstart = ros::Time::now();

  double v = sqrt(pow(goals[pub_index].v.x, 2) + pow(goals[pub_index].v.y, 2));  // 2D current (goal) vel
  double theta = atan2(goals[pub_index].p.y - cy_,
                       goals[pub_index].p.x - cx_);  // current (goal) angle wrt the center

  std::vector<snapstack_msgs::Goal> goals_tmp;
  std::unordered_map<int, std::string> index_msgs_tmp;

  index_msgs_tmp[0] = "Circle traj: pressed END, decelerating to 0 m/s";

  while (v > 0)
  {
    // generate points in the circle with *decreasing* velocity
    v = std::max(v - accel_ * dt_, 0.0);
    double omega = v / r_;
    theta += omega * dt_;

    goals_tmp.push_back(createCircleGoal(v, -accel_, theta));
  }

  // we reached zero velocity
  index_msgs_tmp[goals_tmp.size() - 1] = "Circle traj: stopped";

  goals = std::move(goals_tmp);
  index_msgs = std::move(index_msgs_tmp);
  pub_index = 0;

  ROS_INFO("Time to calculate the braking traj (s): %f", (ros::Time::now() - tstart).toSec());
  ROS_INFO("Goal vector size = %lu", goals.size());
}

bool Circle::trajectoryInsideBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  return isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, Eigen::Vector3d(cx_ - r_, cy_ - r_, alt_)) and
         isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, Eigen::Vector3d(cx_ + r_, cy_ + r_, alt_));
}

}  // namespace trajectory_generator
