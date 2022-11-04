/**
 * @file TrajectoryGenerator.hpp
 * @brief Trajectory Generator class
 * @author Aleix Paris
 * @date 2020-01-08
 */

#pragma once

#include <vector>

#include "trajectories/Trajectory.hpp"

// ROS
#include <ros/ros.h>
#include <snapstack_msgs/Goal.h>
#include <snapstack_msgs/QuadFlightMode.h>
#include <snapstack_msgs/State.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace trajectory_generator
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class TrajectoryGenerator
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TrajectoryGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);

  /*!
   * Destructor.
   */
  virtual ~TrajectoryGenerator();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  void modeCB(const snapstack_msgs::QuadFlightMode& msg);
  void stateCB(const snapstack_msgs::State& msg);
  void pubCB(const ros::TimerEvent& event);

  void resetGoal();

  // Utils
  static snapstack_msgs::Goal simpleInterpolation(const snapstack_msgs::Goal& current,
                                                  const geometry_msgs::Vector3& dest_pos, double dest_yaw, double vel,
                                                  double vel_yaw, double dist_thresh, double yaw_thresh, double dt,
                                                  bool& finished);
  static snapstack_msgs::Goal simpleInterpolation(const snapstack_msgs::Goal& current,
                                                  const snapstack_msgs::Goal& dest_pos, double dest_yaw, double vel,
                                                  double vel_yaw, double dist_thresh, double yaw_thresh, double dt,
                                                  bool& finished);
  static double quat2yaw(const geometry_msgs::Quaternion& q);
  static double saturate(double val, double low, double high);
  static double wrap(double val);

  // ROS
  ros::NodeHandle nh_, nhp_;
  ros::Subscriber subs_mode_;   // "flightmode" subscriber
  ros::Subscriber subs_state_;  // "state" subscriber
  ros::Publisher pub_goal_;     // "goal" publisher
  ros::Timer pub_timer_;        // timer for pub_goal

  // Trajectory
  std::unique_ptr<Trajectory> traj_;

  std::string veh_name_;  // vehicle name
  enum FlightMode
  {
    GROUND,
    TAKING_OFF,
    HOVERING,
    INIT_POS_TRAJ,
    TRAJ_FOLLOWING,
    LANDING,
    INIT_POS
  };
  FlightMode flight_mode_;
  geometry_msgs::Pose pose_;
  snapstack_msgs::Goal goal_;

  double alt_;  // altitude in m where to take off, and set the traj alt to this too
  double dt_;   // goal publication period [s], and set the traj dt to this too

  std::vector<snapstack_msgs::Goal> traj_goals_;       // vector of trajectory goals currently being followed
  std::vector<snapstack_msgs::Goal> traj_goals_full_;  // goals for all the maneuver: circles with v = v_goals_
  std::unordered_map<int, std::string> index_msgs_;
  std::unordered_map<int, std::string> index_msgs_full_;
  int pub_index_;  // current index in traj_goals_ vector

  double vel_initpos_, vel_take_, vel_land_fast_, vel_land_slow_, vel_yaw_;  // interpolation vels
  double dist_thresh_, yaw_thresh_;
  double margin_takeoff_outside_bounds_;
  double xmin_, xmax_, ymin_, ymax_, zmin_, zmax_;  // safety bouds

  geometry_msgs::Vector3 init_pos_;  // pos where we took off and where we will land
                                     // z component is the initial altitude of the robot before take off, "ground level"
};

}  // namespace trajectory_generator
