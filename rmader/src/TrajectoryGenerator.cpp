/**
 * @file TrajectoryGenerator.cpp
 * @brief Trajectory Generator class
 * @author Aleix Paris
 * @date 2020-01-08
 */

#include "TrajectoryGenerator.hpp"
#include "trajectories/Circle.hpp"
#include "trajectories/Line.hpp"

/*
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <eigen3/Eigen/Eigen>*/

namespace trajectory_generator
{
TrajectoryGenerator::TrajectoryGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nhp) : nh_(nh), nhp_(nhp)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // Check namespace to find name of vehicle
  veh_name_ = ros::this_node::getNamespace();
  size_t n = veh_name_.find_first_not_of('/');
  veh_name_.erase(0, n);  // remove leading slashes
  if (veh_name_.empty())
  {
    ROS_ERROR("Error :: You should be using a launch file to specify the "
              "node namespace!\n");
    ros::shutdown();
    return;
  }
  ROS_INFO_STREAM("veh_name_ = " << veh_name_);

  traj_->generateTraj(traj_goals_full_, index_msgs_full_);
  traj_goals_ = traj_goals_full_;
  index_msgs_ = index_msgs_full_;

  subs_mode_ = nh_.subscribe("/globalflightmode", 1, &TrajectoryGenerator::modeCB, this);
  subs_state_ = nh_.subscribe("state", 1, &TrajectoryGenerator::stateCB, this);
  pub_timer_ = nh_.createTimer(ros::Duration(dt_), &TrajectoryGenerator::pubCB, this);
  pub_goal_ = nh_.advertise<snapstack_msgs::Goal>("goal", 1, false);  // topic, queue_size, latch

  ros::Duration(1.0).sleep();  // to ensure that the state has been received

  flight_mode_ = GROUND;

  // Init goal (and update with current pos every time that the mode switches to GROUND)
  resetGoal();
  goal_.p.x = pose_.position.x;
  goal_.p.y = pose_.position.y;
  goal_.p.z = pose_.position.z;

  ROS_INFO("Successfully launched trajectory generator node.");
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

bool TrajectoryGenerator::readParameters()
{
  if (!nhp_.getParam("alt", alt_))
    return false;
  double freq;
  if (!nhp_.getParam("pub_freq", freq))
    return false;
  dt_ = 1.0 / freq;

  std::string traj_type;
  if (!nhp_.getParam("traj_type", traj_type))
    return false;
  if (traj_type == "Circle")
  {
    // circular trajectory parameters
    double r, cx, cy, t_traj, circle_accel;
    std::vector<double> v_goals;
    if (!nhp_.getParam("r", r))
      return false;
    if (!nhp_.getParam("center_x", cx))
      return false;
    if (!nhp_.getParam("center_y", cy))
      return false;
    if (!nhp_.getParam("v_goals", v_goals))
      return false;
    for (double vel : v_goals)
    {
      if (vel <= 0)
      {
        ROS_ERROR("All velocities must be > 0");
        return false;
      }
    }
    if (!nhp_.getParam("t_traj", t_traj))
      return false;
    if (!nhp_.getParam("circle_accel", circle_accel))
      return false;
    if (circle_accel <= 0)
    {
      ROS_ERROR("accel must be > 0");
      return false;
    }
    traj_ = std::make_unique<Circle>(alt_, r, cx, cy, v_goals, t_traj, circle_accel, dt_);
  }
  else if (traj_type == "Line")
  {
    // line trajectory parameters
    double Ax, Ay, Bx, By, v_line, a1, a3;
    if (!nhp_.getParam("Ax", Ax))
      return false;
    if (!nhp_.getParam("Ay", Ay))
      return false;
    if (!nhp_.getParam("Bx", Bx))
      return false;
    if (!nhp_.getParam("By", By))
      return false;
    if (!nhp_.getParam("v_line", v_line))
      return false;
    if (v_line <= 0)
    {
      ROS_ERROR("The velocity must be > 0");
      return false;
    }
    if (!nhp_.getParam("By", By))
      return false;
    if (!nhp_.getParam("By", By))
      return false;
    if (!nhp_.getParam("line_accel", a1))
      return false;
    if (!nhp_.getParam("line_decel", a3))
      return false;
    if (a1 <= 0 or a3 <= 0)
    {
      ROS_ERROR("accel and decel must be > 0");
      return false;
    }
    std::vector<double> v_goals;
    v_goals.push_back(v_line);
    traj_ = std::make_unique<Line>(alt_, Eigen::Vector3d(Ax, Ay, alt_), Eigen::Vector3d(Bx, By, alt_), v_goals, a1, a3,
                                   dt_);
  }
  else
  {
    ROS_ERROR("Trajectory type not valid.");
    return false;
  }

  // other params
  if (!nhp_.getParam("vel_initpos", vel_initpos_))
    return false;
  if (!nhp_.getParam("vel_take", vel_take_))
    return false;
  if (!nhp_.getParam("vel_land_fast", vel_land_fast_))
    return false;
  if (!nhp_.getParam("vel_land_slow", vel_land_slow_))
    return false;
  if (!nhp_.getParam("vel_yaw", vel_yaw_))
    return false;

  if (!nhp_.getParam("dist_thresh", dist_thresh_))
    return false;
  if (!nhp_.getParam("yaw_thresh", yaw_thresh_))
    return false;

  if (!nhp_.getParam("margin_takeoff_outside_bounds", margin_takeoff_outside_bounds_))
    return false;

  // bounds
  if (!nhp_.getParam("/room_bounds/x_min", xmin_))
    return false;
  if (!nhp_.getParam("/room_bounds/x_max", xmax_))
    return false;
  if (!nhp_.getParam("/room_bounds/y_min", ymin_))
    return false;
  if (!nhp_.getParam("/room_bounds/y_max", ymax_))
    return false;
  if (!nhp_.getParam("/room_bounds/z_min", zmin_))
    return false;
  if (!nhp_.getParam("/room_bounds/z_max", zmax_))
    return false;

  // check that the trajectory params don't conflict with the bounds
  if (!traj_->trajectoryInsideBounds(xmin_, xmax_, ymin_, ymax_, zmin_, zmax_))
  {
    ROS_ERROR("The trajectory parameters conflict with the room bounds.");
    return false;
  }

  return true;
}

void TrajectoryGenerator::modeCB(const snapstack_msgs::QuadFlightMode& msg)
{
  // FSM transitions:
  // Any state        --ESTOP--> [kill motors] and switch to ground mode
  // On the ground    --START--> Take off and then hover
  // Hovering         --START--> Go to init pos of the trajectory
  // Init pos of traj --START--> follow the generated trajectory
  // Init pos of traj --END--> switch to hovering where the drone currently is
  // Traj following   --END--> change the traj goals vector to a braking trajectory and then switch to hovering
  // Hovering         --END--> Go to the initial position, land, and then switch to ground mode

  // Behavior selector button to mode mapping
  // START -> GO   (4)
  // END   -> LAND (2)
  // ESTOP -> KILL (6)
  if (msg.mode == msg.KILL)
  {
    goal_.power = false;
    goal_.header.stamp = ros::Time::now();
    pub_goal_.publish(goal_);
    flight_mode_ = GROUND;
    resetGoal();
    ROS_INFO("Motors killed, switched to GROUND mode.");
    return;
  }
  else if (flight_mode_ == GROUND and msg.mode == msg.GO)
  {
    // Check inside safety bounds, and don't let the takeoff happen if outside them
    double xmin = xmin_ - margin_takeoff_outside_bounds_;
    double ymin = ymin_ - margin_takeoff_outside_bounds_;
    double xmax = xmax_ + margin_takeoff_outside_bounds_;
    double ymax = ymax_ + margin_takeoff_outside_bounds_;
    if (pose_.position.x < xmin or pose_.position.x > xmax or pose_.position.y < ymin or pose_.position.y > ymax)
    {
      ROS_WARN("Can't take off: the vehicle is outside the safety bounds.");
      return;
    }

    // Takeoff initializations
    init_pos_.x = pose_.position.x;
    init_pos_.y = pose_.position.y;
    init_pos_.z = pose_.position.z;
    // set the goal to our current position + yaw
    resetGoal();
    goal_.p.x = init_pos_.x;
    goal_.p.y = init_pos_.y;
    goal_.p.z = init_pos_.z;
    goal_.psi = quat2yaw(pose_.orientation);

    // Take off
    flight_mode_ = TAKING_OFF;
    ROS_INFO("Taking off...");
    // then it will switch automatically to HOVERING

    // switch on motors after flight_mode changes, to avoid timer callback setting power to false
    goal_.power = true;
  }
  else if (flight_mode_ == HOVERING and msg.mode == msg.GO)
  {
    traj_goals_ = traj_goals_full_;
    index_msgs_ = index_msgs_full_;
    flight_mode_ = INIT_POS_TRAJ;
    ROS_INFO("Going to the initial position of the generated trajectory...");
  }
  else if (flight_mode_ == INIT_POS_TRAJ and msg.mode == msg.GO)
  {
    // Start following the generated trajectory if close to the init pos (in 2D)
    double dist_to_init =
        sqrt(pow(traj_goals_[0].p.x - pose_.position.x, 2) + pow(traj_goals_[0].p.y - pose_.position.y, 2));
    double delta_yaw = traj_goals_[0].psi - quat2yaw(pose_.orientation);
    delta_yaw = wrap(delta_yaw);
    if (dist_to_init > dist_thresh_ or fabs(delta_yaw) > yaw_thresh_)
    {
      ROS_INFO("Can't switch to the generated trajectory following mode, too far from the init pos");
      return;
    }
    pub_index_ = 0;
    flight_mode_ = TRAJ_FOLLOWING;
    ROS_INFO("Following the generated trajectory...");
  }
  else if (flight_mode_ == INIT_POS_TRAJ and msg.mode == msg.LAND)
  {
    // Change mode to hover wherever the robot was when we clicked "END"
    // Need to send a current goal with 0 vel bc we could be moving to the init pos of traj
    resetGoal();
    goal_.p.x = pose_.position.x;
    goal_.p.y = pose_.position.y;
    goal_.p.z = alt_;
    goal_.psi = quat2yaw(pose_.orientation);
    goal_.header.stamp = ros::Time::now();
    pub_goal_.publish(goal_);
    flight_mode_ = HOVERING;
    ROS_INFO("Switched to HOVERING mode");
  }
  else if (flight_mode_ == TRAJ_FOLLOWING and msg.mode == msg.LAND)
  {
    // Generate a braking trajectory. Then, we will automatically switch to hover when done
    traj_->generateStopTraj(traj_goals_, index_msgs_, pub_index_);
  }
  else if (flight_mode_ == HOVERING and msg.mode == msg.LAND)
  {
    // go to the initial position
    flight_mode_ = INIT_POS;
    ROS_INFO("Switched to INIT_POS mode");
  }
}

void TrajectoryGenerator::pubCB(const ros::TimerEvent& event)
{
  // Always publish a goal to avoid ramps in comm_monitor
  if (flight_mode_ == GROUND)
    goal_.power = false;  // not needed but just in case, for safety

  // if taking off, increase alt until we reach
  else if (flight_mode_ == TAKING_OFF)
  {
    // TODO: spinup time

    double takeoff_alt = alt_;  // don't add init alt bc the traj is generated with z = alt_
    // if close to the takeoff_alt, switch to HOVERING
    if (fabs(takeoff_alt - pose_.position.z) < 0.10 and goal_.p.z >= takeoff_alt)
    {
      flight_mode_ = HOVERING;
      ROS_INFO("Take off completed");
    }
    else
    {
      // Increment the z cmd each timestep for a smooth takeoff.
      // This is essentially saturating tracking error so actuation is low.
      goal_.p.z = saturate(goal_.p.z + vel_take_ * dt_, 0.0, takeoff_alt);
    }
  }
  // else if(flight_mode_ == HOVERING) <- just publish current goal
  else if (flight_mode_ == INIT_POS_TRAJ)
  {
    bool finished;
    goal_ = simpleInterpolation(goal_, traj_goals_[0], traj_goals_[0].psi, vel_initpos_, vel_yaw_, dist_thresh_,
                                yaw_thresh_, dt_, finished);
    // if finished, switch to traj following mode? No, prefer to choose when
  }
  else if (flight_mode_ == TRAJ_FOLLOWING)
  {
    goal_ = traj_goals_[pub_index_];
    if (index_msgs_.find(pub_index_) != index_msgs_.end())
    {
      ROS_INFO("%s", index_msgs_[pub_index_].c_str());
    }
    ++pub_index_;
    if (pub_index_ == traj_goals_.size())
    {
      // Switch to HOVERING mode after finishing the trajectory
      resetGoal();
      goal_.p.x = pose_.position.x;
      goal_.p.y = pose_.position.y;
      goal_.p.z = alt_;
      goal_.psi = quat2yaw(pose_.orientation);
      pub_goal_.publish(goal_);
      flight_mode_ = HOVERING;
      ROS_INFO("Trajectory finished. Switched to HOVERING mode");
    }
  }
  else if (flight_mode_ == INIT_POS)
  {
    // go to init_pos_ but with altitude alt_ and current yaw
    bool finished;
    geometry_msgs::Vector3 dest = init_pos_;
    dest.z = alt_;
    goal_ =
        simpleInterpolation(goal_, dest, goal_.psi, vel_initpos_, vel_yaw_, dist_thresh_, yaw_thresh_, dt_, finished);
    if (finished)
    {  // land when close to the init pos
      flight_mode_ = LANDING;
      ROS_INFO("Landing...");
    }
  }
  // if landing, decrease alt until we reach ground (and switch to ground)
  // The goal was already set to our current position + yaw when hovering
  else if (flight_mode_ == LANDING)
  {
    // choose between fast and slow landing
    double vel_land = pose_.position.z > (init_pos_.z + 0.4) ? vel_land_fast_ : vel_land_slow_;
    goal_.p.z = goal_.p.z - vel_land * dt_;

    if (goal_.p.z < 0)
    {  // don't use init alt here. It's safer to try to land to the ground
      // landed, kill motors
      goal_.power = false;
      flight_mode_ = GROUND;
      ROS_INFO("Landed");
    }
  }

  // apply safety bounds
  goal_.p.x = saturate(goal_.p.x, xmin_, xmax_);  // val, low, high
  goal_.p.y = saturate(goal_.p.y, ymin_, ymax_);
  goal_.p.z = saturate(goal_.p.z, zmin_, zmax_);

  goal_.header.stamp = ros::Time::now();  // set current time

  // Goals should only be published here because this is the only place where we
  // apply safety bounds. Exceptions: when killing the drone and when clicking END at init pos traj
  pub_goal_.publish(goal_);
}

void TrajectoryGenerator::stateCB(const snapstack_msgs::State& msg)
{
  pose_.position.x = msg.pos.x;
  pose_.position.y = msg.pos.y;
  pose_.position.z = msg.pos.z;
  pose_.orientation = msg.quat;
}

void TrajectoryGenerator::resetGoal()
{
  // Creating a new goal message should already set this correctly, but just in case
  // Exception: yaw would be 0 instead of current yaw
  goal_.p.x = goal_.p.y = goal_.p.z = 0;
  goal_.v.x = goal_.v.y = goal_.v.z = 0;
  goal_.a.x = goal_.a.y = goal_.a.z = 0;
  goal_.j.x = goal_.j.y = goal_.j.z = 0;
  // goal_.s.x = goal_.s.y = goal_.s.z = 0;
  goal_.psi = quat2yaw(pose_.orientation);
  goal_.dpsi = 0;
  //    goal_.power = false;
  // reset_xy_int and  reset_z_int are not used
  goal_.mode_xy = snapstack_msgs::Goal::MODE_POSITION_CONTROL;
  goal_.mode_z = snapstack_msgs::Goal::MODE_POSITION_CONTROL;
}

// Utils
snapstack_msgs::Goal TrajectoryGenerator::simpleInterpolation(const snapstack_msgs::Goal& current,
                                                              const geometry_msgs::Vector3& dest_pos, double dest_yaw,
                                                              double vel, double vel_yaw, double dist_thresh,
                                                              double yaw_thresh, double dt, bool& finished)
{
  snapstack_msgs::Goal goal;
  // interpolate from current goal pos to the initial goal pos
  double Dx = dest_pos.x - current.p.x;
  double Dy = dest_pos.y - current.p.y;
  double dist = sqrt(Dx * Dx + Dy * Dy);

  double delta_yaw = dest_yaw - current.psi;
  delta_yaw = wrap(delta_yaw);

  bool dist_far = dist > dist_thresh;
  bool yaw_far = fabs(delta_yaw) > yaw_thresh;
  finished = not dist_far and not yaw_far;  // both are close

  bool accel_for_vel = 0.1;

  goal.p.z = dest_pos.z;  // this should be alt_ and the altitude where the drone took off too
  // are we too far from the dest?
  if (dist_far)
  {
    double c = Dx / dist;
    double s = Dy / dist;
    goal.p.x = current.p.x + c * vel * dt;
    goal.p.y = current.p.y + s * vel * dt;

    // make the vel ref smooth
    // old lines are:
    // goal.v.x = c*vel;
    // goal.v.y = s*vel;

    goal.v.x = std::min(current.v.x + accel_for_vel * dt, c * vel);
    goal.v.y = std::min(current.v.y + accel_for_vel * dt, s * vel);
  }
  else
  {
    goal.p.x = dest_pos.x;
    goal.p.y = dest_pos.y;

    // make the vel ref smooth
    // old lines are:
    // goal.v.x = 0;
    // goal.v.y = 0;

    goal.v.x = std::max(0.0, current.v.x - accel_for_vel * dt);
    goal.v.y = std::max(0.0, current.v.y - accel_for_vel * dt);
  }
  // is the yaw close enough to the desired?
  if (yaw_far)
  {
    int sgn = delta_yaw >= 0 ? 1 : -1;
    vel_yaw = sgn * vel_yaw;  // ccw or cw, the smallest angle
    goal.psi = current.psi + vel_yaw * dt;
    goal.dpsi = vel_yaw;
  }
  else
  {
    goal.psi = dest_yaw;
    goal.dpsi = 0;
  }

  // Remember to set power
  goal.power = true;

  return goal;
}

// for new snapstack messages
snapstack_msgs::Goal TrajectoryGenerator::simpleInterpolation(const snapstack_msgs::Goal& current,
                                                              const snapstack_msgs::Goal& dest_pos, double dest_yaw,
                                                              double vel, double vel_yaw, double dist_thresh,
                                                              double yaw_thresh, double dt, bool& finished)
{
  snapstack_msgs::Goal goal;
  // interpolate from current goal pos to the initial goal pos
  double Dx = dest_pos.p.x - current.p.x;
  double Dy = dest_pos.p.y - current.p.y;
  double dist = sqrt(Dx * Dx + Dy * Dy);

  double delta_yaw = dest_yaw - current.psi;
  delta_yaw = wrap(delta_yaw);

  bool dist_far = dist > dist_thresh;
  bool yaw_far = fabs(delta_yaw) > yaw_thresh;
  finished = not dist_far and not yaw_far;  // both are close

  bool accel_for_vel = 0.1;

  goal.p.z = dest_pos.p.z;  // this should be alt_ and the altitude where the drone took off too
  // are we too far from the dest?
  if (dist_far)
  {
    double c = Dx / dist;
    double s = Dy / dist;
    goal.p.x = current.p.x + c * vel * dt;
    goal.p.y = current.p.y + s * vel * dt;

    // make the vel ref smooth
    // old lines are:
    // goal.v.x = c*vel;
    // goal.v.y = s*vel;

    goal.v.x = std::min(current.v.x + accel_for_vel * dt, c * vel);
    goal.v.y = std::min(current.v.y + accel_for_vel * dt, s * vel);
  }
  else
  {
    goal.p.x = dest_pos.p.x;
    goal.p.y = dest_pos.p.y;

    // make the vel ref smooth
    // old lines are:
    // goal.v.x = 0;
    // goal.v.y = 0;

    goal.v.x = std::max(0.0, current.v.x - accel_for_vel * dt);
    goal.v.y = std::max(0.0, current.v.y - accel_for_vel * dt);
  }
  // is the yaw close enough to the desired?
  if (yaw_far)
  {
    int sgn = delta_yaw >= 0 ? 1 : -1;
    vel_yaw = sgn * vel_yaw;  // ccw or cw, the smallest angle
    goal.psi = current.psi + vel_yaw * dt;
    goal.dpsi = vel_yaw;
  }
  else
  {
    goal.psi = dest_yaw;
    goal.dpsi = 0;
  }

  // Remember to set power
  goal.power = true;

  return goal;
}

double TrajectoryGenerator::quat2yaw(const geometry_msgs::Quaternion& q)
{
  double yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  return yaw;
}

double TrajectoryGenerator::saturate(double val, double low, double high)
{
  if (val > high)
    val = high;
  else if (val < low)
    val = low;

  return val;
}

double TrajectoryGenerator::wrap(double val)
{
  if (val > M_PI)
    val -= 2.0 * M_PI;
  if (val < -M_PI)
    val += 2.0 * M_PI;
  return val;
}

}  // namespace trajectory_generator
