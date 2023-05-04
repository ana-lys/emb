#ifndef MAV_TRAJECTORY_GENERATION_HELPER_H
#define MAV_TRAJECTORY_GENERATION_HELPER_H

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>

class PlannerHelper {
 public:
  PlannerHelper(const Eigen::Vector3d& curr_pos ,const Eigen::Vector3d& curr_vel ,const double& curr_yaw ,const double& curr_yawvel);
  void setConstraints(const double& maxv,const double& maxa,const double& maxav,const double& maxaa);
  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planMultiTrajectory(const std::vector<Eigen::VectorXd>& goal_pos,
                      const std::vector<Eigen::VectorXd>& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);

  bool planMultiTrajectory(const std::vector<Eigen::VectorXd>& goal_pos,
                      const std::vector<Eigen::VectorXd>& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
  bool publishPoint(const Eigen::VectorXd& point);
 private:
  ros::Publisher pub_trajectory_;

  Eigen::Vector3d current_pose_;
  Eigen::Vector3d current_velocity_;
  double current_yaw_;
  double current_yawvel_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;

};
PlannerHelper::PlannerHelper(const Eigen::Vector3d& curr_pos ,const Eigen::Vector3d& curr_vel ,const double& curr_yaw ,const double& curr_yawvel){
    current_pose_ = curr_pos;
    current_velocity_ = curr_vel;
    current_yaw_ = curr_yaw;
    current_yawvel_ = curr_yawvel;
}

void PlannerHelper::setConstraints(const double& maxv,const double& maxa,const double& maxav,const double& maxaa){
    max_v_ = maxv;
    max_a_ = maxa;
    max_ang_v_ = maxav;
    max_ang_a_ = maxaa;
}

bool PlannerHelper::planMultiTrajectory(
    const std::vector<Eigen::VectorXd>& goal_pos, const std::vector<Eigen::VectorXd>& goal_vel,
    mav_trajectory_generation::Trajectory* trajectory) {
  assert(trajectory);
  trajectory->clear();

  // 3 Dimensional trajectory => 3D position
  // 4 Dimensional trajectory => 3D position + yaw
  const int dimension = goal_pos[0].size();
  bool success = false;
  if (dimension == 4) 
  {
    mav_trajectory_generation::Trajectory trajectory_trans, trajectory_rot;

    std::vector<Eigen::Matrix<double, -1, 1>> goal_position, goal_lin_vel, goal_rotation, goal_ang_vel;
    for (auto p : goal_pos) {
        Eigen::Matrix<double, -1, 1> pos(3, 1);
        pos << p.head(3);
        goal_position.push_back(pos);
        Eigen::Matrix<double, -1, 1> rot(1, 1);
        rot << p.tail(1);
        goal_rotation.push_back(rot);
    }
    for (auto v : goal_vel) {
        Eigen::Matrix<double, -1, 1> lin_vel(3, 1);
        lin_vel << v.head(3);
        goal_lin_vel.push_back(lin_vel);
        Eigen::Matrix<double, -1, 1> ang_vel(1, 1);
        ang_vel << v.tail(1);
        goal_ang_vel.push_back(ang_vel);
    }
    
    success = planMultiTrajectory(
      goal_position,goal_lin_vel,current_pose_,current_velocity_,max_v_,max_a_,&trajectory_trans);
    ROS_INFO("translation_finished");
    // Rotation trajectory.
    Eigen::Matrix<double, -1, 1> current_rot_vec(1, 1);
    Eigen::Matrix<double, -1, 1> current_angular_velocity_(1, 1);
    current_rot_vec << current_yaw_;
    current_angular_velocity_ << current_yawvel_;
    success &= planMultiTrajectory(
        goal_rotation, goal_ang_vel, current_rot_vec, current_angular_velocity_,
        max_ang_v_, max_ang_a_, &trajectory_rot);
    ROS_INFO("yaw trajectory finished");
    // Combine trajectories.
    success &= trajectory_trans.getTrajectoryWithAppendedDimension(
            trajectory_rot, &(*trajectory));
    return success;
  } 
  else if (dimension == 3) 
  { 
    std::vector<Eigen::Matrix<double, -1, 1>> goal_position, goal_lin_vel, goal_rotation, goal_ang_vel;
    for (auto p : goal_pos) {
      Eigen::Matrix<double, -1, 1> pos(3, 1);
      pos << p.head(3);
      goal_position.push_back(pos);
    }
    for (auto v : goal_vel) {
        Eigen::Matrix<double, -1, 1> lin_vel(3, 1);
        lin_vel << v.head(3);
        goal_lin_vel.push_back(lin_vel);
    }
    success = planMultiTrajectory(
        goal_position, goal_lin_vel, current_pose_, current_velocity_,
        max_v_, max_a_, &(*trajectory));
    return success;
  } 
  else 
  {
    ROS_ERROR("Dimension must be 3, 4 to be valid.");
    return false;
  }
}

// Plans a trajectory from a start position and velocity to a goal position and velocity
bool PlannerHelper::planMultiTrajectory(const std::vector<Eigen::VectorXd>& goal_pos,
                      const std::vector<Eigen::VectorXd>& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory){
  assert(trajectory);
  const int dimension = goal_pos[0].size();
  // Array for all waypoints and their constraints
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // start = desired start vector
  // end = desired end vector
  mav_trajectory_generation::Vertex start(dimension), end(dimension);

  /******* Configure start point *******/
  ROS_INFO_STREAM(start_pos);
  start.makeStartOrEnd(start_pos, derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel);
  vertices.push_back(start);
  if(goal_pos.size()>1)
  for(int i = 0 ; i <goal_pos.size()-1; ++i){
  mav_trajectory_generation::Vertex middle(dimension);
  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,goal_pos[i]);
  middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,goal_vel[i]);
  vertices.push_back(middle);
  }
  /******* Configure end point *******/
  // set end point constraints to desired position and set all derivatives to zero
  end.makeStartOrEnd(goal_pos.back(), derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                    goal_vel.back());
  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, v_max, a_max);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  trajectory->scaleSegmentTimesToMeetConstraints(v_max, a_max);
  
  return true;
}
#endif // MAV_TRAJECTORY_GENERATION_HELPER_H