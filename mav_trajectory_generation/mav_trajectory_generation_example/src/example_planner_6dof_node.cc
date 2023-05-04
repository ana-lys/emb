/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>
#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  ExamplePlanner planner(n);

  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  std::vector<Eigen::VectorXd> pose, twist;
  Eigen::Vector3d position, rotation_vec;
  position << 0.0, 1.0, 2.0;
  Eigen::VectorXd temp,tempv;
  temp.resize(3);
  temp<<Eigen::Vector3d(2,0,2);
  pose.push_back(temp);
  tempv.resize(3);
  tempv<<Eigen::Vector3d(0,1,0);
  twist.push_back(tempv);

  temp.head(3)= Eigen::Vector3d(2,2,2);
  pose.push_back(temp);
  tempv.resize(3);
  tempv.head(3)= Eigen::Vector3d(-1,0,0);
  twist.push_back(tempv);

  temp.head(3)= Eigen::Vector3d(0,2,2);
  pose.push_back(temp);
  tempv.resize(3);
  tempv.head(3)= Eigen::Vector3d(0,-1,0);
  twist.push_back(tempv);

  temp.head(3)= Eigen::Vector3d(0,0,2);
  pose.push_back(temp);
  tempv.resize(3);
  tempv.head(3)= Eigen::Vector3d(0,0,-0.6);
  twist.push_back(tempv);

  temp.head(3)= Eigen::Vector3d(0,0,0);
  pose.push_back(temp);
  tempv.resize(3);
  tempv.head(3)= Eigen::Vector3d(0,0,-0.2);
  twist.push_back(tempv);
  
   

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  // ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++) {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  mav_trajectory_generation::Trajectory trajectory;
  planner.planMultiTrajectory(pose, twist, &trajectory);
  planner.publishTrajectory(trajectory);
  std::cout << trajectory.getMinTime()<< " " <<trajectory.getMaxTime()<<" \n";
  double sampling_time = 2.0;
    int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    double t_start = trajectory.getMinTime();
    double t_end = trajectory.getMaxTime();
    ros::Rate Rdt = 20;
    double dt = 0.1;
    std::vector<Eigen::VectorXd> Rp,Rv,Ra;
    std::vector<double> sampling_times; // Optional.
    trajectory.evaluateRange(t_start, t_end, dt, 0, &Rp, &sampling_times);
    trajectory.evaluateRange(t_start, t_end, dt, 1, &Rv, &sampling_times);
    trajectory.evaluateRange(t_start, t_end, dt, 2, &Ra, &sampling_times);
    for(int i = 0 ; i < Rp.size() ; i++){
    planner.publishPoint(Rp[i]);
    Rdt.sleep();
    }
    ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}