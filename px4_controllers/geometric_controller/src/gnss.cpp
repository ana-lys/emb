#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/trajectory_helper.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <controller_msgs/PositionCommand.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define latA 21.0065177
#define longA 105.8429565

#define latB 21.006523
#define longB 105.8431188

#define latC 21.0065228
#define longC 105.8432823

#define latD 21.0066968
#define longD 105.8432903

#define latE 21.0066886
#define longE 105.8432813

double maxj = 5.0 , maxa = 4.0 , maxv = 2.5 , maxav = 1.2 , maxaa = 2.0;
int sample_idx=0;
bool gps_home_init = false, odom_init = false , setmode_init =false ,plan_init = false ,plan_fin =false;
Eigen::Quaterniond mav_att ;
double mav_yaw = 0,mav_yawvel = 0;
Eigen::Vector3d gps_home,gpsraw,local_start,mav_pos,mav_vel;
std::vector<Eigen::Vector3d> gps_target,local_setpoint;
double UTM_X,UTM_Y;
double UTM_SP_X,UTM_SP_Y;

double ToEulerYaw(const Eigen::Quaterniond& q){
    Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

void FastYaw(const double& start, double& end){
  double yawdiff = end - start;
  while(fabs(yawdiff)> fabs(yawdiff-2*MathPI)){
    end -= 2*MathPI;
    yawdiff = end - start;
  }
  while(fabs(yawdiff)> fabs(yawdiff+2*MathPI)){
    end += 2*MathPI;
    yawdiff = end - start;
  }
}

void odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_att.w()=odomMsg.pose.pose.orientation.w;
  mav_att.x()=odomMsg.pose.pose.orientation.x;
  mav_att.y()=odomMsg.pose.pose.orientation.y;
  mav_att.z()=odomMsg.pose.pose.orientation.z;
  mav_yaw = ToEulerYaw(mav_att);
  mav_yawvel = (mav_att * toEigen(odomMsg.twist.twist.angular))(2);
  mav_pos = toEigen(odomMsg.pose.pose.position);
  mav_vel = mav_att * toEigen(odomMsg.twist.twist.linear);
  odom_init = true;
}
double distance( Eigen::Vector3d cur , Eigen::Vector3d fin) {
  Eigen::Vector3d rm = cur-fin;
  return (cur-fin).norm();
}
void gpsrawCallback(const sensor_msgs::NavSatFix &msg){
 
 if(!gps_home_init && odom_init){
 local_start = mav_pos;
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),48,UTM_X,UTM_Y);//32 zurich 48 VietNam
 for(auto target : gps_target){
    LatLonToUTMXY(target(0),target(1),48,UTM_SP_X,UTM_SP_Y);
    Eigen::Vector3d setpoint;
    setpoint(0) = mav_pos(0) - UTM_X + UTM_SP_X;
    setpoint(1) = mav_pos(1) - UTM_Y + UTM_SP_Y;
    setpoint(2) = target(2);
    // setpoint(3) = target(3);
    local_setpoint.push_back(setpoint);
 }
 gps_home_init = true; 
 for(auto lsp: local_setpoint){
  ROS_INFO_STREAM("local_setpoint" << lsp(0) << " " << lsp(1));
 }
 
 }
}

int main(int argc, char **argv) {
  // for(int i = 1; i < argc ; i++){ 
    switch(argv[1][0]){
      case 'A': {
        gps_target.push_back(Eigen::Vector3d(latA,longA,5.0));
        break;
      }

      case 'B': {
        gps_target.push_back(Eigen::Vector3d(latB,longB,5.0));
        break;
      }

      case 'C': {
        gps_target.push_back(Eigen::Vector3d(latC,longC,5.0));
        break;
      }

      case 'D': {
        gps_target.push_back(Eigen::Vector3d(latD,longD,5.0));
        break;
      }

      case 'E': {
        gps_target.push_back(Eigen::Vector3d(latE,longE,5.0));
        break;
      }

      default:{
        ROS_ERROR("Invalid command");
        return 0;
      }
    // }

  }
 
  int argc_ = 0;
  char ** argv_;

  ros::init(argc_, argv_, "controller_gps");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/global_position/global", 1, &gpsrawCallback, ros::TransportHints().tcpNoDelay());
  ros::Publisher pos_cmd = n.advertise<controller_msgs::PositionCommand>("/controller/pos_cmd",1);
  ros::ServiceClient setModeClient = n.serviceClient<geometric_controller::setmode>("/controller/set_mode");
  geometric_controller::setmode setModeCall;
  
  std::vector<Eigen::VectorXd> Rp,Rv,Ra;
  std::vector<double> sampling_times;
  while(ros::ok()){
    loop_rate.sleep();
    if(!setmode_init && gps_home_init){
      setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
      setModeCall.request.timeout = 50;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success)
      setmode_init = true;
    }
    if(!plan_init && gps_home_init && setmode_init){
        PlannerHelper plan(mav_pos,mav_vel,mav_yaw,mav_yawvel);
        plan.setConstraints(maxv,maxa,maxav,maxaa);
        std::vector<Eigen::VectorXd> pose, twist;
        Eigen::VectorXd temp,tempv;
        temp.resize(3);
        tempv.resize(3);
        for (auto sp : local_setpoint){
          temp << sp;
          tempv << Eigen::Vector3d::Zero();
          pose.push_back(temp);
          twist.push_back(tempv);
        }
        mav_trajectory_generation::Trajectory trajectory;
          plan.planMultiTrajectory(pose, twist, &trajectory);
          double t_start = trajectory.getMinTime();
          double t_end = trajectory.getMaxTime();
          ROS_INFO_STREAM("TIME: " << t_start <<" " << t_end);
          double dt = 0.01;
          trajectory.evaluateRange(t_start, t_end, dt, 0, &Rp, &sampling_times);
          trajectory.evaluateRange(t_start, t_end, dt, 1, &Rv, &sampling_times);
          trajectory.evaluateRange(t_start, t_end, dt, 2, &Ra, &sampling_times);
          plan_init = true;}
      if(plan_init && !plan_fin){
        
        controller_msgs::PositionCommand msg;
        msg.position = toGeometry_msgs(Rp[sample_idx].head(3));
        msg.velocity = toVector3(Rv[sample_idx].head(3));
        msg.acceleration = toVector3(Ra[sample_idx].head(3));
        msg.yaw = 0.0;
        msg.yaw_dot = 0.0;
        msg.trajectory_flag = msg.TRAJECTORY_STATUS_READY;
        pos_cmd.publish(msg);
        sample_idx++;
        if(sample_idx==sampling_times.size())
        plan_fin = true;
      }
      if(plan_fin){
        setModeCall.request.mode = setModeCall.request.HOLD;
        setModeCall.request.timeout = 50;
        setModeClient.call(setModeCall);
        if(setModeCall.response.success);
        ros::shutdown();
      }
    ros::spinOnce();
  }
   
   return 0;
 }