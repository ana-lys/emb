#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define latA 21.0064962
#define longA 105.8429093

#define latB 21.0064962
#define longB 105.8430814

#define latC 21.0064956
#define longC 105.8432475

double maxa = 0.8 , maxv = 2.5;

bool gps_home_init = false, odom_init = false , setmode_init =false;

double maxvRiseDistance( Eigen::Vector3d vel_){
  double velnorm = vel_.norm();
  double maxvRiseTime = velnorm /maxa;
  return -maxvRiseTime * maxvRiseTime * maxa *0.5 + maxvRiseTime * velnorm;
}

Eigen::Vector3d gps_home,gpsraw,gps_target,local_setpoint,local_start,mav_pose,mav_vel;
double UTM_X,UTM_Y;
double UTM_SP_X,UTM_SP_Y;
Eigen::Vector3d acc = Eigen::Vector3d::Zero(),vel=Eigen::Vector3d::Zero(),pos;
Eigen::Vector3d remain,direction;

void odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_pose = toEigen(odomMsg.pose.pose.position);
  mav_vel = toEigen(odomMsg.twist.twist.linear);
  odom_init = true;
}
double distance( Eigen::Vector3d cur , Eigen::Vector3d fin) {
  Eigen::Vector3d rm = cur-fin;
  // ROS_INFO_STREAM(rm(0)<<" "<<rm(1)<<" "<<rm(2));
  return (cur-fin).norm();
}
void gpsrawCallback(const sensor_msgs::NavSatFix &msg){
 
 if(!gps_home_init && odom_init){
 local_start = mav_pose;
 pos = local_start;
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gps_target(0),gps_target(1),48,UTM_SP_X,UTM_SP_Y);
 LatLonToUTMXY(gpsraw(0),gpsraw(1),48,UTM_X,UTM_Y); //32 zurich 48 VietNam
 local_setpoint(0) = mav_pose(0) - UTM_X + UTM_SP_X;
 local_setpoint(1) = mav_pose(1) - UTM_Y + UTM_SP_Y;
 local_setpoint(2) = 5.0;
 gps_home_init = true; 
 ROS_INFO_STREAM("local_setpoint" << local_setpoint(0) << " " << local_setpoint(1));
 }
}
int main(int argc, char **argv) {
  if(argv[1][0]=='A'){
    gps_target(0)=latA;
    gps_target(1)=longA;
    gps_target(2)=5.0;
  }
  else if (argv[1][0]=='B'){
    gps_target(0)=latB;
    gps_target(1)=longB;
    gps_target(2)=5.0;
  }
  else if (argv[1][0]=='C'){
    gps_target(0)=latC;
    gps_target(1)=longC;
    gps_target(2)=5.0;
  }
  else {
    return 0;
  }
 
  int argc_ = 0;
  char ** argv_;

  ros::init(argc_, argv_, "controller_gps");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/global_position/global", 1, &gpsrawCallback, ros::TransportHints().tcpNoDelay());
  ros::Publisher flat = n.advertise<controller_msgs::FlatTarget>("/controller/flatsetpoint",1);
  ros::ServiceClient setModeClient = n.serviceClient<geometric_controller::setmode>("/controller/set_mode");
  geometric_controller::setmode setModeCall;
  while(ros::ok()){
    loop_rate.sleep();
    if(!setmode_init && gps_home_init){
      setModeCall.request.mode = setModeCall.request.MISSION_EXECUTION;
      setModeCall.request.timeout = 50;
      setModeClient.call(setModeCall);
      if(setModeCall.response.success)
      setmode_init = true;
    }
    if(gps_home_init && setmode_init){
      remain = local_setpoint - pos;
      direction = remain.normalized();
      if(distance(local_setpoint , pos) > maxvRiseDistance(vel)){
        vel += direction * maxa * 0.01;
        if(vel.norm() > maxv)
        vel = (maxv / vel.norm()) * vel;
        pos += vel *0.01;
      }
      else if(distance(local_setpoint , pos) < maxvRiseDistance(vel) ){
        if(distance(local_setpoint , pos) > 0.4){
          vel-= direction * maxa * 0.01;
          pos+= vel * 0.01;
          if(vel.norm()< 0.2){
            vel = Eigen::Vector3d::Zero(); 
            pos = local_setpoint;
          }
        }
        else{
          vel = Eigen::Vector3d::Zero();
          pos = local_setpoint;
        }
      }
      ROS_INFO_STREAM(distance(local_setpoint , mav_pose));
      if(distance(local_setpoint , mav_pose) < 0.2){
        setModeCall.request.mode = setModeCall.request.HOLD;
        setModeCall.request.timeout = 50;
        setModeClient.call(setModeCall);
        if(setModeCall.response.success)
        ros::shutdown();
      }
      controller_msgs::FlatTarget msg;
      msg.type_mask = 0;
      msg.position = toVector3(pos);
      msg.velocity = toVector3(vel);
      flat.publish(msg);
    }
    ros::spinOnce();
  }
   
   return 0;
 }