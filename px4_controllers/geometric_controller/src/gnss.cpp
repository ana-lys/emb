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

#define latA 47.3977421
#define longA 8.5455936

#define latB 47.3977423
#define longB 8.5456615

#define latC 47.3977425
#define longC 8.5457287
double maxa = 0.8 , maxv = 2.5;

bool gps_home_init = false, odom_init = false;
double maxvRiseDistance( Eigen::Vector3d vel_){
  double velnorm = vel_.norm();
  double maxvRiseTime = velnorm /maxa;
  return -maxvRiseTime * maxvRiseTime * maxa *0.5 + maxvRiseTime * velnorm;
}

Eigen::Vector3d gps_home,gpsraw,gps_target,local_setpoint,local_start,mav_pose;
double UTM_X,UTM_Y;
double UTM_SP_X,UTM_SP_Y;
Eigen::Vector3d acc = Eigen::Vector3d::Zero(),vel=Eigen::Vector3d::Zero(),pos;
Eigen::Vector3d remain,direction;

void odomCallback(const nav_msgs::Odometry &odomMsg){
  mav_pose(0) = odomMsg.pose.pose.position.x;
  mav_pose(1) = odomMsg.pose.pose.position.y;
  mav_pose(2) = odomMsg.pose.pose.position.z;
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
 LatLonToUTMXY(gps_target(0),gps_target(1),32,UTM_SP_X,UTM_SP_Y);
 LatLonToUTMXY(gpsraw(0),gpsraw(1),32,UTM_X,UTM_Y); //32 zurich 48 VietNam
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
   while(ros::ok()){
   loop_rate.sleep();
   if(gps_home_init){
    remain = local_setpoint - pos;
    direction = remain.normalized();
    // ROS_INFO_STREAM(distance(local_setpoint , pos)<<" "<<maxvRiseDistance(vel));
    if(distance(local_setpoint , pos) > maxvRiseDistance(vel)){
      vel += direction * maxa * 0.01;
      if(vel.norm() > maxv)
      vel = (maxv / vel.norm()) * vel;
      pos += vel *0.01;
      ROS_INFO("outrange");
    }
    else if(distance(local_setpoint , pos) < maxvRiseDistance(vel) ){
      if(distance(local_setpoint , pos) > 0.4)
      {
        vel-= direction * maxa * 0.01;
        pos+= vel * 0.01;
        if(vel.norm()< 0.2){
        vel = Eigen::Vector3d::Zero(); 
        pos = local_setpoint;
        }        ROS_INFO("slowrange");
      }
      else {
      vel = Eigen::Vector3d::Zero();
      pos = local_setpoint;
      ROS_INFO("setprange");
      }
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