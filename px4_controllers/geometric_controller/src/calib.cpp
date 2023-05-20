#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h> // Include the YAML-CPP library
#include <fstream>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "geometric_controller/common.h"
#include "geometric_controller/UTM.h"
#include <mavros_msgs/GPSRAW.h>
#include <sensor_msgs/NavSatFix.h>

#define latX 21.0067289
#define longX 105.8427996

#define latC 21.0065166
#define longC 105.8427789

#define latV 21.0065194
#define longV 105.8429490

#define latB 21.0065096
#define longB 105.8431182

bool gps_home_init = false, odom_init = false, setpoint_init = false;
Eigen::Vector3d gps_home, gpsraw, local_start, mav_pos, mav_vel, offset;
Eigen::Vector3d gps_target, local_setpoint;
double UTM_X, UTM_Y;
double UTM_SP_X, UTM_SP_Y;

void odomCallback(const nav_msgs::Odometry& odomMsg)
{
  mav_pos = toEigen(odomMsg.pose.pose.position);
  odom_init = true;
}

void gpsrawCallback(const sensor_msgs::NavSatFix& msg)
{
  if (setpoint_init && !gps_home_init && odom_init) {
    local_start = mav_pos;
    gpsraw(0) = msg.latitude;
    gpsraw(1) = msg.longitude;
    gpsraw(2) = msg.altitude;
    LatLonToUTMXY(gpsraw(0), gpsraw(1), 48, UTM_X, UTM_Y);           // 32 zurich 48 VietNam
    LatLonToUTMXY(gps_target(0), gps_target(1), 48, UTM_SP_X, UTM_SP_Y);
    Eigen::Vector3d setpoint;
    offset(0) = -UTM_X + UTM_SP_X;
    offset(1) = -UTM_Y + UTM_SP_Y;
    std::string yamlPath = ros::package::getPath("geometric_controller") + "/cfg/gps_calib.yaml";
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "offsetX" << YAML::Value << offset(0);
    emitter << YAML::Key << "offsetY" << YAML::Value << offset(1);
    emitter << YAML::EndMap;
    std::ofstream file(yamlPath, std::ofstream::out | std::ofstream::trunc);
    file << emitter.c_str();
    file.close();
    std::cout << "offset setted to " << offset(0) << " " << offset(1) << std::endl;
    gps_home_init = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calib_node");
  ros::NodeHandle nh("~"); // Use a private node handle for parameters

  for (int i = 1; i < argc; i++) {
    switch (argv[i][0]) {
      case 'X': {
        gps_target(0) = latX;
        gps_target(1) = longX;
        break;
      }
      case 'C': {
        gps_target(0) = latC;
        gps_target(1) = longC;
        break;
      }
      case 'V': {
        gps_target(0) = latV;
        gps_target(1) = longV;
        break;
      }
      case 'B': {
        gps_target(0) = latB;
        gps_target(1) = longB;
        break;
      }
      default:{
        std::cout << "Unknown setpoint";
        return 0;
        break;
      }
    }
  }
  setpoint_init = true;

  ros::Rate loop_rate(20);
  ros::Subscriber odomSub_ = nh.subscribe("/mavros/local_position/odom", 1, &odomCallback,
                                          ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = nh.subscribe("/mavros/global_position/global", 1, &gpsrawCallback,
                                          ros::TransportHints().tcpNoDelay());

  while (ros::ok()) {
    std::cout << "OK";
    ros::spinOnce();
    if (gps_home_init)
      break;
    loop_rate.sleep();
  }

  return 0;
}
