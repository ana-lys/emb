#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <controller_msgs/FlatTarget.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sys/resource.h>
#include <deque>
#include <vector>


class Gauss_pdf{
private: 
double mean = 0 ;
double sigma = 0.75;
constexpr static const double inv_sqrt_2pi = 0.3989422804014327;

public:
Gauss_pdf() {}
Gauss_pdf(double mean_ , double sigma_){
  mean = mean_;
  sigma = sigma_;
}
~Gauss_pdf(){}
void set_param(double mean_ , double sigma_) { mean = mean_; sigma = sigma_; }
double normal_pdf (double x)
{      
    double a = (x - mean) / sigma;

    return inv_sqrt_2pi / sigma * std::exp(-0.5f * a * a);
}
};

class Aruco_event{
private: 
ros::Time Event_start ;
Eigen::Vector3d sigma =  Eigen::Vector3d(1,1,1);
Eigen::Vector3d scalar = Eigen::Vector3d::Zero();
Gauss_pdf function[3];

public:
Aruco_event() {}
Aruco_event(Eigen::Vector3d sigma_, Eigen::Vector3d scalar_){
  Event_start  = ros::Time::now();
  sigma = sigma_;
  scalar = scalar_;
  for(int i =0 ; i < 3 ; i++){
	function[i] = Gauss_pdf(sigma[i]*3,sigma[i]);
  }
}
~Aruco_event(){}
bool check_available(){
	if ((ros::Time::now()-Event_start).toSec()<6*std::max(std::max(sigma(0),sigma(1)),sigma(2))) return true ;
	else return false ;
}
Eigen::Vector3d getscalar ()
{
return scalar;
}
Eigen::Vector3d getfuction_value ()
{
Eigen::Vector3d result;
    for(int i =0 ; i < 3 ; i++){
		result(i) = function[i].normal_pdf((ros::Time::now()-Event_start).toSec())*scalar(i);
	}
return result;
}
Eigen::Vector3d getz_value ()
{
    Eigen::Vector3d zvalue = ((ros::Time::now()-Event_start).toSec()*Eigen::Vector3d(1.0f,1.0f,1.0f)- 3.0 * sigma).cwiseProduct(sigma.cwiseInverse());
    return zvalue;
}
};

class MarkerPlanner
{
private:
	ros::NodeHandle nh;

	/******************************Subscriber***********************************/
	ros::Subscriber aruco_sub,whycon_sub;
	ros::Subscriber odom_sub;

	/******************************Publisher***********************************/
	ros::Publisher pose_pub;
	ros::Publisher controller_pub;

    /******************************Publisher***********************************/
	ros::Timer planner,pose_mk;
	/******************************Variable***********************************/
	Gauss_pdf reference,matching,height_matching;
	bool safezone;
	Eigen::Vector3d discovery = Eigen::Vector3d::Zero();
	Eigen::Vector3d output = Eigen::Vector3d::Zero();
	Eigen::Vector3d safezone_output = Eigen::Vector3d::Zero();
	double zreference[100] = {0};
	double height_coeff=0.4;
    std::deque<Aruco_event> event_dq;
	Eigen::Quaterniond base_quaternion;
	Eigen::Vector3d base_pose;
	double trust_coeff;
	std::vector<geometry_msgs::Point> pose_points;

public:
	/*
	 * Constructor
	 */
	MarkerPlanner() : nh("")
	{	
		aruco_sub = nh.subscribe("/landing/marker_pose/aruco",1,&MarkerPlanner::marker_callback,this);

		whycon_sub = nh.subscribe("/landing/marker_pose/whycon",1,&MarkerPlanner::marker_callback,this);

		odom_sub = nh.subscribe("/mavros/local_position/odom",1, &MarkerPlanner::odom_callback,this);

		pose_pub = nh.advertise<visualization_msgs::Marker>("/pose/array", 1);

		controller_pub = nh.advertise<controller_msgs::FlatTarget>("/reference/flatsetpoint",2);

		planner = nh.createTimer(ros::Duration(0.03), &MarkerPlanner::plannerCallback , this);

		pose_mk = nh.createTimer(ros::Duration(0.03), &MarkerPlanner::rvisualize , this);
	
		double temp = 0;
		reference.set_param(0,1.0);
		for (int i = 0; i < 100; i++) {
			temp += reference.normal_pdf(-3.0+0.06*i)*0.06;
			zreference[i] = temp;  
		}
		safezone = false;
		matching.set_param(0,3.0);
		height_matching.set_param(0,0.2);
	}
	Eigen::Vector3d landing_limmiter(Eigen::Vector3d &raw){
		double limmited =  std::min(std::max(fabs(raw(0))/sin(0.35),fabs(raw(1))/sin(0.65)),22.0);
		return raw + Eigen::Vector3d::UnitZ() * (limmited + 1);
	}
	void marker_callback(const geometry_msgs::Point &pose){
		Eigen::Quaterniond Base_camera = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.70710678118,-0.70710678118,0.0)));
		Eigen::Vector3d M_Cam = toEigen(pose);
		Eigen::Vector3d M_Base  = Base_camera * M_Cam;
		Eigen::Vector3d M_Inertia  = base_quaternion * M_Base;
		Eigen::Vector3d Setpoint;
		
		if(M_Inertia(2) < -2.0){
			Setpoint = landing_limmiter(M_Inertia);
			safezone = false;
		}
		else {
			Setpoint = M_Inertia;
			safezone = true;
		}
		event_execute(Setpoint);
	}

	Eigen::Vector3d getzreference(Eigen::Vector3d zvalue){
		Eigen::Vector3d result;
		zvalue += 3.0 * Eigen::Vector3d(1,1,1);
		zvalue /= 0.06;
		Eigen::Vector3i index (floor(zvalue(0)),floor(zvalue(1)),floor(zvalue(2)));
		for(int i=0 ; i < 3 ; i++) {
		if( index(i) < 0 ) result(i) = 1;
		else if( index(i) > 99 ) result(i) = 0;
		else result(i) = 1 - zreference[index(i)] ;
		}
		return result;
	}
	void plannerCallback(const ros::TimerEvent &event){
		check_event_deque();
		discovery = Eigen::Vector3d::Zero();
		output = Eigen::Vector3d::Zero();

		controller_msgs::FlatTarget control_msg;
		control_msg.header.stamp = ros::Time::now();

		if(event_dq.size()>0)
			for (int i = 0 ; i < event_dq.size();i++){
				discovery += getzreference(event_dq[i].getz_value()).cwiseProduct(event_dq[i].getscalar());
				output += event_dq[i].getfuction_value();
				control_msg.type_mask = 2 ;
				control_msg.velocity.x = output(0);
				control_msg.velocity.y = output(1);
				control_msg.velocity.z = output(2);
			}
		if(safezone){
			control_msg.type_mask = 0;
			control_msg.position.x = safezone_output(0);
			control_msg.position.y = safezone_output(1);
			control_msg.position.z = safezone_output(2);
		}
		
		controller_pub.publish(control_msg);
	}
	int event_execute(Eigen::Vector3d &setpoint){
		Eigen::Vector3d discovarable = setpoint - discovery;
		if(discovarable.hasNaN())
		return 0;
		else if(safezone){
		safezone_output = setpoint + Eigen::Vector3d::UnitZ() * 1.5 + base_pose;
		return 1;
		}
		else{
		Eigen::Vector3d event_sigma = Eigen::Vector3d(4.0,4.0,6.0) - 18 * Eigen::Vector3d(matching.normal_pdf(discovarable(0)),matching.normal_pdf(discovarable(1)),matching.normal_pdf(discovarable(2)));
		Aruco_event new_event(event_sigma,discovarable);
		event_dq.push_back(new_event);
		return 1;
		}
	
	}
	void odom_callback(const nav_msgs::Odometry &msg){
		if(!isnan(msg.pose.pose.orientation.w)&&!isnan(msg.pose.pose.orientation.x)&&!isnan(msg.pose.pose.orientation.y)&&!isnan(msg.pose.pose.orientation.z)){
		base_quaternion.w() = msg.pose.pose.orientation.w;
		base_quaternion.x() = msg.pose.pose.orientation.x;
		base_quaternion.y() = msg.pose.pose.orientation.y;
		base_quaternion.z() = msg.pose.pose.orientation.z;
		base_pose = toEigen(msg.pose.pose.position);
		Eigen::Vector3d linear_velocity = toEigen(msg.twist.twist.linear);
		Eigen::Vector3d angular_velocity = toEigen(msg.twist.twist.angular);
		trust_coeff = 0.05/(log(linear_velocity.squaredNorm()+1.5)*log(angular_velocity.squaredNorm()+1.135));
		}
		pose_points.push_back(msg.pose.pose.position);
	}
	void check_event_deque(){
		if(event_dq.size()>0)
		while (!event_dq.at(0).check_available()) {
			event_dq.pop_front();
			if (event_dq.size() < 1 )
			break;
		}
	}
	geometry_msgs::Quaternion toGeometry_msgs( Eigen::Quaterniond e){
		geometry_msgs::Quaternion q;
		q.w = e.w();
		q.x = e.x();
		q.y = e.y();
		q.z = e.z();
		return q;
	}
	geometry_msgs::Point toGeometry_msgs( const Eigen::Vector3d &v3 ){
		geometry_msgs::Point r;
		r.x =v3(0) ;
		r.y =v3(1) ;
		r.z =v3(2) ;
		return r;
		}
	void rvisualize(const ros::TimerEvent &event){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.ns = "accelerations";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CUBE_LIST;
		marker.action = visualization_msgs::Marker::ADD;
		marker.points = pose_points;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0; 
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0; 
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		pose_pub.publish(marker);
	}
		
	inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
     Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
     return ev3;
    }
	inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
     Eigen::Vector3d ev3(p.x, p.y, p.z);
     return ev3;
    }
	inline Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &p) {
     Eigen::Quaterniond q4(p.w ,p.x, p.y, p.z);
     return q4;
    }
	inline Eigen::Vector4d toEigen4V(const geometry_msgs::Quaternion &p) {
     Eigen::Vector4d v4(p.w ,p.x, p.y, p.z);
     return v4;
    }

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "aruco_detect");

	MarkerPlanner ArucoDetector;

	ROS_INFO("planner started");

	ros::spin();
}
