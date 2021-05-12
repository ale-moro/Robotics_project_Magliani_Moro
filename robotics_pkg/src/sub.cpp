#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "robotics_pkg/customOdometry.h"
#include <boost/function.hpp>

typedef struct odometry_values{
    double x;
    double y;
    double theta;
    double v_x;
    double v_y;
    double omega;
    double steer;
    double v_r;
    double v_l;
} OdometryValues;


void Callback(const nav_msgs::Odometry::ConstPtr& msg){


    //ROS_INFO("I heard: [%d]", msg->twist.twist.angular.z);
	ROS_INFO("I heard");
	
  
}

int main(int argc, char **argv){

ros::init(argc,argv,"sub_eseg");
ros::NodeHandle n;

ros::Subscriber sub = n.subscribe("Odometry",1000,Callback);
ros::spin();
