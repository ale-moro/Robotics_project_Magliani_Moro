#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "robotics_pkg/customOdometry.h"

void subCallback(const nav_msgs::Odometry::ConstPtr& msg){
ROS_INFO("I heard: [%d]", msg->Odometry);
}

int main(int argc, char **argv){

ros::init(argc,argv,"sub_eseg");
ros::NodeHandle n;

ros::Subscriber sub = n.subscribe("Odometry",1000,subCallback);
ros::spin();
return 0;


}
