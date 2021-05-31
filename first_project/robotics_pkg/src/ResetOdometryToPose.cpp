#include "ros/ros.h"
#include "cmath"
#include "robotics_pkg/ResetOdometryToPose.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_to_pose");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<service::ResetOdometryToPose>("reset_given_pose");
  service::ResetOdometryToPose srv;
  srv.request.x = atoll(argv[1]);
  srv.request.y = atoll(argv[2]);
  srv.request.theta = atoll(argv[3]);
  ROS_INFO("Ready to reset the odometry to given pose.");
  ros::spin();

  return 0;
}
