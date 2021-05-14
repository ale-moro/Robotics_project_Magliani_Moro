  
#include "ros/ros.h"
#include "cmath"
#include "robotics_pkg/ResetOdometryToInit.h"


bool reset_to_init(robotics_pkg::ResetOdometryToInit::Request& req,
                      robotics_pkg::ResetOdometryToInit::Response& res)
{
    x = 0.0;
    y = 0.0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ClientResetOdomToZero");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("ClientResetOdomToZero", ClientResetOdomToZero);
  ROS_INFO("Ready to reset the odometry to (0,0).");
  ros::spin();

  return 0;
}
