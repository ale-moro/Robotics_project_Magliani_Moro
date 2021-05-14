#include <ros/ros.h>
#include <math.h>
#include <sstream>
#include <dynamic_reconfigure/server.h>
#include <robotics_project1/parametersConfig.h>
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "robotics_project1/OdometryWithIntegrationMethod.h"
#include <tf/transform_broadcaster.h>
#include "robotics_project1/ResetOdometryToZero.h"
#include "robotics_project1/ResetOdometryToGivenPose.h"

/*
	This node extimates the pose of the robot from the TwistStamped messages published in /velocities, 
	and publishes the results to two other topics: /odometry (nav_msgs/Odometry messages) and /odometry_cust
	(custom messages OdometryWithIntegrationMethod).
	It also publishes the transformation between the odom and base_link reference systems.
	It offers two services to reset the pose to (0,0) and to a given pose (x,y,theta).
	The odometry is calculated using either Euler or Runge-Kutta integration method, depending on a parameter
	which can be set with dynamic reconfigure. The default method is Euler.
*/

enum integration_methods {EULER, RUNGE_KUTTA};

class odometry_calculator {
	private:

		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher pub_odom; // publishes Odometry messages
		ros::Publisher pub_custom; // publishes OdometryWithIntegrationMethod messages

		dynamic_reconfigure::Server<robotics_project1::parametersConfig> parameters_server; 

		integration_methods integration_method; // the integration method used to compute the odometry

		double x_previous;
		double y_previous;
		double theta_previous;
		ros::Time t_previous;

		tf::TransformBroadcaster transform_broadcaster;
  		tf::Transform transform;

		ros::ServiceServer reset_odometry_to_zero_service;
		ros::ServiceServer reset_odometry_to_given_pose_service;

	public: 

		void handle_parameters(robotics_project1::parametersConfig &config, uint32_t level) {

			// callback of the dynamic reconfigure server: depending on the parameter received,
			// sets the integration method to Euler or Runge-Kutta

			switch (config.integration_method) {
				case 0: this->integration_method = EULER; break;
				case 1: this->integration_method = RUNGE_KUTTA; break;
			}
		}

		void calculate_odometry(const geometry_msgs::TwistStamped::ConstPtr& msg){

			// callback of the subscriber to the topic /velocities: calculates the pose of the robot
			// from the linear and angular velocity and publishes them in the topic /odometry within
			// nav_msgs/Odometry messages, and in the topic /odometry_cust within the custom message
			// OdometryWithIntegrationMethod.

			// calculation of the odometry

			double x_next, y_next, theta_next, delta_t;
			delta_t = (msg->header.stamp - this->t_previous).toSec();
			if (this->integration_method == EULER) {
				x_next = this->x_previous + msg->twist.linear.x * delta_t * cos(theta_previous);
				y_next = this->y_previous + msg->twist.linear.x * delta_t * sin(theta_previous);
			} else { // if this->integration_method == RUNGE_KUTTA
				x_next = this->x_previous + msg->twist.linear.x * delta_t * cos(theta_previous + msg->twist.angular.z * delta_t * 0.5);
				y_next = this->y_previous + msg->twist.linear.x * delta_t * sin(theta_previous + msg->twist.angular.z * delta_t * 0.5);
			}
			theta_next = this->theta_previous + msg->twist.angular.z * delta_t;
			
			// publication of the Odometry and OdometryWithIntegrationMethod messages

			robotics_project1::OdometryWithIntegrationMethod msg_odometry;

			msg_odometry.odom.header.seq = msg->header.seq;
			msg_odometry.odom.header.stamp = msg->header.stamp;
			msg_odometry.odom.header.frame_id = "odom";

			msg_odometry.odom.child_frame_id = "base_link";

			msg_odometry.odom.pose.pose.position.x = x_next;
			msg_odometry.odom.pose.pose.position.y = y_next;
			msg_odometry.odom.pose.pose.position.z = 0.0; 
			msg_odometry.odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_next);

			for (int i = 0; i < 16; i++) {
				msg_odometry.odom.pose.covariance[i] = 0.0;
			}

			msg_odometry.odom.twist.twist = msg->twist;

			for (int i = 0; i < 16; i++) {
				msg_odometry.odom.twist.covariance[i] = 0.0;
			}

			std::stringstream method;
			if (this->integration_method == EULER) {
				method << "euler";
			} else {
				method << "rk";
			}
			msg_odometry.method.data = method.str();

			this->pub_custom.publish(msg_odometry);
			this->pub_odom.publish(msg_odometry.odom);

			// publication of the transormation between odom and base_link reference systems

			transform.setOrigin( tf::Vector3(x_next, y_next, 0));
    		tf::Quaternion q;
    		q.setRPY(0, 0, theta_next);
    		transform.setRotation(q);
    		transform_broadcaster.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_link"));

    		// saving of the values needed for the next integration

			this->x_previous = x_next;
			this->y_previous = y_next;
			this->theta_previous = theta_next;
			this->t_previous = msg->header.stamp;

		}

		bool reset_odometry_to_zero(robotics_project1::ResetOdometryToZero::Request  &req,
	        robotics_project1::ResetOdometryToZero::Response &res) {

			// callback of the service ResetOdometryToZero: forces the robot position to be (0,0),
			// keeping unchanged the orientation theta

			this->x_previous = 0.0;
			this->y_previous = 0.0;  
			
			return true;
		}

		bool reset_odometry_to_given_pose(robotics_project1::ResetOdometryToGivenPose::Request  &req,
	        robotics_project1::ResetOdometryToGivenPose::Response &res) {

			// callback of the service ResetOdometryToGivenPose: forces the robot pose to 
			// (x,y,theta) chosen by the caller of the service

			this->x_previous = req.x;
			this->y_previous = req.y;
			this->theta_previous = req.theta; 

			return true;
		}

		odometry_calculator() {

			dynamic_reconfigure::Server<robotics_project1::parametersConfig>::CallbackType parameters_handler;
			parameters_handler = boost::bind(&odometry_calculator::handle_parameters, this, _1, _2);
  			this->parameters_server.setCallback(parameters_handler);
  			this->integration_method = RUNGE_KUTTA; // default

  			sub = n.subscribe("/velocities", 10, &odometry_calculator::calculate_odometry, this);

  			pub_odom = n.advertise<nav_msgs::Odometry>("odometry", 10);
  			pub_custom = n.advertise<robotics_project1::OdometryWithIntegrationMethod>("odometry_cust", 10);

  			this->n.getParam("/initial_pose_x", this->x_previous);
  			this->n.getParam("/initial_pose_j", this->y_previous);
  			this->n.getParam("/initial_pose_theta", this->theta_previous);

  			this->reset_odometry_to_zero_service = n.advertiseService("reset_odometry_to_zero", &odometry_calculator::reset_odometry_to_zero, this);
  			this->reset_odometry_to_given_pose_service = n.advertiseService("reset_odometry_to_given_pose", &odometry_calculator::reset_odometry_to_given_pose, this);
		}
};	


int main(int argc, char **argv) {
  ros::init(argc, argv, "pub_odometry");
  odometry_calculator odometry_calculator;
  ros::spin();
  return 0;
}
