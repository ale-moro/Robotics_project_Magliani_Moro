#include "ros/ros.h" 
#include <std_msgs/String.h>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include "robotics_pkg/customOdometry.h"
#include "robotics_pkg/MotorSpeed.h"
#include "robotics_pkg/ResetOdometryToInit.h"
#include "robotics_pkg/ResetOdometryToPose.h"
#include <dynamic_reconfigure/server.h>
#include <robotics_pkg/parametersConfig.h>
#include <boost/bind.hpp>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/connection.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


//Topic/Frame Description
#define FRAME_ID "odom"
#define CHILD_FRAME_ID "base_link"
#define TOPIC_ID "odometry"
#define CUSTOM_TOPIC_ID "customOdometry"

//Values given by the text
#define BASELINE 0.583
#define RADIUS 0.1575
#define APPARENT_BASELINE 1.055

//Default values given by the text
#define INIT_POSITION_X 0.0
#define INIT_POSITION_Y 0.0
#define INIT_POSITION_THETA 0.0
#define INIT_VELOCITY_X 0.0
#define INIT_VELOCITY_Y 0.0
#define INIT_ANGULAR_VELOCITY 0.0
#define INIT_VELOCITY_LEFT 0.0
#define INIT_VELOCITY_RIGHT 0.0

//Approximation choice
#define EULER_APPROXIMATION true
#define RUNGE_KUTTA_APPROXIMATION false
#define GEAR_RATIO 38.0
#define PI 3.14159265

//Struct for odometry pose
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

typedef struct skid_steering_variables{
    double omega_r;
    double omega_l;
} SkidSteeringVariables;

robotics_pkg::parametersConfig last_config;
OdometryValues odometry_values;
double last_msg_time;


//Function for reset the odometry to initial pose
void reset_odometry_to_init(){
	
    odometry_values.x = INIT_POSITION_X;
    odometry_values.y = INIT_POSITION_Y;
    odometry_values.theta = INIT_POSITION_THETA;
    odometry_values.v_r = INIT_VELOCITY_RIGHT;
    odometry_values.v_l = INIT_VELOCITY_LEFT;
    odometry_values.v_x = INIT_VELOCITY_X;
    odometry_values.v_y = INIT_VELOCITY_Y;
    odometry_values.omega = INIT_ANGULAR_VELOCITY;
}

void configCallBack(robotics_pkg::parametersConfig &config, uint32_t level){

	ROS_INFO("ENTRATO IN CONFIGCALLBACK");
	if(config.approximation_model_mode!=last_config.approximation_model_mode) {
       ROS_INFO("Approximation changed: changing odometry model to: \t%s", config.approximation_model_mode?"Runge Kutta":"Euler");
       last_config.approximation_model_mode = config.approximation_model_mode; 
    } 
    if(config.approximation_set_position){
        ROS_INFO("Position changed: setting position to: \t(X: %f, Y: %f)", config.approximation_x_position, config.approximation_y_position);
        odometry_values.x = config.approximation_x_position;
        odometry_values.y = config.approximation_y_position;
    } else if(config.approximation_reset_default){
        ROS_INFO("Position changed: setting position to: \t(X: %f, Y: %f)", INIT_POSITION_X,INIT_POSITION_Y);
        reset_odometry_to_init();
    }
}


void Euler_Approximation(double sample_time, double speed_r, double speed_l, OdometryValues& approximate_values){

    double linear_velocity = (odometry_values.v_r + odometry_values.v_l) / 2;
    double angular_velocity = (odometry_values.v_r - odometry_values.v_l) / APPARENT_BASELINE;
    double delta_theta = angular_velocity * sample_time;

	speed_r = speed_r/(GEAR_RATIO*60);
	speed_l = speed_l/(GEAR_RATIO*60);
	
	
    approximate_values.x = odometry_values.x + linear_velocity * sample_time * cos(odometry_values.theta);
    approximate_values.y = odometry_values.y + linear_velocity * sample_time * sin(odometry_values.theta);
    approximate_values.theta = odometry_values.theta + delta_theta;


    if (approximate_values.theta > 2*PI) {
        approximate_values.theta -= 2*PI;
    } else if (approximate_values.theta < 0) {
        approximate_values.theta += 2*PI;
    }
	
    approximate_values.omega = angular_velocity;

    approximate_values.v_x = (linear_velocity/2) * cos(approximate_values.theta);
    approximate_values.v_y = (linear_velocity/2) * sin(approximate_values.theta);

    approximate_values.v_r = speed_r;
    approximate_values.v_l = speed_l;

}

void Runge_Kutta_Approximation(double sample_time, double speed_r, double speed_l, OdometryValues& approximate_values){

    double linear_velocity = (odometry_values.v_r + odometry_values.v_l) / 2;
    double angular_velocity = (odometry_values.v_r - odometry_values.v_l) / APPARENT_BASELINE;
    double delta_theta = angular_velocity * sample_time;

	speed_r = speed_r/(GEAR_RATIO*60);
	speed_l = speed_l/(GEAR_RATIO*60);
    

    approximate_values.theta = odometry_values.theta + delta_theta;

    if (approximate_values.theta > 2*PI) {
        approximate_values.theta -= 2*PI;
    } else if (approximate_values.theta < 0) {
        approximate_values.theta += 2*PI;
    }

    approximate_values.x = odometry_values.x + linear_velocity * sample_time * cos(odometry_values.theta + (angular_velocity * sample_time)/2);
    approximate_values.y = odometry_values.y + linear_velocity * sample_time * sin(odometry_values.theta + (angular_velocity * sample_time)/2);

    approximate_values.omega = angular_velocity;

    approximate_values.v_x = (linear_velocity/2) * cos(approximate_values.theta);
    approximate_values.v_y = (linear_velocity/2) * sin(approximate_values.theta);

    approximate_values.v_r = speed_r;
    approximate_values.v_l = speed_l;

}

nav_msgs::Odometry setOdometry(){

    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = FRAME_ID;

    odometry.pose.pose.position.x = odometry_values.x;
    odometry.pose.pose.position.y = odometry_values.y;
    odometry.pose.pose.position.z = 0.0;
    odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometry_values.theta);

    odometry.child_frame_id = CHILD_FRAME_ID;
    
    odometry.twist.twist.linear.x = odometry_values.v_x;
    odometry.twist.twist.linear.y = odometry_values.v_y;
    odometry.twist.twist.angular.z = odometry_values.omega;

    return odometry;
}

//per associare ad ogni odometry l'approssimazione scelta dall'utente
robotics_pkg::customOdometry customOdometry(nav_msgs::Odometry odometry, std_msgs::String approximation_model){

    ROS_INFO("ENTRATO IN CUSTOMODOMETRY");
    robotics_pkg::customOdometry customOdometry;
    customOdometry.odometry = odometry;
	customOdometry.approxMode = approximation_model;

    return customOdometry;
}

geometry_msgs::TransformStamped setTransform(){
    
    ROS_INFO("ENTRATO IN SET TRASFORM");
    geometry_msgs::TransformStamped odomtransform;
    odomtransform.header.stamp = ros::Time::now();
    odomtransform.header.frame_id = FRAME_ID;
    odomtransform.child_frame_id = CHILD_FRAME_ID;

    odomtransform.transform.translation.x = odometry_values.x;
    odomtransform.transform.translation.y = odometry_values.y;
    odomtransform.transform.translation.z = 0.0;
    odomtransform.transform.rotation = tf::createQuaternionMsgFromYaw(odometry_values.theta);

    return odomtransform;

}

void callback(const robotics_pkg::MotorSpeed::ConstPtr& left, const robotics_pkg::MotorSpeed::ConstPtr& right, tf::TransformBroadcaster& broadcaster, ros::Publisher& publisherOdometry, ros::Publisher& publisherCustomOdometry){

	ROS_INFO("ENTRATO IN SUBCALLBACK");
	std_msgs::String message;
    double time = (left->header.stamp.toSec() + right->header.stamp.toSec()) / 2;
    double delta_time = time - last_msg_time;
    double left_w = -1*(left->rpm);
    last_msg_time = time;
    OdometryValues new_odometry_values;
    if(last_config.approximation_model_mode == EULER_APPROXIMATION){
        Euler_Approximation(delta_time, right->rpm, left_w, new_odometry_values);
        char odometryModel[20] = "Euler Approximation";
        message.data = odometryModel;
    } else if(last_config.approximation_model_mode == RUNGE_KUTTA_APPROXIMATION){
        Runge_Kutta_Approximation(delta_time, right->rpm, left_w, new_odometry_values);
        char odometryModel[26] = "Runge Kutta Approximation";
        message.data = odometryModel;
    } else {
        ROS_INFO("ERROR CONFIG!");
    }
    
    odometry_values = new_odometry_values;

    nav_msgs::Odometry odometry = setOdometry();
    publisherOdometry.publish(odometry);

    ROS_INFO("SET ODOMETRY: %f , %f", odometry.pose.pose.position.x, odometry.pose.pose.position.y);
    ROS_INFO(" LINEAR: %f , %f , %f", odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.angular.z );

    geometry_msgs::TransformStamped transform = setTransform();
    broadcaster.sendTransform(transform);

    ROS_INFO("SET TRASFORM");

    robotics_pkg::customOdometry custom_Odometry = customOdometry(odometry, message);
    publisherCustomOdometry.publish(custom_Odometry);

    ROS_INFO("CUSTODOMETRY");
}

bool reset_to_init(robotics_pkg::ResetOdometryToInit::Request &req, robotics_pkg::ResetOdometryToInit::Response &resp){
    odometry_values.x = 0.0;
    odometry_values.y =0.0;
    return true;
}


bool reset_to_pose(robotics_pkg::ResetOdometryToPose::Request &req, robotics_pkg::ResetOdometryToPose::Response &resp){
    odometry_values.x = req.x;
    odometry_values.y = req.y;
    odometry_values.theta = req.theta;
    return true;
}

int main(int argc, char **argv){

	//initialization
	ros::init(argc,argv, "main_node_eseg");
	ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("reset_to_init", reset_to_init);
    ros::ServiceServer service2 = n.advertiseService("reset_to_pose", reset_to_pose);
    
	reset_odometry_to_init();
    last_msg_time = ros::Time::now().toSec();
    ROS_INFO("INIZIALIZZATO");

	//publisher creation
	ros::Publisher odom_publisher = n.advertise<nav_msgs::Odometry>(TOPIC_ID,1000);
    ros::Publisher cust_odom_publisher = n.advertise<robotics_pkg::customOdometry>(CUSTOM_TOPIC_ID,1000);
   	
   	ROS_INFO("PUB CREATI");

    //dynamic reconfigure
    dynamic_reconfigure::Server<robotics_pkg::parametersConfig> server;
    server.setCallback(boost::bind(&configCallBack, _1, _2));

    ROS_INFO("RECONFIGURE FATTA");

    //tf broadcaster
    tf::TransformBroadcaster odom_broadcaster;

    ROS_INFO("BROADCASTER CREATO");
    //message filters
    message_filters::Subscriber<robotics_pkg::MotorSpeed> left_velocity_sub(n, "/motor_speed_fl", 1000);
    message_filters::Subscriber<robotics_pkg::MotorSpeed> right_velocity_sub(n, "/motor_speed_fr", 1000);
    typedef message_filters::sync_policies::ApproximateTime<robotics_pkg::MotorSpeed, robotics_pkg::MotorSpeed> MySyncPolicy;

    ROS_INFO("SUBSCRIBER BUILT\n");
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_velocity_sub, right_velocity_sub);
    ROS_INFO("SYNCHRONIZER STARTED\n");
    sync.registerCallback(boost::bind(&callback, _1, _2, odom_broadcaster, odom_publisher, cust_odom_publisher));
    ROS_INFO("TOPICS SYNCHRONIZED\n");


    ros::spin();
    return 0;
}
