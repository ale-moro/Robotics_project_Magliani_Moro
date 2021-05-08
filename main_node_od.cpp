#include "ros/ros.h" 
#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>
#include "robotics_pkg/customOdometry.h"
#include "robotics_pkg/floatStamped.h"
#include <dynamic_reconfigure/server.h>
#include <robotics_pkg/parametersConfig.h>
#include <boost/bind.hpp>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/connection.h>
#include <message_filters/sync_policies/approximate_time.h>

//Topic/Frame Description
#define FRAME_ID "world"
#define CHILD_FRAME_ID "base_link"
#define TOPIC_ID "odometry"
#define CUSTOM_TOPIC_ID "customOdometry"

//Values given by the text
#define BASELINE 0.583
#define RADIUS 0.1575

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

void configCallBack(robotics_pkg::parametersConfig &config, uint32_t level){
	//capire dove viene inizializzata odometry_model_mode da andrea finazzi ed inizializzarla
}

//Service for reset the odometry to initial pose
void reset_odometry_to_init(){
    odometry_values.x = INIT_POSITION_X;
    odometry_values.y = INIT_POSITION_Y;
    odometry_values.theta = INIT_POSITION_THETA;
}

//Service for reset the odometry to pose(x,y,theta)
void reset_odometry_to_pose(double x, double y, double theta){
    odometry_values.x = x;
    odometry_values.y = y;
    odometry_values.theta = theta;
}

void Euler_Approximation(double sample_time, double speed_r, double speed_l, OdometryValues& approximate_values){

    double linear_velocity = (odometry_values.v_r + odometry_values.v_l) / 2;
    double angular_velocity = odometry_values.omega;
    double delta_theta = angular_velocity * sample_time;

   
    double Lambda = (odometry_values.v_r + odometry_values.v_l)/(odometry_values.v_r - odometry_values.v_l);
    double y0 = RADIUS/Lambda;
    double apparent_baseline = 2*y0;

    approximate_values.theta = odometry_values.theta + delta_theta;

    approximate_values.x = odometry_values.x + linear_velocity * sample_time * cos(odometry_values.theta);
    approximate_values.y = odometry_values.y + linear_velocity * sample_time * sin(odometry_values.theta);

    approximate_values.omega = (speed_r - speed_l)/ apparent_baseline;

    approximate_values.v_x = ((speed_r - speed_l)/2) * cos(approximate_values.theta);
    approximate_values.v_y = ((speed_r - speed_l)/2) * sin(approximate_values.theta);

    approximate_values.v_r = speed_r;
    approximate_values.v_l = speed_l;

}

void Runge_Kutta_Approximation(double sample_time, double speed_r, double speed_l, OdometryValues& approximate_values){

    double linear_velocity = (odometry_values.v_r + odometry_values.v_l) / 2;
    double angular_velocity = odometry_values.omega;
    double delta_theta = angular_velocity * sample_time;

    
    double Lambda = (odometry_values.v_r + odometry_values.v_l)/(odometry_values.v_r - odometry_values.v_l);
    double y0 = RADIUS/Lambda;
    double apparent_baseline = 2*y0;

    approximate_values.theta = odometry_values.theta + delta_theta;

    approximate_values.x = odometry_values.x + linear_velocity * sample_time * cos(odometry_values.theta + (angular_velocity * sample_time)/2);
    approximate_values.y = odometry_values.y + linear_velocity * sample_time * sin(odometry_values.theta + (angular_velocity * sample_time)/2);

    approximate_values.omega = (speed_r - speed_l)/ apparent_baseline;

    approximate_values.v_x = ((speed_r - speed_l)/2) * cos(approximate_values.theta);
    approximate_values.v_y = ((speed_r - speed_l)/2) * sin(approximate_values.theta);

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
    
    //vedere inizializzazione "linear" andrea finazzi per inizializzare velocity
    //odometry.twist.twist.velocity.left = odometry_values.v_l;
    //odometry.twist.twist.velocity.right = odometry_values.v_r;

    return odometry;
}

//per associare ad ogni odometry l'approssimazione scelta dall'utente
robotics_pkg::customOdometry customOdometry(nav_msgs::Odometry odometry){


	//NB: AGGIUNGERE 'STRING APPROXYMODE' IN INGRESSO 
    
    robotics_pkg::customOdometry customOdometry;
    
    //cercare inizializzazione odometry
    //customOdometry.odometry = odometry;

    //cercare inizializzazione odometryMode
    //customOdometry.odometryMode.data = approxMode;

    return customOdometry;
}

geometry_msgs::TransformStamped setTransform(){
    
    geometry_msgs::TransformStamped odomtransform;
    odomtransform.header.stamp = ros::Time::now();
    odomtransform.header.frame_id = FRAME_ID;
    
    //vedere inizializzazione "child_frame_id" e "translation_x" e "translation_y" su andrea finazzi
    //odomtransform.header.child_frame_id = CHILD_FRAME_ID;

    //odomtransform.transform.traslation_x = odometry_values.x;
    //odomtransform.transform.traslation_y = odometry_values.y;
    //odomtransform.transform.traslation_z = 0.0;
    //odomtransform.transform.rotation = tf::createQuaternionMsgFromYaw(odometry_values.theta);

    return odomtransform;

}

int main(int argc, char **argv){

	//initialization
	ros::init(argc,argv, "main_node_eseg");
	ros::NodeHandle n;

	//publisher creation
	ros::Publisher odom_publisher = n.advertise<nav_msgs::Odometry>(TOPIC_ID,1000);
    ros::Publisher cust_odom_publisher = n.advertise<robotics_pkg::customOdometry>(CUSTOM_TOPIC_ID,1000);
   
    //dynamica reconfigure
    dynamic_reconfigure::Server<robotics_pkg::parametersConfig> server;
    server.setCallback(boost::bind(&configCallBack, _1, _2));

    //tf broadcaster
    tf::TransformBroadcaster odom_broadcaster;

    //message filters
    message_filters::Subscriber<robotics_pkg::floatStamped> left_velocity_sub(n, "LEFT_VELOCITY", 1);
    message_filters::Subscriber<robotics_pkg::floatStamped> right_velocity_sub(n, "RIGHT_VELOCITY", 1);
    typedef message_filters::sync_policies::ApproximateTime<robotics_pkg::floatStamped, robotics_pkg::floatStamped> SyncPolicy;




    ros::spin();
    return 0;
}