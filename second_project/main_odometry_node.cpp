#define ACKERMAN_ODOM_H
#define CHILD_FRAME_ID "base_link"
#define FRAME_ID "odom"
#define STEERING_FACTOR 18
#define FRONT_REAR_DISTANCE 1.765 // from cm (176.5) to m

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

typedef struct Odometry {
    
    double x_dot = 0;
    double y_dot = 0;
    double theta_dot = 0;

    double v = 0;
    double v_x = 0;
    double v_y = 0;
    double omega = 0;
    
    ros::Time time_ = ros::Time(0, 0);

    ros::NodeHandle n;
    ros::Publisher p_odom;

    ros::Subscriber speedsteer;
    tf::TransformBroadcaster broadcaster;
    
} OdometryValues;


void setOdometry(double x, double y, double theta)
{
    x_dot = x;
    y_dot = y;
    theta_dot = theta;
    time_ = ros::Time::now();
    speedsteer = n.subscribe("/speedsteer", 1000, &Odometry::calculate, this); //needs changes

    p_odom = n.advertise<nav_msgs::Odometry>("/car/odometry/ackerman", 50);
}

void broadcastTransform()
{
    geometry_msgs::TransformStamped odom_to_base;
    odom_to_base.header.frame_id = "odom";
    odom_to_base.header.stamp = ros::Time::now();
    odom_to_base.child_frame_id = "base_link";

    odom_to_base.transform.translation.x = x_dot;
    odom_to_base.transform.translation.y = y_dot;
    odom_to_base.transform.translation.z = 0;
    odom_to_base.transform.rotation = tf::createQuaternionMsgFromYaw(theta_dot);

    broadcaster.sendTransform(odom_to_base);
}

void publishAsOdom()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_dot;
    odom.pose.pose.position.y = y_dot;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_dot);

    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.angular.z = omega;

    p_odom.publish(odom);
}

inline double Odometry::deg2rad(double degrees)
{
    return (degrees * M_PI) / 180;
}

inline double Odometry::kmph2mps(double speed_km_per_hour){
    return speed_km_per_hour / 3.6;
}

void calculate(const geometry_msgs::PointStampedConstPtr &speed_steer)
{
    double alpha = deg2rad(speed_steer->point.x) / STEERING_FACTOR;

    const ros::Time& current_time = speed_steer->header.stamp;
    double dt = (current_time - time_).toSec();
    time_ = current_time;

    v = kmph2mps(speed_steer->point.y);
    omega = v * std::tan(alpha) / FRONT_REAR_DISTANCE;

    v_x = v * std::cos(theta_dot);
    v_y = v * std::sin(theta_dot);

    x_dot += v * std::cos(theta_dot) * dt;
    y_dot += v * std::sin(theta_dot) * dt;
    theta_dot += omega * dt;

    std::cout <<  alpha << std::endl<< dt << std::endl<< V << std::endl<< omega << std::endl<< theta_dot << std::endl;

    broadcastTransform();
    publishAsOdom();
}

int main(int argc, char **argv)
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    if(argc >= 4){
        x = std::strtod(argv[1], NULL);
        y = std::strtod(argv[2], NULL);
        theta = std::strtod(argv[3], NULL);
    }

    ros::init(argc, argv, "odometry_node");

    setOdometry(x, y, theta);
    ROS_INFO("test");

    ros::spin();
}
