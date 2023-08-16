#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

ros::Publisher ir_front_pub;
ros::Publisher ir_right_pub;
ros::Publisher ir_left_pub;
ros::Publisher odom_pub;

// Callback function for the /pose topic
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& pose_msg)
{
    // Create an Odometry message
    nav_msgs::Odometry odom_msg;

    // Populate the Odometry message
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = pose_msg->x;
    odom_msg.pose.pose.position.y = pose_msg->y;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_msg->theta);

    // Publish the Odometry message on the /odom topic
    odom_pub.publish(odom_msg);

    // Broadcast the TF transform
    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_msg->x, pose_msg->y, 0.0));
    transform.setRotation(tf::Quaternion(odom_msg.pose.pose.orientation.x,
                                         odom_msg.pose.pose.orientation.y,
                                         odom_msg.pose.pose.orientation.z,
                                         odom_msg.pose.pose.orientation.w));
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}
// Callback function for front_distance topic
void frontDistanceCallback(const std_msgs::Float32::ConstPtr& distance_msg)
{
    sensor_msgs::Range ir_front_msg;
    ir_front_msg.header.stamp = ros::Time::now();
    ir_front_msg.header.frame_id = "front_ir";
    ir_front_msg.radiation_type = sensor_msgs::Range::INFRARED;
    ir_front_msg.field_of_view = 0.034906585; // 2 degrees in radians
    ir_front_msg.min_range = 0.1;
    ir_front_msg.max_range = 0.8;
    ir_front_msg.range = distance_msg->data;
    
    ir_front_pub.publish(ir_front_msg);
}

// Callback function for right_distance topic
void rightDistanceCallback(const std_msgs::Float32::ConstPtr& distance_msg)
{
    sensor_msgs::Range ir_right_msg;
    ir_right_msg.header.stamp = ros::Time::now();
    ir_right_msg.header.frame_id = "right_ir";
    ir_right_msg.radiation_type = sensor_msgs::Range::INFRARED;
    ir_right_msg.field_of_view = 0.034906585; // 2 degrees in radians
    ir_right_msg.min_range = 0.1;
    ir_right_msg.max_range = 0.8;
    ir_right_msg.range = distance_msg->data;
    
    ir_right_pub.publish(ir_right_msg);
}

// Callback function for left_distance topic
void leftDistanceCallback(const std_msgs::Float32::ConstPtr& distance_msg)
{
    sensor_msgs::Range ir_left_msg;
    ir_left_msg.header.stamp = ros::Time::now();
    ir_left_msg.header.frame_id = "left_ir";
    ir_left_msg.radiation_type = sensor_msgs::Range::INFRARED;
    ir_left_msg.field_of_view = 0.034906585; // 2 degrees in radians
    ir_left_msg.min_range = 0.1;
    ir_left_msg.max_range = 0.8;
    ir_left_msg.range = distance_msg->data;
    
    ir_left_pub.publish(ir_left_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_driver_node");
    ros::NodeHandle nh;

    // Create a publisher for the /odom topic
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    // Subscribe to the /pose topic
    ros::Subscriber pose_sub = nh.subscribe("/pose", 10, poseCallback);

    // Create publishers for the new topics
    ir_front_pub = nh.advertise<sensor_msgs::Range>("/ir_front_sensor", 10);
    ir_right_pub = nh.advertise<sensor_msgs::Range>("/ir_right_sensor", 10);
    ir_left_pub = nh.advertise<sensor_msgs::Range>("/ir_left_sensor", 10);

    // Subscribe to distance topics and assign callback functions
    ros::Subscriber front_distance_sub = nh.subscribe("/front_distance", 10, frontDistanceCallback);
    ros::Subscriber right_distance_sub = nh.subscribe("/right_distance", 10, rightDistanceCallback);
    ros::Subscriber left_distance_sub = nh.subscribe("/left_distance", 10, leftDistanceCallback);


    ros::spin();

    return 0;
}
