#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher cmd_vel_pub;
ros::Subscriber odom_sub;
geometry_msgs::Twist cmd;
ros::Publisher set_pose_pub;
geometry_msgs::Pose2D msg;

int state = 0;  // 0: moving forward, 1: rotating
double target_angle = 0.0;
double target_distance = 0.3;  // 30 cm
double angular_velocity = 0.3;  // Adjust as needed

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    if (state == 0) {
        if (current_x < target_distance) {
            cmd.linear.x = 0.06;  // Adjust linear velocity as needed
            cmd.angular.z = 0.0;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            state = 1;  // Start rotating
            target_angle = tf::getYaw(msg->pose.pose.orientation) + M_PI / 2.0;  // 90 degrees
        }
    } else if (state == 1) {
        double current_yaw = tf::getYaw(msg->pose.pose.orientation);
        double angle_error = target_angle - current_yaw;

        if (std::abs(angle_error) > 0.1) {
            cmd.linear.x = 0.0;
            cmd.angular.z = angular_velocity;  // Rotate
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            state = 2;  // Move forward again
            target_distance += 0.3;  // Move to the next side of the square
        }
    } else if (state == 2) {
        if (current_x < target_distance) {
            cmd.linear.x = 0.06;  // Move forward
            cmd.angular.z = 0.0;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            state = 1;  // Start rotating
            target_angle = tf::getYaw(msg->pose.pose.orientation) + M_PI / 2.0;  // 90 degrees
        }
    }

    cmd_vel_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_test");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    odom_sub = nh.subscribe("/odom", 1, odomCallback);
    set_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/set_pose", 10);

    msg.x = 0;
    msg.y = 0;
    msg.theta = 0;
    set_pose_pub.publish(msg);

    ros::spin();

    return 0;
}
