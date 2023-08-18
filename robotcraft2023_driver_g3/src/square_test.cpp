#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>
ros::Publisher cmd_vel_pub;
ros::Subscriber ir_front_sub;
ros::Subscriber ir_left_sub;
ros::Subscriber ir_right_sub;
ros::Subscriber odom_sub;
geometry_msgs::Twist cmd;
ros::Publisher set_pose_pub;
geometry_msgs::Pose2D msg;
geometry_msgs::Pose2D reset_pose;
int state = 0;  // 0: moving forward, 1: rotating
double target_angle = 0.0;
double target_distance = 0.5;  // 30 cm
double angular_velocity = 0.3;  // Adjust as needed


void irFrontCallback(const sensor_msgs::Range::ConstPtr& ir_front_msg)
{
    if (ir_front_msg->range < 0.15 && ir_front_msg->range >= 0.01)
    {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle, on the front side",
                 ir_front_msg->range);
    }
}

void irRightCallback(const sensor_msgs::Range::ConstPtr& ir_right_msg)
{
    if (ir_right_msg->range < 0.15 && ir_right_msg->range >= 0.01)
    {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle, on the right side",
                 ir_right_msg->range);
    }
}

void irLeftCallback(const sensor_msgs::Range::ConstPtr& ir_left_msg)
{
    if (ir_left_msg->range < 0.15 && ir_left_msg->range >= 0.01)
    {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle, on the left side",
                 ir_left_msg->range);
    }
}




void resetRobotOrientation() {
    set_pose_pub.publish(reset_pose);
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    if (state == 0) {
        if (current_x < target_distance) {
            cmd.linear.x = 0.1;  // Adjust linear velocity as needed
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
            resetRobotOrientation();  // Reset orientation to 0 degrees
            state = 0;  // Move forward again
        }
    }

    cmd_vel_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_test");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_sub = nh.subscribe("/odom", 10, odomCallback);
    set_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/set_pose", 10);
    ir_front_sub = nh.subscribe("/ir_front_sensor", 10, irFrontCallback);
    ir_right_sub = nh.subscribe("/ir_right_sensor", 10, irRightCallback);
    ir_left_sub = nh.subscribe("/ir_left_sensor", 10, irLeftCallback);

    reset_pose.x = 0;
    reset_pose.y = 0;
    reset_pose.theta = 0;
    set_pose_pub.publish(reset_pose);

    ros::Rate loop_rate(10);
    ros::spin();
}
