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
geometry_msgs::Pose2D msg;



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


double pos_x = 0.0;  // Initial x-coordinate
double pos_y = 0.0;  // Initial y-coordinate

double target_distance = 0.5;  // 50 cm
double target_angle = 0.0;
int step = 0;  // Current step in the pattern
double target_angle_val[4] = {1.5708,3.14159,-1.5708,0};
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;
    double current_yaw = tf::getYaw(msg->pose.pose.orientation);
    double d = sqrt((current_x - pos_x) * (current_x - pos_x) + (current_y - pos_y) * (current_y - pos_y));
    static int i = 0;
    if (step == 0) {
        if (d < target_distance) {
            cmd.linear.x = 0.1;  // Move forward
            cmd.angular.z = 0.0;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            step = 1;  // Proceed to turning step
        }
    } else if (step == 1) {  
        double angle_error = target_angle_val[i] - current_yaw;
        ROS_INFO("Angle Error = %lf",angle_error);
        if (std::abs(angle_error) > 0.1) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.3;  // Adjust angular velocity for turning
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            step = 0;  // Reset step to move forward
            pos_x = current_x;  // Update initial position for next side
            pos_y = current_y;
            i++;
            if(i == 4)
            {
                i = 0;
            }
        }
    }
    ROS_INFO("Target Angle:%lf",target_angle);

    // Publish the calculated linear and angular velocities to cmd_vel topic
    cmd_vel_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_test");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ir_front_sub = nh.subscribe("/ir_front_sensor", 10, irFrontCallback);
    ir_right_sub = nh.subscribe("/ir_right_sensor", 10, irRightCallback);
    ir_left_sub = nh.subscribe("/ir_left_sensor", 10, irLeftCallback);


    ros::Rate loop_rate(10);
    while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();  
    }
}
