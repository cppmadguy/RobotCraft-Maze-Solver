#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

ros::Publisher cmd_vel_pub;
ros::Subscriber ir_front_sub;
ros::Subscriber ir_right_sub;
ros::Subscriber ir_left_sub;
ros::Subscriber odom_sub;

double linear_speed = 0.1;
double angular_speed = 0.0;
bool turning = false;
double side_length = 0.5;  // 0.5 meters side length for the square
int steps = 0;

geometry_msgs::Twist calculateMovement(double linear_speed, double angular_speed)
{
    geometry_msgs::Twist twist;
    twist.linear.x = linear_speed;
    twist.angular.z = angular_speed;
    return twist;
}

void irFrontCallback(const sensor_msgs::Range::ConstPtr& ir_front_msg)
{
    if (ir_front_msg->range < 0.15)
    {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle, on the front side",
                 ir_front_msg->range);
    }
}

void irRightCallback(const sensor_msgs::Range::ConstPtr& ir_right_msg)
{
    if (ir_right_msg->range < 0.15)
    {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle, on the right side",
                 ir_right_msg->range);
    }
}

void irLeftCallback(const sensor_msgs::Range::ConstPtr& ir_left_msg)
{
    if (ir_left_msg->range < 0.15)
    {
        ROS_WARN("Collision risk! The robot is %.2f meters from an obstacle, on the left side",
                 ir_left_msg->range);
    }
}
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    static double start_x = 0.0;
    static double start_y = 0.0;
    static bool started = false;

    if (!started)
    {
        // Initialize the starting position
        start_x = odom_msg->pose.pose.position.x;
        start_y = odom_msg->pose.pose.position.y;
        started = true;
    }

    double current_x = odom_msg->pose.pose.position.x;
    double current_y = odom_msg->pose.pose.position.y;

    double distance_x = current_x - start_x;
    double distance_y = current_y - start_y;

    if (steps < 4)
    {
        if (!turning)
        {
            if (steps % 2 == 0)
            {
                if (distance_x >= side_length * (steps / 2))
                {
                    turning = true;
                    steps++;
                    angular_speed = -0.785; // Turn 45 degrees (pi/4 radians)
                }
            }
            else
            {
                if (distance_y >= side_length * ((steps + 1) / 2))
                {
                    turning = true;
                    steps++;
                    angular_speed = -0.785; // Turn 45 degrees (pi/4 radians)
                }
            }
        }
        else
        {
            if (odom_msg->pose.pose.orientation.z <= 0.0)
            {
                turning = false;
                angular_speed = 0.0; // Stop turning
            }
        }

        // Move straight for side_length
        if (!turning)
        {
            cmd_vel_pub.publish(calculateMovement(linear_speed, angular_speed));
        }
    }
    else
    {
        // Stop after completing the square
        cmd_vel_pub.publish(calculateMovement(0.0, 0.0));
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "square_test");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ir_front_sub = nh.subscribe("/ir_front_sensor", 10, irFrontCallback);
    ir_right_sub = nh.subscribe("/ir_right_sensor", 10, irRightCallback);
    ir_left_sub = nh.subscribe("/ir_left_sensor", 10, irLeftCallback);
    odom_sub = nh.subscribe("/odom", 10, odomCallback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
