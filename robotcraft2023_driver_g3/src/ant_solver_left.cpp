
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#define STATE_LOST 0
#define STATE_CW  1
#define STATE_WALL1 2

#define STATE_WALL2 3

const float  wall_distance = 0.40;
const float linear_speed = 0.35;
const float angular_speed = 0.20;

bool sensR, sensL;
ros:: Publisher cmd_vel_pub;
void sensR_Callback (const sensor_msgs::Range::ConstPtr& ir_right_msg) 
{ 
if (ir_right_msg->range < wall_distance) {
     sensR = true; 
}
else
{ 
    sensR = false;
}

}
void sensL_Callback (const sensor_msgs::Range::ConstPtr& ir_left_msg) 
{ if (ir_left_msg->range < wall_distance)
    {
    sensL= true; 
    }
    else{
    sensL= false;
    }
}
void move (double linear_speed)
{
  geometry_msgs::Twist cmd_msg;
  cmd_msg.linear.x = linear_speed;
  cmd_vel_pub.publish(cmd_msg);
}
void turn (double angular_speed)
{
  geometry_msgs::Twist cmd_msg;
  cmd_msg.angular.z = angular_speed;
  cmd_vel_pub.publish(cmd_msg);
}
int lost(void)
{
    if(!sensR && !sensL)
    {
        move(linear_speed);
    }
    else
    {
        return STATE_CW;
    }
    return STATE_LOST;
}
int cw(void)
{
    if(sensR || sensL)  
    {
        turn(-angular_speed);
    }
    else
    {
        return STATE_WALL1;
    }
    return STATE_CW;
}
int wall1(void)
{
    if(!sensL)
    {
        move(linear_speed);
        turn(+angular_speed);
    }
    else
    {
        return STATE_WALL2;
    }
    return STATE_WALL1;
}
int wall2(void)
{
    if(!sensR)
    {
     if(sensL)
     {
        turn(-angular_speed); 
        move(linear_speed);
     }
     else{
        return STATE_WALL1;
     }
    }
    else
    {
        return STATE_CW;
    }
    return STATE_WALL2;
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"maze_BASICsolver");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",100);

    ros::Subscriber sensL_sub = n.subscribe("/ir_left_sensor",100,sensL_Callback);

    ros::Subscriber sensR_sub = n.subscribe("/ir_right_sensor",100,sensR_Callback);

    int state = STATE_LOST;

    while(ros::ok())
    {
     switch(state)
     {
      case STATE_LOST:
        ROS_INFO("LOST");
        state = lost();
        break;
      case STATE_CW:
        ROS_INFO("CW");
        state = cw();
        break;
      case STATE_WALL1:
        ROS_INFO("WALL1");
        state = wall1();
        break;
      case STATE_WALL2:
        ROS_INFO("WALL2");
        state = wall2();
        break;
      default :
        ROS_INFO("INVALID STATE");
        state = STATE_LOST;
        break;                                
     }
     ros::spinOnce();
     loop_rate.sleep();

    }
  return 0;
}