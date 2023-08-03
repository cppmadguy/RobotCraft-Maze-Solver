#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance_front;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Find the starting and ending indices for the front region (approximately 120 degrees)
  int front_start_index = msg->ranges.size() / 3;
  int front_end_index = 2 * msg->ranges.size() / 3;

  // Find the minimum distance value in the front region
  double min_distance_front = msg->ranges[front_start_index];
  for (int i = front_start_index + 1; i < front_end_index; ++i) {
    min_distance_front = std::min<double>(min_distance_front, msg->ranges[i]);
  }

  obstacle_distance_front = min_distance_front;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_avoidance");
  ros::NodeHandle n;

  // Publisher for /cmd_vel
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  // Subscriber for /base_scan
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

  ros::Rate loop_rate(20); // 20 Hz

  // Initializations:
  geometry_msgs::Twist cmd_vel_msg;

  double obstacle_avoidance_distance = 0.9; // Set the distance at which the robot stops for obstacle avoidance

  while (ros::ok()) {
    // Obstacle avoidance behavior:
    if (obstacle_distance_front < obstacle_avoidance_distance) {
      // If an obstacle is detected in the front region, stop and rotate in place
      cmd_vel_msg.linear.x = 0.0; // Stop forward movement
      cmd_vel_msg.angular.z = 1.2; // Rotate in place
    } else {
      // If no obstacle is detected in the front region, move forward
      cmd_vel_msg.linear.x = 2.5; // Move forward at a constant speed
      cmd_vel_msg.angular.z = 0.0; // No angular velocity
    }

    // Publish velocity commands
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
