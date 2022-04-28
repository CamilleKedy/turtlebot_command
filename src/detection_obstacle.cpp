#include "geometry_msgs/Twist.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Float32.h>

bool obstacle = false;

void laserCallback(const sensor_msgs::LaserScan &laser) {
  for (int i = 0; i < sizeof(laser.ranges); i++) {
    if (laser.ranges[i] < 0.3) {//si on détecte un obstacle à moins de 30cm on signale 
      ROS_INFO_STREAM("obstacle detecte");
      obstacle = true;
      // break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "detection_obstacle");
  ros::NodeHandle n;
  ros::Subscriber sub_laser = n.subscribe("/scan", 1000, laserCallback);
  ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  geometry_msgs::Twist msg;

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (obstacle) {
      // on stoppe le robot
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      pub_cmd.publish(msg);
      obstacle = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
