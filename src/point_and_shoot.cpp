#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

bool envoyerangle=false;
bool envoyerdist=false;
std_msgs::Float32 angle,dist;

float curX,curY;
double curYaw;

void odomCb(const nav_msgs::Odometry& pos){
    curX = pos.pose.pose.position.x;
    curY = pos.pose.pose.position.y;
    tf::Quaternion q(pos.pose.pose.orientation.x,pos.pose.pose.orientation.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch;
    m.getRPY(roll,pitch,curYaw);
}

void cibleCbOdom(const geometry_msgs::PoseStamped& cible){ // Cible exprimée dans le repère odom

    angle.data = atan2(cible.pose.position.y-curY,cible.pose.position.x-curX)-curYaw;
    dist.data = sqrt(pow(cible.pose.position.x-curX,2)+pow(cible.pose.position.y-curY,2));

    envoyerangle = true;
}

void cibleCb(const geometry_msgs::PoseStamped& cible){ // Cible exprimée dans le repère robot

    angle.data = atan2(cible.pose.position.y,cible.pose.position.x);
    dist.data = sqrt(pow(cible.pose.position.x,2)+pow(cible.pose.position.y,2));

    envoyerangle = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_and_shoot");
    ros::NodeHandle n;
    ros::Subscriber sub_cible = n.subscribe("cible",1000,cibleCb);
    ros::Subscriber sub_cible_odom = n.subscribe("/move_base_simple/goal",1000,cibleCbOdom);
    ros::Subscriber sub_odom = n.subscribe("/odom",1000,odomCb);
    ros::Publisher pub_tourner = n.advertise<std_msgs::Float32>("/tourner", 1000);
    ros::Publisher pub_avancer = n.advertise<std_msgs::Float32>("/avancer", 1000);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        if(envoyerangle){
            pub_tourner.publish(angle);
            envoyerangle = false;
            envoyerdist = true;
        }
        if(envoyerdist){
            ROS_INFO_STREAM(angle<<" "<<dist);
            pub_avancer.publish(dist);
            envoyerdist = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
