#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/tf.h>

std::vector<int> typeMouvements;
std::vector<float> amplitudeMouvements;
float curX,curY,debutX,debutY;
double curYaw,debutYaw;

bool mvmtFini(){

    if(typeMouvements[0] == 0){ // Si on est en train d'avancer-
        return sqrt(pow(curX-debutX,2)+pow(curY-debutY,2)) >= amplitudeMouvements[0];
    }
    else if(typeMouvements[0] == 1){ // Si on est en train de tourner dans le sens anti-horaire
        double angleParcouru;
        if(debutYaw - curYaw <= 0.1)
            angleParcouru = curYaw-debutYaw;
        else
            angleParcouru = 2*M_PI - debutYaw + curYaw;

        return angleParcouru > amplitudeMouvements[0];
    }
    else{ // Si on est en train de tourner dans le sens horaire
        double angleParcouru;
        if(debutYaw - curYaw >= -0.1)
            angleParcouru = debutYaw-curYaw;
        else
            angleParcouru = 2*M_PI - curYaw + debutYaw;

        return angleParcouru > amplitudeMouvements[0];
    }
}

void odomCb(const nav_msgs::Odometry& pos){
    curX = pos.pose.pose.position.x;
    curY = pos.pose.pose.position.y;

    tf::Quaternion q(pos.pose.pose.orientation.x,pos.pose.pose.orientation.y,pos.pose.pose.orientation.z,pos.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch;
    m.getRPY(roll,pitch,curYaw);
}

void avancerCb(const std_msgs::Float32& dist){
    if(dist.data<0)return; // On avance, on ne recule pas

    typeMouvements.push_back(0);
    amplitudeMouvements.push_back(dist.data);
}
void tournerCb(const std_msgs::Float32& angle){
    if(angle.data > 2*M_PI || angle.data < -2*M_PI)return; // L'angle à fournir doit être compris entre -2PI et 2PI

    if(angle.data > M_PI){
        typeMouvements.push_back(2);
        amplitudeMouvements.push_back(2*M_PI-angle.data);
    }
    if(angle.data > 0 && angle.data <= M_PI){
        typeMouvements.push_back(1);
        amplitudeMouvements.push_back(angle.data);
    }
    if(angle.data < 0 && angle.data >= -M_PI){
        typeMouvements.push_back(2);
        amplitudeMouvements.push_back(-angle.data);
    }
    if(angle.data < -M_PI){
        typeMouvements.push_back(1);
        amplitudeMouvements.push_back(2*M_PI+angle.data);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle n;
    ros::Publisher pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub_odom = n.subscribe("/odom",1000,odomCb);
    ros::Subscriber sub_avancer = n.subscribe("/avancer",1000,avancerCb);
    ros::Subscriber sub_tourner = n.subscribe("/tourner",1000,tournerCb);
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        geometry_msgs::Twist msg;

        if(!typeMouvements.empty()){ // S'il faut bouger
            if(typeMouvements[0] == 0)
                msg.linear.x = 0.1; // 10cm/sec
            else if(typeMouvements[0] == 1)
                msg.angular.z = 0.2; // 0.2rad/sec
            else
                msg.angular.z = -0.2; // -0.2rad/sec

            if(mvmtFini()){
                typeMouvements.erase(typeMouvements.begin());
                amplitudeMouvements.erase(amplitudeMouvements.begin());
                debutX = curX; debutY = curY; debutYaw = curYaw;
            }
        }
        else{
            debutX = curX; debutY = curY; debutYaw = curYaw;
        }

        pub_cmd.publish(msg); // On envoie la commande
        ros::spinOnce();
        loop_rate.sleep();
    }


    geometry_msgs::Twist msg;
    pub_cmd.publish(msg); // On arrête le robot

    return 0;
}
