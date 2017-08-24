//
// Created by viktor on 17/08/17.
//

//                                      Description                                     //
//Tryout of the m-file : ROS_Simulink_communication_node.m                              //
//The Simulink is subscribing to SLSC_pose(Point) and publishes sensor_msgs::JointState //
//The m-file will later on communicate with the simulink                                //
//**************************************************************************************//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"


#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("SLSC_pose", 1000);
    ros::Rate loop_rate(5);
    geometry_msgs::Point point;

    int count = 0;
    while (ros::ok())
    {
        point.x += 0.02;
        point.y += 0.04;
        point.z += 0.06;

        chatter_pub.publish(point);
        ROS_INFO("Published : J1=%f J2=%f J3=%f",point.x,point.y,point.z);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }


    return 0;
}

