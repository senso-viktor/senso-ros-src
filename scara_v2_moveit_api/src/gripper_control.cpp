//
// Created by viktor on 30/06/17.
//
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Bool.h"
#include "scara_v2_moveit_api/pose_and_gripperState.h"

double positionX,positionY,positionZ;
bool gripper = false;
std_msgs::Bool grip_state;

void gripperCallback (const scara_v2_moveit_api::pose_and_gripperState gripperInfo){
    ROS_INFO("Heard message : gripperState=%d  posX=%f  posY=%f  posZ=%f", gripperInfo.gripperState, gripperInfo.posX,  gripperInfo.posY,  gripperInfo.posZ);
    gripper = gripperInfo.gripperState;
    positionX = gripperInfo.posX;
    positionY = gripperInfo.posY;
    positionZ = gripperInfo.posZ;
    grip_state.data = gripperInfo.gripperState;
}
bool gripper_pick (sensor_msgs::JointState *joint_state){

    const double pos = 0.003;
    joint_state->header.stamp = ros::Time::now();
    joint_state->position[0] = pos;
    return 1;
}
bool gripper_place (sensor_msgs::JointState *joint_state){

    const double pos = 0.0;
    joint_state->header.stamp = ros::Time::now();
    joint_state->position[0] = pos;
    return 1;
}

int main (int argc, char **argv){

    int mode = 0;       // 0 - simulation , 1 - real, 2 - both
    grip_state.data = false;
    int counter = 0;
    ros::init(argc, argv, "gripper_control");
    ros::NodeHandle n;
    ros::NodeHandle n_gripper_pub,n_gripper_sub;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher grip_topic_pub = n_gripper_pub.advertise<std_msgs::Bool>("gripper_state_execution",1);
    ros::Subscriber grip_topic_sub = n_gripper_sub.subscribe("gripper_state",1000,gripperCallback);
    ros::Rate loop_rate(40);

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(1);
    joint_state.position.resize(1);
    joint_state.name[0] = "Joint_Gripper";



    sleep (2);
    ROS_INFO("starting while");

    while (ros::ok()){
        if (mode == 0){
            ROS_INFO_ONCE("simulation mode");
            if (gripper){
                gripper_pick(&joint_state);
            } else {
                gripper_place(&joint_state);
            }
        }else if (mode == 1){
            ROS_INFO_ONCE("real mode");
        }else if (mode == 2) {
            ROS_INFO_ONCE("combined mode");
        }

        joint_pub.publish(joint_state);
        grip_topic_pub.publish(grip_state);

        if (counter == 1000){
            ROS_INFO("\nPublished info:");
            ROS_INFO_STREAM(joint_state);
            ROS_INFO_STREAM(grip_state);
            counter =0;
        }

        ros::spinOnce();
        loop_rate.sleep();
        counter++;

    }

    return 0;
}
