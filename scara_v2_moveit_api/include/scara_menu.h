//
// Created by viktordluhos on 28/08/17.
//

#ifndef PROJECT_SCARA_MENU_H
#define PROJECT_SCARA_MENU_H

#include <ros/ros.h>
#include "cstdio"
#include "stdio.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/PointStamped.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"

//Global variables
int current_mode = 10;
bool start_state = false;
bool gripper_state = false;

std::vector<double> jointControl_jointValues(3);
std::vector<double> jointControl_lastJointValues{9.99,9.99,9.99};


//********************** Functions ********************************//
bool jointModeControll (moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan){

    ROS_INFO("joint mode controll");
    bool success = move_group->plan(my_plan);
    if (!success){
        ROS_ERROR("Could not create a plan!");
        return false;
    }
    move_group->asyncExecute(my_plan);
    move_group->asyncMove();

}



//*********************** Callbacks *******************************//
void modeSelectCallback(const std_msgs::Int32 mode){

    //ROS_INFO("Mode select callback");
    current_mode = mode.data;
}

void startStateCallback(const std_msgs::Bool startState_msg){

    //ROS_INFO("Start callback");
    start_state = startState_msg.data;
}

void gripperStateCallback(const std_msgs::Bool gripperState_msg){

    //ROS_INFO("Gripper callback");
    gripper_state = gripperState_msg.data;
}

void jointControlCallback(const geometry_msgs::PointStamped pointStamped){

    //ROS_INFO("joint control values callback");
    jointControl_jointValues[0] = pointStamped.point.x;
    jointControl_jointValues[1] = pointStamped.point.y;
    jointControl_jointValues[2] = pointStamped.point.z;
}

bool valuesChanged(){

    if (jointControl_jointValues[0] == jointControl_lastJointValues[0])
        return false;
    else if (jointControl_jointValues[1] == jointControl_lastJointValues[1])
        return false;
    else if (jointControl_jointValues[2] == jointControl_lastJointValues[2])
        return false;
    else{
        jointControl_lastJointValues[0] = jointControl_jointValues[0];
        jointControl_lastJointValues[1] = jointControl_jointValues[1];
        jointControl_lastJointValues[2] = jointControl_jointValues[2];
        return true;
    }

}
#endif //PROJECT_SCARA_MENU_H

