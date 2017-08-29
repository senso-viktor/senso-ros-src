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
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <tf/transform_listener.h>

//Global variables
bool start_state = false;
bool gripper_state = false;
bool success;
int IK_mode = 1;
int current_mode = 10;
double x_offset, y_offset, z_offset;
std::vector<double> jointControl_jointValues(3);
std::vector<double> jointControl_lastJointValues{9.99,9.99,9.99};
std::vector<double> positionControl_values(3);
std::vector<double> positionControl_lastValues {9.99,9.99,9.99};
std::vector<double> link_length(2);
std::vector<double> joint_positions(3);

geometry_msgs::Point point;
geometry_msgs::Pose endEffectorPose;

//********************** Functions ********************************//
//-------------------Mode joint control---------------------------//
bool jointModeControll (moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan){

    ROS_INFO("joint mode controll");
    bool success = move_group->plan(my_plan);
    if (!success){
        ROS_ERROR("Could not create a plan!");
        return false;
    }
    move_group->asyncExecute(my_plan);
    //move_group->asyncMove();

}
//--------------------------------------------------------------//

//*******************Mode position control ---------------------//
geometry_msgs::Point getPoseFromTF(std::string source, std::string target) {

    geometry_msgs::Point point;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform(source, target, ros::Time(0), ros::Duration(1));
        listener.lookupTransform(source, target, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_WARN("OSM planner: %s. Can't update pose from TF, for that will be use the latest source point.",
                 ex.what());
    }
    tf::pointTFToMsg(transform.getOrigin(), point);
    return point;

}
void getOffsets(){

    geometry_msgs::Point scaraLink1, scaraLink2, scaraBase;

    point = getPoseFromTF("world","BaseBox");
    x_offset = x_offset + point.x;
    y_offset = y_offset + point.y;
    point = getPoseFromTF("BaseBox","ScaraBase");
    x_offset = x_offset + point.x;
    y_offset = y_offset + point.y;
    point = getPoseFromTF("world","tool0");
    z_offset = point.z;

    scaraBase = getPoseFromTF("world","ScaraLink1");
    scaraLink1 = getPoseFromTF("world","ScaraLink2");
    scaraLink2 = getPoseFromTF("world","GripperBase");
    link_length[0] = sqrt (pow(scaraLink1.x-scaraBase.x,2.0) + pow(scaraLink1.y-scaraBase.y,2.0) );
    link_length[1] = sqrt (pow(scaraLink2.x-scaraLink1.x,2.0) + pow(scaraLink2.y-scaraLink1.y,2.0) );
    //ROS_INFO("final offsets: x_offset=%f y_offset=%f z_offset=%f",x_offset, y_offset, z_offset);
    //ROS_INFO("final arm lenths: arm1=%f arm2=%f",link_length[0], link_length[1]);
    ROS_INFO("Get offsets OK");

}
bool countIK(double x, double y, double z,  int mode){

    //ROS_INFO("input numbers %f %f %f",x,y,z);
    x = x - x_offset;
    y = y - y_offset;
    z = z - z_offset;
    //ROS_INFO("input numbers - offset = %f %f %f",x,y,z);

    double c2 = (pow(x,2.0) + pow(y,2.0) - pow(link_length[0],2.0) - pow(link_length[1],2.0))/(2*link_length[0]*link_length[1]);
    if ((1-c2) > 0.0 ){
        if (mode == 1) {
            joint_positions[1] = atan2(-sqrt(1 - c2), c2);
        }else{
            joint_positions[1] = atan2(sqrt(1 - c2), c2);
        }
        joint_positions[0] =  atan2(y,x) - atan2(link_length[1]*sin(joint_positions[1]),link_length[0] + link_length[1]*cos(joint_positions[1]));
        joint_positions[1] = - joint_positions[1] ; //otocenie kvoli opacnej rotacii klbu
    }else if ((0.00001 >=(1-c2)) && (-0.00001 <= (1-c2))){
        joint_positions[1] = atan2(0,c2);
        joint_positions[0] =  atan2(y,x) - atan2(link_length[1]*sin(joint_positions[1]),link_length[0] + link_length[1]*cos(joint_positions[1]));
        joint_positions[1] = -joint_positions[1];
    }
    else{
        ROS_ERROR("Target is out of range");
        return false;
    }

    if (z<0.05 && z>-0.04){
        joint_positions[2] = -z;
    }else{
        ROS_ERROR("Target is out of range");
        return false;
    }

    ROS_INFO("output joint positions %f %f %f",joint_positions[0],joint_positions[1],joint_positions[2]);
    //ROS_INFO("count IK OK!");
    return true;

}
//-------------------------------------------------------//


//--------------------------------------------------------//
bool valuesChanged(){

    if ((jointControl_jointValues[0] != jointControl_lastJointValues[0]) || (jointControl_jointValues[1] != jointControl_lastJointValues[1]) || (jointControl_jointValues[2] != jointControl_lastJointValues[2])){
        //ROS_ERROR("change!");
        jointControl_lastJointValues[0] = jointControl_jointValues[0];
        jointControl_lastJointValues[1] = jointControl_jointValues[1];
        jointControl_lastJointValues[2] = jointControl_jointValues[2];
        return true;
    }else{
        //ROS_ERROR("no change!");
        return false;
    }
}

bool positionsChanged(){

    if ((positionControl_values[0] != positionControl_lastValues[0]) || (positionControl_values[1] != positionControl_lastValues[1]) || (positionControl_values[2] != positionControl_lastValues[2])){
        //ROS_ERROR("change!");
        positionControl_lastValues[0] = positionControl_values[0];
        positionControl_lastValues[1] = positionControl_values[1];
        positionControl_lastValues[2] = positionControl_values[2];
        return true;
    }else{
        //ROS_ERROR("no change!");
        return false;
    }

}

void sendEndEffectorPose(ros::Publisher *pub,moveit::planning_interface::MoveGroupInterface *move_group){

    geometry_msgs::PoseStamped ws1 = move_group->getCurrentPose();
    endEffectorPose.position.x = ws1.pose.position.x;
    endEffectorPose.position.y = ws1.pose.position.y;
    endEffectorPose.position.z = ws1.pose.position.z;
    ROS_INFO("Publishing pose x=%f y=%f z=%f",endEffectorPose.position.x, endEffectorPose.position.y, endEffectorPose.position.z);
    pub->publish(endEffectorPose);

}
//--------------------------------------------------------//

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

    //ROS_INFO("joint control values callback %f %f %f",pointStamped.point.x,pointStamped.point.y,pointStamped.point.z);
    jointControl_jointValues[0] = pointStamped.point.x;
    jointControl_jointValues[1] = pointStamped.point.y;
    jointControl_jointValues[2] = pointStamped.point.z;
}

void positionControlCallback(const geometry_msgs::Point point){

    //ROS_INFO("position control values callback %f %f %f",point.x,point.y,point.z);
    positionControl_values[0] = point.x;
    positionControl_values[1] = point.y;
    positionControl_values[2] = point.z;
}

#endif //PROJECT_SCARA_MENU_H

