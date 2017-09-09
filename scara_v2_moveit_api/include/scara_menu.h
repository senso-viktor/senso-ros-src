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
#include "std_msgs/Float64.h"
#include "std_msgs/Byte.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <tf/transform_listener.h>
#include "scara_msgs/robot_info.h"

//Global variables
bool start_state = false;
bool teach_start_state = false;
bool button_state = false;
bool success;
bool colisionDetection = false;
bool executionOK = true;
bool initTeachedPositions = true;
bool zeroPositionForTeach = true;
bool pick = false;
bool central_stop = false;
bool moveitState = false;

int IK_mode = 1;
int DEMO_mode = -1;
int teach_mode = -1;
int current_mode = 10;
int count1 = 0;
int last_trajectory_size = -5;
int jointControl_counter = 0, positionControl_counter = 0, demoControl_counter = 0, teachMode_counter = 0, teachModeHand_counter = 0;
double x_offset, y_offset, z_offset;
double max_torque_value = 5.0, torque_value = 0.0;
double maxJointDeviation = 0.1;
double J3_position = 0.0;

std::vector<double> jointControl_jointValues(3);
std::vector<double> jointControl_lastJointValues{9.99,9.99,9.99};
std::vector<double> initJointValues{0.0,0.0,0.0};
std::vector<double> positionControl_values(3);
std::vector<double> positionControl_lastValues {9.99,9.99,9.99};
std::vector<double> link_length(2);
std::vector<double> joint_positions(3);

std::vector<std::vector<double>> desiredJointsDEMO(11, std::vector<double>(3));
std::vector<std::vector<double>> desiredJointsTeach;
std::vector<std::vector<double>> teachPositionsHand;
std::vector<geometry_msgs::Point> desiredPositionsDEMO(11);
std::vector<geometry_msgs::Point> teachPositions;

std_msgs::Bool moveitMode;
std_msgs::Byte selectedMode;
std_msgs::Byte gripper_state;
std_msgs::Int32 centralStop_msg;
std_msgs::Int32 errorCodeMsg;
geometry_msgs::Point point;
geometry_msgs::Point desiredPositions;
geometry_msgs::Pose endEffectorPose;
geometry_msgs::PoseStamped ws1;
geometry_msgs::Pose pos_and_vel;
geometry_msgs::Point acc;
sensor_msgs::JointState currentJointStates;
geometry_msgs::Point currentTeachPoint, lastTeachPoint;



moveit::planning_interface::MoveGroupInterface::Plan my_plan;
moveit::planning_interface::MoveGroupInterface *mg;
ros::Publisher *info_pub;
ros::Publisher *desPos_pub;

//************************************************** Functions *************************************************************//

//This function creates a movement plan and also executes it (used for joint, position and DEMO control)
bool jointModeControll (moveit::planning_interface::MoveGroupInterface *move_group){

    //*****************************************************************************************************************//
    //   This function creates a movement plan and also executes it (used for joint, position and DEMO control)        //
    //   If it fails to create plan it returns false, otherwise it creates plan and executes it in assynchronous mode  //
    //*****************************************************************************************************************//

    ROS_INFO("joint mode controll");
    success = move_group->plan(my_plan);
    if (!success){
        ROS_ERROR("Could not create a plan!");
        return false;
    }
    ROS_INFO("my plan size %d",my_plan.trajectory_.joint_trajectory.points.size());
    move_group->asyncExecute(my_plan);
    //move_group->asyncMove();

}

//This functions server as a support function to function getOffsets
geometry_msgs::Point getPoseFromTF(std::string source, std::string target) {

    //**************************************************************************//
    //   This functions server as a support function to function getOffsets     //
    //   It looks up for transformation frames from the moveit, gets the        //
    //   transforms between source and target frame.                            //
    //   The final transform is returned as a geometry_msgs::Point              //
    //**************************************************************************//

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

//This function serves as a support function to function calculateIK
void getOffsets(){

    //*****************************************************************************//
    //   This function serves as a support function to function calculateIK        //
    //   It uses getPoseFromTF function to get the offsets between each specified  //
    //   link and it calculates the final offsets of the SCARA (in relation to     //
    //   world frame)                                                              //
    //   Then in calculates the exact length for both scara link and stores it     //
    //   in variable link_length                                                   //
    //*****************************************************************************//

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

//This function is used to calculate the Inverse kinematics for the SCARA
bool calculateIK(double x, double y, double z,  int mode, int working_mode, int index){

    //******************************************************************************************//
    //   This function is used to calculate the Inverse kinematics for the SCARA                //
    //   Specification of input variables:                                                      //
    //      - double x,y,z = desired position                                                   //
    //      - int mode = Specifies if the IK should be calculated in clockwise(right-handed)    //
    //                   or anticlockwise(left-handed) mode                                     //
    //      - int working_mode = This parameter is used for the DEMO mode and it serves to     //
    //                    save the calculated IK solution to:                                   //
    //                          1)desiredJointsDEMO     2)desiredJointsTeach                    //
    //      - int index = This parameter is also used for DEMO mode. It specifies to which      //
    //                    index of desiredJointsDEMO should be the solution saved               //
    //******************************************************************************************//

    //ROS_INFO("input numbers %f %f %f",x,y,z);
    x = x - x_offset;
    y = y - y_offset;
    z = -(z - z_offset);
    ROS_INFO("input numbers - offset = %f %f %f",x,y,z);
    //sleep(2);

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

    joint_positions[2] = -0.8*z + 0.024;
    if (joint_positions[2] <=0.0){  //pridat error hlasku...........................................................
        joint_positions[2] =0.0;
    }else if (joint_positions[2] >=0.04){   //pridat error hlasku...................................................
        joint_positions[2] =0.04;
    }

    if (working_mode == 1){             //DEMO
        for (int i=0;i<joint_positions.size();i++)
            desiredJointsDEMO[index][i] = joint_positions[i];
    }else if (working_mode == 2){       //TEACH
        for (int i=0;i<joint_positions.size();i++)
            desiredJointsTeach[index][i] = joint_positions[i];
        }
    return true;

}

//This function checks if the input joint values (from the joint control mode) were changed
bool valuesChanged(){

    //*******************************************************************************************************//
    //   This function checks if the input joint values (from the joint control mode) were changed           //
    //   This function can be used only for joint control mode                                               //
    //   It compares the actual jointControl_jointValues with the jointControl_lastJointValues               //
    //   If between them is a difference the function will return true and also it will insert the           //
    //   jointControl_jointValues to jointControl_lastJointValues                                            //
    //   else it will return false                                                                           //
    //*******************************************************************************************************//

    if ((jointControl_jointValues[0] != jointControl_lastJointValues[0]) || (jointControl_jointValues[1] != jointControl_lastJointValues[1]) || (jointControl_jointValues[2] != jointControl_lastJointValues[2])){
        jointControl_lastJointValues[0] = jointControl_jointValues[0];
        jointControl_lastJointValues[1] = jointControl_jointValues[1];
        jointControl_lastJointValues[2] = jointControl_jointValues[2];
        return true;
    }else{
        //ROS_ERROR("no change!");
        return false;
    }
}

//This function checks if the input position values (from the position control mode) were changed
bool positionsChanged(){

    //*******************************************************************************************************//
    //   This function checks if the input position values (from the position control mode) were changed     //
    //   This function can be used only for position control mode                                            //
    //   It compares the actual positionControl_values with the positionControl_lastValues                   //
    //   If between them is a difference the function will return true and also it will insert the           //
    //   positionControl_values to positionControl_lastValues                                                 //
    //   else it will return false                                                                           //
    //*******************************************************************************************************//

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

//This function sends the current pose of end effector to GUI
void sendEndEffectorPose(ros::Publisher *pub,moveit::planning_interface::MoveGroupInterface *move_group){

    //*****************************************************************//
    //   This function sends the current pose of end effector to GUI   //
    //   It gets the current pose from move_group and the current pose //
    //   is sent via publisher specified at the input of the function  //
    //*****************************************************************//

    ws1 = move_group->getCurrentPose();
    endEffectorPose.position.x = ws1.pose.position.x;
    endEffectorPose.position.y = ws1.pose.position.y;
    endEffectorPose.position.z = ws1.pose.position.z;
    //ROS_INFO("Publishing pose x=%f y=%f z=%f",endEffectorPose.position.x, endEffectorPose.position.y, endEffectorPose.position.z);
    pub->publish(endEffectorPose);

}

//This function send error codes to GUI node
void sendErrorCode(ros::Publisher *pub, int code){

    //**********************************************************************//
    //   This function send error codes to GUI node                         //
    //   Description of the error codes:                                    //
    //      0 - Everything OK! (just at the start of the program)           //
    //      1 - Bad input joint values -> joint values are not in range     //
    //          or they are in colision state (joint control mode)          //
    //      2 - Could not create a good plan (in position control)          //
    //      3 - IK detects colision -> change IK calculation (pos. control) //
    //      4 - IK could not find a suitable solution (pos. control)        //
    //      5 - Cannot solve IK (pos. control)                              //
    //      6 - ......                                                      //
    //**********************************************************************//
    errorCodeMsg.data = code;
    for (int i=0;i<10;i++){
        pub->publish(errorCodeMsg);
    }
}

//This function send the planned poses and velocities to SLRT and SCARA
void sendJointPoses(ros::Publisher *pose_and_vel_pub,ros::Publisher *accel_pub, moveit::planning_interface::MoveGroupInterface::Plan *plan, int i){

    //**************************************************************************************************************************************************//
    //   This function send the planned poses and velocities to SLRT and SCARA                                                                          //
    //   The position respresents joint values (in rad) and orientation respresents the joint velocities in each planned pose from the PLAN             //
    //   If you chose i=999 you will send the current pose for every joint position and speed of SCARA(it is used to sync the code with the SCARA)      //
    //**************************************************************************************************************************************************//

    if ((i != 999) && (i != 998)){
        if (i >= last_trajectory_size){
            i= last_trajectory_size-1;
        }
    }




    if (i == 999){
        pos_and_vel.position.x = currentJointStates.position[0];
        pos_and_vel.position.y =  currentJointStates.position[1];
//        pos_and_vel.position.z =  currentJointStates.position[2];
        pos_and_vel.position.z =  0.04;
        pos_and_vel.orientation.x =  currentJointStates.velocity[0];
        pos_and_vel.orientation.y =  currentJointStates.velocity[1];
//        pos_and_vel.orientation.z =  currentJointStates.velocity[2];
        pos_and_vel.orientation.z =  0.0;
        acc.x = 0.0;
        acc.y = 0.0;
        acc.z = 0.0;
    }else if (i == 998){
        pos_and_vel.position.x = 0.0;
        pos_and_vel.position.y =  0.0;
        pos_and_vel.position.z =  0.04;
        pos_and_vel.orientation.x =  0.0;
        pos_and_vel.orientation.y =  0.0;
        pos_and_vel.orientation.z =  0.0;
        acc.x = 0.0;
        acc.y = 0.0;
        acc.z = 0.0;
    }else{
        pos_and_vel.position.x = plan->trajectory_.joint_trajectory.points[i].positions[0];
        pos_and_vel.position.y = plan->trajectory_.joint_trajectory.points[i].positions[1];

        if (current_mode == 1){
            pos_and_vel.position.z = jointControl_jointValues[2];

        }else if (current_mode == 2){
            pos_and_vel.position.z = joint_positions[2];

        }else if (current_mode == 3){
            if (pick){
                pos_and_vel.position.z =  0.0;
            }else{
                pos_and_vel.position.z =  desiredJointsDEMO[DEMO_mode][2];
            }

        }else if (current_mode == 4){
            if (pick){
                pos_and_vel.position.z =  0.0;
            }else{
                pos_and_vel.position.z =  desiredJointsTeach[count1][2];
            }
        }else if (current_mode == 5){
            if (pick){
                pos_and_vel.position.z =  0.0;
            }else{
                pos_and_vel.position.z =  0.04;
            }
        }else{
            pos_and_vel.position.z = plan->trajectory_.joint_trajectory.points[i].positions[2];
        }


        pos_and_vel.orientation.x = plan->trajectory_.joint_trajectory.points[i].velocities[0];
        pos_and_vel.orientation.y = plan->trajectory_.joint_trajectory.points[i].velocities[1];
        if (current_mode == 1){
            pos_and_vel.orientation.z =  0.0;
        }else if (current_mode == 3){
            if (pick){
                pos_and_vel.orientation.z =  0.0;
            }else{
                pos_and_vel.orientation.z = plan->trajectory_.joint_trajectory.points[i].velocities[2];
            }
        }else{
            pos_and_vel.orientation.z = plan->trajectory_.joint_trajectory.points[i].velocities[2];
        }



        acc.x = plan->trajectory_.joint_trajectory.points[i].accelerations[0];
        acc.y = plan->trajectory_.joint_trajectory.points[i].accelerations[1];
        acc.z = 0.0;
    }

    pose_and_vel_pub->publish(pos_and_vel);
    accel_pub->publish(acc);

}

//This function checks if they are enough publishers on the input topic
void waitForPublishers (ros::Subscriber *sub, int numOfPublishers){

    //*******************************************************************************//
    //   This function checks if they are enough publishers on the input topic       //
    //   If there are enough publishers on the topic it breaks the while and exits   //
    //   the function                                                                //
    //   If they are not enough publishers the function will wait and check for      //
    //   publishers every 500 ms                                                     //
    //*******************************************************************************//

    while (ros::ok()){
        if (sub->getNumPublishers() >= numOfPublishers){
            ROS_INFO("New publisher on topic %s [%d]",sub->getTopic().c_str(),sub->getNumPublishers());
            break;
        }
        ROS_WARN("Not enought publishers on topic %s [%d]",sub->getTopic().c_str(),sub->getNumPublishers());
        usleep(500000);
    }

}

//This function sets the desired poses for DEMO program
void setDesiredPosesDEMO(){

    //************************************************************************************************//
    //   This function sets the desired poses for DEMO program - here you can edit the desired poses  //
    // z values down/up => <0.99 , 1.04>
    //************************************************************************************************//

    //Home position
    desiredPositionsDEMO[0].x = 0.7;
    desiredPositionsDEMO[0].y = 0.57;
    desiredPositionsDEMO[0].z = 1.04;
    //Pick position
    desiredPositionsDEMO[1].x = 0.4;
    desiredPositionsDEMO[1].y = 0.24;
    desiredPositionsDEMO[1].z = 1.04;
    //Work position
    desiredPositionsDEMO[2].x = 0.58;
    desiredPositionsDEMO[2].y = 0.59;
    desiredPositionsDEMO[2].z = 1.04;
    //Place position 1
    desiredPositionsDEMO[3].x = 0.51;
    desiredPositionsDEMO[3].y = 0.87;
    desiredPositionsDEMO[3].z = 1.04;
    //Place position 2
    desiredPositionsDEMO[4].x = 0.51;
    desiredPositionsDEMO[4].y = 0.93;
    desiredPositionsDEMO[4].z = 1.04;
    //Place position 3
    desiredPositionsDEMO[5].x = 0.47;
    desiredPositionsDEMO[5].y = 0.87;
    desiredPositionsDEMO[5].z = 1.04;
    //Place position 4
    desiredPositionsDEMO[6].x = 0.47;
    desiredPositionsDEMO[6].y = 0.93;
    desiredPositionsDEMO[6].z = 1.04;
    //Place position 5
    desiredPositionsDEMO[7].x = 0.43;
    desiredPositionsDEMO[7].y = 0.87;
    desiredPositionsDEMO[7].z = 1.04;
    //Place position 6
    desiredPositionsDEMO[8].x = 0.43;
    desiredPositionsDEMO[8].y = 0.93;
    desiredPositionsDEMO[8].z = 1.04;
    //Place position 7
    desiredPositionsDEMO[9].x = 0.39;
    desiredPositionsDEMO[9].y = 0.87;
    desiredPositionsDEMO[9].z = 1.04;
    //Place position 8
    desiredPositionsDEMO[10].x = 0.39;
    desiredPositionsDEMO[10].y = 0.93;
    desiredPositionsDEMO[10].z = 1.04;

}

//This function compares the current and the desired joint values (used for DEMO program)
bool inPosition(int currentMode) {

    //****************************************************************************************************//
    //   This function compares the current and the desired joint values (used for DEMO program)          //
    //   It returns true if the currentJointStates.position has a value of desiredJointsDEMO +-deathzone  //
    //   else it returns false                                                                            //
    //****************************************************************************************************//

    if ((desiredJointsDEMO[currentMode][0] - maxJointDeviation < currentJointStates.position[0]) &&
        (currentJointStates.position[0] < desiredJointsDEMO[currentMode][0] + maxJointDeviation)) {
        if ((desiredJointsDEMO[currentMode][1] - maxJointDeviation < currentJointStates.position[1]) &&
            (currentJointStates.position[1] < desiredJointsDEMO[currentMode][1] + maxJointDeviation)) {
//            if (((-desiredJointsDEMO[currentMode][2]+0.04) - (maxJointDeviation/10.0 + 0.01) < currentJointStates.position[2]) &&
//                (currentJointStates.position[2] < (-desiredJointsDEMO[currentMode][2]+0.04) + (maxJointDeviation/10.0))) {

                ROS_WARN("!!!!!   In place  !!!!!!");
                for (int i = 0; i < joint_positions.size(); i++) {
                    ROS_ERROR("Desired joint %d value %f", i, desiredJointsDEMO[currentMode][i]);
                    ROS_ERROR("Joint %d value %f", i, currentJointStates.position[i]);
                }
                return true;

//            } else
//                ROS_INFO("J3 not in place %f [%f]", currentJointStates.position[2], desiredJointsDEMO[currentMode][2]);
        } else
            ROS_INFO("J2 not in place %f [%f]", currentJointStates.position[1], desiredJointsDEMO[currentMode][1]);
    } else
        ROS_INFO("J1 not in place %f [%f]", currentJointStates.position[0], desiredJointsDEMO[currentMode][0]);

    return false;
}

//This function compares the current and the last position in teach control -> dokoncit popis...
bool teachPointChanged(){

    static bool initLastPoint = true;

    if (initLastPoint){ //Just once - init last points
        ROS_INFO("Init last teach point");
        lastTeachPoint.x = 9.99;
        lastTeachPoint.y = 9.99;
        lastTeachPoint.z = 9.99;
        initLastPoint = false;
    }
    ROS_INFO("%f %f",currentTeachPoint.x, lastTeachPoint.x);
    ROS_INFO("%f %f",currentTeachPoint.y, lastTeachPoint.z);
    ROS_INFO("%f %f",currentTeachPoint.z, lastTeachPoint.y);
    if ((currentTeachPoint.x != lastTeachPoint.x) || (currentTeachPoint.y != lastTeachPoint.y) || (currentTeachPoint.z != lastTeachPoint.z)){
        ROS_INFO("change detected!");
        lastTeachPoint.x = currentTeachPoint.x;
        lastTeachPoint.y = currentTeachPoint.y;
        lastTeachPoint.z = currentTeachPoint.z;
        return true;
    }else{
        ROS_INFO("no change detected!");
        return false;
    }

}

//shows the teached positions and initializes vector for calculated joint values -> dokoncit popis...
void showAndInitVector(){

    ROS_INFO("size of teached points %d",teachPositions.size());
    for (int i =0;i<teachPositions.size();i++){
        ROS_INFO("%d x=%f y=%f z=%f",i, teachPositions[i].x, teachPositions[i].y, teachPositions[i].z);
    }
    desiredJointsTeach.resize(teachPositions.size());
    for (int i=0; i< desiredJointsTeach.size(); i++){
        desiredJointsTeach[i].resize(3);
    }
    //ROS_INFO("desireJointsTeach size = %d x %d",desiredJointsTeach.size(), desiredJointsTeach[0].size());

}

//This function compares the current and the desired joint values (used for DEMO program)
bool inPositionTeach(int currentMode, int type) {

    //****************************************************************************************************//
    //   This function compares the current and the desired joint values (used for DEMO program)          //
    //   It returns true if the currentJointStates.position has a value of desiredJointsDEMO +-deathzone  //
    //   else it returns false                                                                            //
    //   type respresents the usage type of this function (1-teach mode, 2-hand teach mode)               //
    //****************************************************************************************************//

    if (type == 1){
        if ((desiredJointsTeach[currentMode][0] - maxJointDeviation < currentJointStates.position[0]) &&
            (currentJointStates.position[0] < desiredJointsTeach[currentMode][0] + maxJointDeviation)) {
            if ((desiredJointsTeach[currentMode][1] - maxJointDeviation < currentJointStates.position[1]) &&
                (currentJointStates.position[1] < desiredJointsTeach[currentMode][1] + maxJointDeviation)) {
//                if ((-desiredJointsTeach[currentMode][2] - maxJointDeviation < currentJointStates.position[2]) &&
//                    (currentJointStates.position[2] < -desiredJointsTeach[currentMode][2] + maxJointDeviation)) {

                    ROS_WARN("!!!!!   In place  !!!!!!");
                    for (int i = 0; i < joint_positions.size(); i++) {
                        ROS_ERROR("Desired joint %d value %f", i, desiredJointsTeach[currentMode][i]);
                        ROS_ERROR("Joint %d value %f", i, currentJointStates.position[i]);
                    }
                    return true;
//                }
            }
        }

//                } else
//                    ROS_INFO("J3 not in place %f [%f]", currentJointStates.position[2], desiredJointsTeach[currentMode][2]);
//            } else
//                ROS_INFO("J2 not in place %f [%f]", currentJointStates.position[1], desiredJointsTeach[currentMode][1]);
//        } else
//            ROS_INFO("J1 not in place %f [%f]", currentJointStates.position[0], desiredJointsTeach[currentMode][0]);

        return false;

    }else if (type == 2) {
        if ((teachPositionsHand[currentMode][0] - maxJointDeviation < currentJointStates.position[0]) &&
            (currentJointStates.position[0] < teachPositionsHand[currentMode][0] + maxJointDeviation)) {
            if ((teachPositionsHand[currentMode][1] - maxJointDeviation < currentJointStates.position[1]) &&
                (currentJointStates.position[1] < teachPositionsHand[currentMode][1] + maxJointDeviation)) {
//                if ((teachPositionsHand[currentMode][2] - maxJointDeviation < currentJointStates.position[2]) &&
//                    (currentJointStates.position[2] < teachPositionsHand[currentMode][2] + maxJointDeviation)) {

                    ROS_WARN("!!!!!   In place  !!!!!!");
                    for (int i = 0; i < joint_positions.size(); i++) {
                        ROS_ERROR("Desired joint %d value %f", i, teachPositionsHand[currentMode][i]);
                        ROS_ERROR("Joint %d value %f", i, currentJointStates.position[i]);
                    }
                    return true;

//                }
            }
        }
        return false;
    }
//                else
//                    //ROS_INFO("J3 not in place %f [%f]", currentJointStates.position[2], teachPositionsHand[currentMode][2]);
//            } else
//                //ROS_INFO("J2 not in place %f [%f]", currentJointStates.position[1], teachPositionsHand[currentMode][1]);
//        } else
//            //ROS_INFO("J1 not in place %f [%f]", currentJointStates.position[0], teachPositionsHand[currentMode][0]);
//
//        return false;
//
//    }

}

//This function shows the teached joint values
void showTeachedJointValues (){

    ROS_INFO("size of teached points %d x %d",teachPositionsHand.size(), teachPositionsHand[0].size());

    for (int i=0;i<teachPositionsHand.size();i++){
        ROS_ERROR("[%d]: J1=%f J2=%f J3=%f",i, teachPositionsHand[i][0],teachPositionsHand[i][1],teachPositionsHand[i][2]);
    }
    sleep(2);
}

void initMatlab(ros::Publisher *pub_gripper, ros::Publisher *pub_centralStop, ros::Publisher *pub_pose_and_vel,ros::Publisher *pub_accel, moveit::planning_interface::MoveGroupInterface::Plan *initPlan){

    gripper_state.data = 0;
    centralStop_msg.data = 0;

    pub_gripper->publish(gripper_state);
    pub_centralStop->publish(centralStop_msg);
    sendJointPoses(pub_pose_and_vel, pub_accel, initPlan, 998);
    usleep(200000);


}

//This function sends the desired position to GUI
void sendPositionToGUI(double input_x, double input_y, double input_z){

    desiredPositions.x = input_x;
    desiredPositions.y = input_y;
    desiredPositions.z = input_z;

    desPos_pub->publish(desiredPositions);

}


//******************************************************************************************************************************//




//*********************** Callbacks *******************************//
void modeSelectCallback(const std_msgs::Int32 mode){

    //ROS_INFO("Mode select callback");
    current_mode = mode.data;
}                             //GUI -> MENU

void startStateCallback(const std_msgs::Bool startState_msg){

    //ROS_INFO("Start callback");
    start_state = startState_msg.data;
}                    //GUI -> MENU

void gripperStateCallback(const std_msgs::Bool gripperState_msg){

    //ROS_INFO("Gripper callback");
    if (gripperState_msg.data)
        gripper_state.data = 1;
    else
        gripper_state.data = 0;
}                //GUI -> MENU

void jointControlCallback(const geometry_msgs::PointStamped pointStamped){

    //ROS_INFO("joint control values callback %f %f %f",pointStamped.point.x,pointStamped.point.y,pointStamped.point.z);
    jointControl_jointValues[0] = pointStamped.point.x;
    jointControl_jointValues[1] = pointStamped.point.y;
    if (pointStamped.point.z > 0.04)
        jointControl_jointValues[2] = 0.04;
    else if (pointStamped.point.z < 0.0)
        jointControl_jointValues[2] = 0.0;
    else
        jointControl_jointValues[2] = pointStamped.point.z;

}       //GUI -> MENU

void positionControlCallback(const geometry_msgs::Point point){

    //ROS_INFO("position control values callback %f %f %f",point.x,point.y,point.z);
    positionControl_values[0] = point.x;
    positionControl_values[1] = point.y;
    positionControl_values[2] = point.z;

    if (teach_mode == 0){
        currentTeachPoint = point;
    }

}                  //GUI -> MENU

void teachModeCallback(const std_msgs::Int32 teachMode){

    //ROS_INFO("teach mode callback %d",teachMode.data);
    teach_mode = teachMode.data;

}                         //GUI -> MENU

void teachModeStartStateCallback(const std_msgs::Bool startState_teachMode){

    //ROS_INFO("teach start state callback");
    teach_start_state = startState_teachMode.data;

}     //GUI -> MENU

void centralStopCallback(const std_msgs::Int32 centralStopState){

    //ROS_INFO("teach central stop callback");
    central_stop = centralStopState.data;

}                //GUI -> MENU

void setTorqueCallback(const std_msgs::Float64 torqueValue){

    ROS_INFO("set torque callback");
    max_torque_value = torqueValue.data;
    ROS_INFO("set torque value to %f",torqueValue.data);


}                     //GUI -> MENU

void setPrecisionCallback(const std_msgs::Float64 precisionValue){

    ROS_INFO("Precision value changed to %f",precisionValue.data);
    maxJointDeviation = precisionValue.data;

}                //GUI -> MENU

void setVelCallback(const std_msgs::Float64 velocityValue){

    ROS_INFO("set velocity callback");
    mg->setMaxVelocityScalingFactor(velocityValue.data);
    ROS_INFO("max velocity set to %f",velocityValue.data);


}                      //GUI -> MENU

void setAccCallback(const std_msgs::Float64 accelerationValue){

    ROS_INFO("set acceleration callback");
    mg->setMaxAccelerationScalingFactor(accelerationValue.data);
    ROS_INFO("max acceleration set to %f",accelerationValue.data);

}                  //GUI -> MENU

void setPlanTimeCallback(const std_msgs::Float64 planTimeValue){

    ROS_INFO("set plan time callback");
    mg->setPlanningTime(planTimeValue.data);
    ROS_INFO("Planning time set to %f",planTimeValue.data);


}                 //GUI -> MENU

void setNumOfAttemptsCallback(const std_msgs::Int32 numOfAttemptsValue){

    ROS_INFO("set number of attempts callback");
    mg->setNumPlanningAttempts(numOfAttemptsValue.data);
    ROS_INFO("set number of attempts %d",numOfAttemptsValue.data);


}         //GUI -> MENU

void moveitModeCallback(const std_msgs::Bool mode){

    moveitState = mode.data;

}                              //GUI -> MENU


void infoCallback(const std_msgs::Bool mode){

    ROS_INFO("INFO callback");

    scara_msgs::robot_info robotInfo_msg;
    std::string robot_model = "scara_arm";
    std::string str;
    robotInfo_msg.robot_model = robot_model;
    robotInfo_msg.reference_frame = mg->getPlanningFrame().c_str();
    robotInfo_msg.efector_link = mg->getEndEffectorLink().c_str();
    std::vector< std::string> activeJoints = mg->getActiveJoints();
    for (int i=0;i<activeJoints.size();i++){
        str.append(activeJoints[i]);
        str.append(" , ");
    }
    robotInfo_msg.active_joints = str;
    ROS_INFO_STREAM(str);

    geometry_msgs::PoseStamped currentPose = mg->getCurrentPose();

    robotInfo_msg.position_x = currentPose.pose.position.x;
    robotInfo_msg.position_y = currentPose.pose.position.y;
    robotInfo_msg.position_z = currentPose.pose.position.z;

    robotInfo_msg.orientation_x = currentPose.pose.orientation.x;
    robotInfo_msg.orientation_y = currentPose.pose.orientation.y;
    robotInfo_msg.orientation_z = currentPose.pose.orientation.z;
    robotInfo_msg.orientation_w = currentPose.pose.orientation.w;

    info_pub->publish(robotInfo_msg);

    activeJoints.clear();

}                              //GUI -> MENU

void buttonStateCallback(const std_msgs::Byte buttonState){

    //ROS_INFO("button state callback");
    if (buttonState.data == 1){
        button_state = true;

    }else{
        button_state = false;
    }
    //ROS_INFO_STREAM(button_state);

}                      //REAL SCARA -> MENU

void jointStatesCallback (const sensor_msgs::JointState jointStates){

    currentJointStates = jointStates;
}            //REAL SCARA -> MENU




#endif //PROJECT_SCARA_MENU_H

