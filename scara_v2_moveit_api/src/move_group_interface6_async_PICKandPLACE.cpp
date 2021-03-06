//
// Created by viktor on 04/07/17.
//

//
// Created by viktor on 09/05/17.
//

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include <moveit/robot_state/conversions.h>
#include <geometric_shapes/solid_primitive_dims.h>
//msgs
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "scara_v2_moveit_api/pose_and_gripperState.h"
#include <sstream>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include <scara_v2_moveit_api/SimpleService.h>
#include <roscpp/GetLoggers.h>
#include <math.h>

using namespace std;






const double DEG2RAD=0.01745329252;
const double RAD2DEG=57.295779513;
std::vector<std::vector<double>> joint_group_positions(13, std::vector<double>(3));
std::vector<std::vector<double>> defaultPositions(11, std::vector<double>(3));
std::vector<std::vector<double>> defaultCartesianPosition(11, std::vector<double>(7));
std::vector<std::vector<double>> anglesFromIK(11, std::vector<double>(4));
std::vector<double> joint_group_position(3);
bool executionOK = false;
bool gripperExecutionState = false;
std::vector<geometry_msgs::Pose> waypoints(2);
int counter = 0;
double torque_value = 0.0;
double max_torque_value = 1.5;
bool threadExecution = false;
bool displayMode = true;
bool rtService = true;
int waitingTime = 0;
bool stop = false;
moveit::planning_interface::MoveGroupInterface* mg;




void setDesiredAngles (){
    ROS_INFO("VECTOR SIZE %d x %d \n", joint_group_positions.size(),joint_group_positions[0].size());

    //Home position
    joint_group_positions[0][0] = 0;
    joint_group_positions[0][1] = 0;
    joint_group_positions[0][2] = 0.00;

    //Pick position
    joint_group_positions[1][0] = -0.44*RAD2DEG;
    joint_group_positions[1][1] = 1.30*RAD2DEG;
    joint_group_positions[1][2] = 0.01;
    //work position
    joint_group_positions[2][0] = -0.68*RAD2DEG;
    joint_group_positions[2][1] = -1.45*RAD2DEG;
    joint_group_positions[2][2] = 0.0;
    //place position 1
    joint_group_positions[3][0] = 0.21*RAD2DEG;
    joint_group_positions[3][1] = -1.15*RAD2DEG;
    joint_group_positions[3][2] = 0.02;

    //Pick position
    joint_group_positions[4][0] = -0.44*RAD2DEG;
    joint_group_positions[4][1] = 1.30*RAD2DEG;
    joint_group_positions[4][2] = 0.01;
    //work position
    joint_group_positions[5][0] = -0.68*RAD2DEG;
    joint_group_positions[5][1] = -1.45*RAD2DEG;
    joint_group_positions[5][2] = 0.0;
    //place position 2
    joint_group_positions[6][0] = 0.54*RAD2DEG;
    joint_group_positions[6][1] = -0.67*RAD2DEG;
    joint_group_positions[6][2] = 0.03;

    //Pick position
    joint_group_positions[7][0] = -0.44*RAD2DEG;
    joint_group_positions[7][1] = 1.30*RAD2DEG;
    joint_group_positions[7][2] = 0.01;
    //work position
    joint_group_positions[8][0] = -0.68*RAD2DEG;
    joint_group_positions[8][1] = -1.45*RAD2DEG;
    joint_group_positions[8][2] = 0.0;
    //place position 3
    joint_group_positions[9][0] = 0.26*RAD2DEG;
    joint_group_positions[9][1] = -1.55*RAD2DEG;
    joint_group_positions[9][2] = 0.03;

    //Pick position
    joint_group_positions[10][0] = -0.44*RAD2DEG;
    joint_group_positions[10][1] = 1.30*RAD2DEG;
    joint_group_positions[10][2] = 0.01;
    //work position
    joint_group_positions[11][0] = -0.68*RAD2DEG;
    joint_group_positions[11][1] = -1.45*RAD2DEG;
    joint_group_positions[11][2] = 0.0;
    //place position 4
    joint_group_positions[12][0] = 0.49*RAD2DEG;
    joint_group_positions[12][1] = -1.28*RAD2DEG;
    joint_group_positions[12][2] = 0.03;
    ROS_INFO("Vector filled up");

}
bool setPositions(bool showAngles){

    ROS_INFO("VECTOR SIZE %d x %d \n", defaultPositions.size(),defaultPositions[0].size());
    //Home position
    defaultPositions[0][0] = 0;
    defaultPositions[0][1] = 0;
    defaultPositions[0][2] = 0.00;
    //Pick position
    defaultPositions[1][0] = -0.44*RAD2DEG;
    defaultPositions[1][1] = 1.30*RAD2DEG;
    defaultPositions[1][2] = 0.01;
    //work position
    defaultPositions[2][0] = -0.68*RAD2DEG;
    defaultPositions[2][1] = -1.45*RAD2DEG;
    defaultPositions[2][2] = 0.0;
    //place position 1
    defaultPositions[3][0] = 0.21*RAD2DEG;
    defaultPositions[3][1] = -1.15*RAD2DEG;
    defaultPositions[3][2] = 0.02;
    //place position 2
    defaultPositions[4][0] = 0.54*RAD2DEG;
    defaultPositions[4][1] = -0.67*RAD2DEG;
    defaultPositions[4][2] = 0.03;
    //place position 3
    defaultPositions[5][0] = 0.23*RAD2DEG;
    defaultPositions[5][1] = -1.25*RAD2DEG;
    defaultPositions[5][2] = 0.03;
    //place position 4
    defaultPositions[6][0] = 0.50*RAD2DEG;
    defaultPositions[6][1] = -0.90*RAD2DEG;
    defaultPositions[6][2] = 0.03;
    //place position 5
    defaultPositions[7][0] = 0.21*RAD2DEG;
    defaultPositions[7][1] = -1.45*RAD2DEG;
    defaultPositions[7][2] = 0.03;
    //place position 6
    defaultPositions[8][0] = 0.45*RAD2DEG;
    defaultPositions[8][1] = -1.12*RAD2DEG;
    defaultPositions[8][2] = 0.03;
    //place position 7
    defaultPositions[9][0] = 0.26*RAD2DEG;
    defaultPositions[9][1] = -1.55*RAD2DEG;
    defaultPositions[9][2] = 0.03;
    //place position 8
    defaultPositions[10][0] = 0.49*RAD2DEG;
    defaultPositions[10][1] = -1.28*RAD2DEG;
    defaultPositions[10][2] = 0.03;
    if (showAngles) {
        ROS_INFO("VECTOR SIZE %d x %d \n", defaultPositions.size(), defaultPositions[0].size());
        ROS_INFO("default positions:");
        for (int i = 0; i < defaultPositions.size(); i++) {
            if (i == 0) {
                ROS_INFO("HOME pos:     x=%f  y=%f  z=%f", defaultPositions[i][0],
                         defaultPositions[i][1], defaultPositions[i][2]);
            } else if (i == 1) {
                ROS_INFO("PICK pos:     x=%f  y=%f  z=%f", defaultPositions[i][0],
                         defaultPositions[i][1], defaultPositions[i][2]);
            } else if (i == 2) {
                ROS_INFO("PLACE pos:    x=%f  y=%f  z=%f", defaultPositions[i][0],
                         defaultPositions[i][1], defaultPositions[i][2]);
            } else {
                ROS_INFO("PLACE pos %d: x=%f  y=%f  z=%f", i - 2, defaultPositions[i][0],
                         defaultPositions[i][1], defaultPositions[i][2]);
            }
        }
       // sleep(2);
    }
    return true;
}
bool setPositionsFromIK(bool showAngles){

    setPositions(true);

    for (int i =0;i<defaultPositions.size();i++){
        for (int j = 0;j<3;j++){
            if (j == 2){
                defaultPositions[i][j] = anglesFromIK[i][j];
            }else {
                defaultPositions[i][j] = (ceil(anglesFromIK[i][j] * 100) / 100) * RAD2DEG;
            }
        }
    }
    for (int i=0;i<defaultPositions.size();i++){
        ROS_INFO("final angle %d is %f %f %f",i,defaultPositions[i][0],defaultPositions[i][1],defaultPositions[i][2]);
    }
    ROS_INFO("Press any key to continue...");
    getchar();
    return true;


}
bool setCartesianPositions(bool showPositions){

//    defaultCartesianPosition.resize(11);
//    defaultCartesianPosition[0].resize(3);
    //Home position
    //xyz
    defaultCartesianPosition[0][0] = 0.704;
    defaultCartesianPosition[0][1] = 0.58;
    defaultCartesianPosition[0][2] = 1.02;
    //rpy+w
    defaultCartesianPosition[0][3] = 0.707;
    defaultCartesianPosition[0][4] = 0.707;
    defaultCartesianPosition[0][5] = -0.00028;
    defaultCartesianPosition[0][6] = -0.00028;

    //Pick position
    //xyz
    defaultCartesianPosition[1][0] = 0.4;
    defaultCartesianPosition[1][1] = 0.24;
    defaultCartesianPosition[1][2] = 1.01;
    //rpy+w
    defaultCartesianPosition[1][3] = 0.99;
    defaultCartesianPosition[1][4] = -0.08;
    defaultCartesianPosition[1][5] = -0.00077;
    defaultCartesianPosition[1][6] = 0.00012;

    //work position 0.57143; 0.6028; 1.02
    defaultCartesianPosition[2][0] = 0.58;
    defaultCartesianPosition[2][1] = 0.59;
    defaultCartesianPosition[2][2] = 1.04;
    //rpy
    defaultCartesianPosition[2][3] = 0.38;
    defaultCartesianPosition[2][4] = 0.92;
    defaultCartesianPosition[2][5] = 0.00059;
    defaultCartesianPosition[2][6] = -0.00061;
    //another orientation
//            defaultCartesianPosition[2][3] = 0.89;
//            defaultCartesianPosition[2][4] = 0.44;
//            defaultCartesianPosition[2][5] = -0.0005;
//            defaultCartesianPosition[2][6] = 0.0006;
    //place position 1 : 0.51; 0.87; 0.9735
    //xyz
    defaultCartesianPosition[3][0] = 0.51;
    defaultCartesianPosition[3][1] = 0.87;
    defaultCartesianPosition[3][2] = 1.01;
    //rpy
    defaultCartesianPosition[3][3] = 0.11;
    defaultCartesianPosition[3][4] = 0.99;
    defaultCartesianPosition[3][5] = 0.00015;
    defaultCartesianPosition[3][6] = -0.00072;
    //place position 2 : 0.51; 0.93; 0.9735
    //xyz
    defaultCartesianPosition[4][0] = 0.51;
    defaultCartesianPosition[4][1] = 0.93;
    defaultCartesianPosition[4][2] = 1.01;
    //rpy
    defaultCartesianPosition[4][3] = 0.18;
    defaultCartesianPosition[4][4] = 0.98;
    defaultCartesianPosition[4][5] = -0.00013;
    defaultCartesianPosition[4][6] = -0.00053;
    //place position 3 : 0.47; 0.87; 0.9735
    //xyz
    defaultCartesianPosition[5][0] = 0.47;
    defaultCartesianPosition[5][1] = 0.87;
    defaultCartesianPosition[5][2] = 1.01;
    //rpy
    defaultCartesianPosition[5][3] = 0.04;
    defaultCartesianPosition[5][4] = 0.99;
    defaultCartesianPosition[5][5] = 0.00018;
    defaultCartesianPosition[5][6] = 0.00018;
    //place position 4 : 0.47; 0.93; 0.9735
    //xyz
    defaultCartesianPosition[6][0] = 0.47;
    defaultCartesianPosition[6][1] = 0.93;
    defaultCartesianPosition[6][2] = 1.01;
    //rpy
    defaultCartesianPosition[6][3] = 0.08;
    defaultCartesianPosition[6][4] = 0.99;
    defaultCartesianPosition[6][5] = 0;
    defaultCartesianPosition[6][6] = -0.00064;
    //place position 5 : 0.43; 0.87; 0.9735
    //xyz
    defaultCartesianPosition[7][0] = 0.43;
    defaultCartesianPosition[7][1] = 0.87;
    defaultCartesianPosition[7][2] = 1.01;
    //rpy
    defaultCartesianPosition[7][3] = -0.04;
    defaultCartesianPosition[7][4] = 0.999;
    defaultCartesianPosition[7][5] = 0.00027;
    defaultCartesianPosition[7][6] = -0.0008;
    //place position 6 : 0.43; 0.93; 0.9735
    //xyz
    defaultCartesianPosition[8][0] = 0.43;
    defaultCartesianPosition[8][1] = 0.93;
    defaultCartesianPosition[8][2] = 1.01;
    //rpy
    defaultCartesianPosition[8][3] = 0.004;
    defaultCartesianPosition[8][4] = 0.999;
    defaultCartesianPosition[8][5] = 0;
    defaultCartesianPosition[8][6] = -0.0072;
    //place position 7 : 0.39; 0.87; 0.9735
    //xyz
    defaultCartesianPosition[9][0] = 0.39;
    defaultCartesianPosition[9][1] = 0.88;
    defaultCartesianPosition[9][2] = 1.01;
    //rpy
    defaultCartesianPosition[9][3] = -0.12;
    defaultCartesianPosition[9][4] = 0.99;
    defaultCartesianPosition[9][5] = 0.00028;
    defaultCartesianPosition[9][6] = -0.00084;
    //place position 8 : 0.39; 0.87; 0.9735
    //xyz
    defaultCartesianPosition[10][0] = 0.39;
    defaultCartesianPosition[10][1] = 0.93;
    defaultCartesianPosition[10][2] = 1.01;
    //rpy
    defaultCartesianPosition[10][3] = -0.095;
    defaultCartesianPosition[10][4] = 0.995;
    defaultCartesianPosition[10][5] = 0;
    defaultCartesianPosition[10][6] = -0.00078;

    if (showPositions){
        ROS_INFO("VECTOR SIZE %d x %d \n", defaultCartesianPosition.size(),defaultCartesianPosition[0].size());
        ROS_INFO("default positions:");
        for (int i=0;i<defaultCartesianPosition.size();i++){
            if (i==0){
                ROS_INFO("HOME pos:     x=%f  y=%f  z=%f",defaultCartesianPosition[i][0],defaultCartesianPosition[i][1],defaultCartesianPosition[i][2]);
            }else if (i==1){
                ROS_INFO("PICK pos:     x=%f  y=%f  z=%f",defaultCartesianPosition[i][0],defaultCartesianPosition[i][1],defaultCartesianPosition[i][2]);
            }else if (i==2){
                ROS_INFO("PLACE pos:    x=%f  y=%f  z=%f",defaultCartesianPosition[i][0],defaultCartesianPosition[i][1],defaultCartesianPosition[i][2]);
            }else{
                ROS_INFO("PLACE pos %d: x=%f  y=%f  z=%f",i-2,defaultCartesianPosition[i][0],defaultCartesianPosition[i][1],defaultCartesianPosition[i][2]);
            }
        }
        sleep(2);
    }

    return 1;
}
void getAnglesFromIK(moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan){

    geometry_msgs::Pose position;
    bool success;
    int size;

    ROS_INFO("def %d",defaultCartesianPosition.size());
    for (int i = 0;i<defaultCartesianPosition.size();i++){

        position.position.x = defaultCartesianPosition[i][0];
        position.position.y = defaultCartesianPosition[i][1];
        position.position.z = defaultCartesianPosition[i][2];
        position.orientation.x = defaultCartesianPosition[i][3];
        position.orientation.y = defaultCartesianPosition[i][4];
        position.orientation.z = defaultCartesianPosition[i][5];
        position.orientation.w = defaultCartesianPosition[i][6];

        if (move_group->setApproximateJointValueTarget(position, "tool0")) {
            ROS_WARN("found IK solution");
        } else
            ROS_WARN("only aproximate IK solution");

        success = static_cast<bool>(move_group->plan(my_plan));
        // ROS_INFO_STREAM("PLAN:" << success);
        if(success){
            size=my_plan.trajectory_.joint_trajectory.points.size();
            ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points[size-1]);

            anglesFromIK[i][0] = my_plan.trajectory_.joint_trajectory.points[size-1].positions[0];
            anglesFromIK[i][1] = my_plan.trajectory_.joint_trajectory.points[size-1].positions[1];
            anglesFromIK[i][2] = my_plan.trajectory_.joint_trajectory.points[size-1].positions[2];
            anglesFromIK[i][3] = 1.0;
        } else{
            ROS_ERROR("not success");
            anglesFromIK[i][0] = 0.0;
            anglesFromIK[i][1] = 0.0;
            anglesFromIK[i][2] = 0.0;
            anglesFromIK[i][3] = 0.0;
        }
        //sleep(2);
    }
    for (int i=0;i<anglesFromIK.size();i++){
        ROS_INFO("final angle %d is %f %f %f [%f]",i,anglesFromIK[i][0],anglesFromIK[i][1],anglesFromIK[i][2], anglesFromIK[i][3]);
    }
    ROS_INFO("Press any key to continue...");
    getchar();

}
void jointModeControll (moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan, int mode, int number_of_place_position, bool asyncMode){

    ROS_INFO("joint mode controll");
    bool success;
    if (mode == 0){         //Home position
        //ROS_INFO("joint mode controll 0");
        //ROS_INFO("defaultPositions %d x %d ",defaultPositions.size(),defaultPositions[0].size());
        //ROS_INFO("joint_group_position %d ",joint_group_position.size());
        joint_group_position[0] = defaultPositions[0][0]*DEG2RAD;  // radians
        joint_group_position[1] = defaultPositions[0][1]*DEG2RAD;
        joint_group_position[2] = defaultPositions[0][2];
        ROS_INFO("moving to home");
    }else if (mode == 1){      //pick position
        joint_group_position[0] = defaultPositions[1][0]*DEG2RAD;  // radians
        joint_group_position[1] = defaultPositions[1][1]*DEG2RAD;
        joint_group_position[2] = defaultPositions[1][2];
        ROS_INFO("moving to pick");
    }else if (mode == 2){       //work position
        joint_group_position[0] = defaultPositions[2][0]*DEG2RAD;  // radians
        joint_group_position[1] = defaultPositions[2][1]*DEG2RAD;
        joint_group_position[2] = defaultPositions[2][2];
        ROS_INFO("moving to work");
    }
    else if (mode == 3){        //place position
        if (number_of_place_position == 1){
            joint_group_position[0] = defaultPositions[3][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[3][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[3][2];
        }else if (number_of_place_position == 2){
            joint_group_position[0] = defaultPositions[4][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[4][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[4][2];
        }else if (number_of_place_position == 3){
            joint_group_position[0] = defaultPositions[5][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[5][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[5][2];
        }else if (number_of_place_position == 4){
            joint_group_position[0] = defaultPositions[6][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[6][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[6][2];
        }else if (number_of_place_position == 5){
            joint_group_position[0] = defaultPositions[7][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[7][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[7][2];
        }else if (number_of_place_position == 6){
            joint_group_position[0] = defaultPositions[8][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[8][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[8][2];
        }else if (number_of_place_position == 7){
            joint_group_position[0] = defaultPositions[9][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[9][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[9][2];
        }else if (number_of_place_position == 8){
            joint_group_position[0] = defaultPositions[10][0]*DEG2RAD;  // radians
            joint_group_position[1] = defaultPositions[10][1]*DEG2RAD;
            joint_group_position[2] = defaultPositions[10][2];
        }else {
            ROS_INFO("NOT DEFINED position");
        }
        ROS_INFO("moving to place");
    }else{
        ROS_INFO("NOT DEFINED mode");
    }
    //ROS_INFO("Desired joint values: %f  %f  %f",joint_group_position[0],joint_group_position[1],joint_group_position[2]);
    move_group->setJointValueTarget(joint_group_position);
    success = static_cast<bool>(move_group->plan(my_plan));
    ROS_INFO_NAMED("Visualizing plan (joint space goal) %s", success ? "GOOD" : "FAILED");
    if (asyncMode){
        move_group->asyncExecute(my_plan);
        //move_group->asyncMove();
    }else{
        move_group->execute(my_plan);
        move_group->move();
    }
}
void positionControll (moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan, int mode, int number_of_place_position, bool asyncMode){

    bool success;
    int numberOfAttempts =0;
    geometry_msgs::Pose position;

    position.orientation.x = 0.0;
    position.orientation.y = 0.0;
    position.orientation.z = 0.0;
    position.orientation.w = 0.0;

    if (mode == 0){         //Home position
        position.position.x = defaultCartesianPosition[0][0];
        position.position.y = defaultCartesianPosition[0][1];
        position.position.z = defaultCartesianPosition[0][2];
        position.orientation.x = defaultCartesianPosition[0][3];
        position.orientation.y = defaultCartesianPosition[0][4];
        position.orientation.z = defaultCartesianPosition[0][5];
        position.orientation.w = defaultCartesianPosition[0][6];
        ROS_INFO("MOVING TO HOME POSITION!");
    }else if (mode == 1){      //pick position
        position.position.x = defaultCartesianPosition[1][0];
        position.position.y = defaultCartesianPosition[1][1];
        position.position.z = defaultCartesianPosition[1][2];
        position.orientation.x = defaultCartesianPosition[1][3];
        position.orientation.y = defaultCartesianPosition[1][4];
        position.orientation.z = defaultCartesianPosition[1][5];
        position.orientation.w = defaultCartesianPosition[1][6];
        ROS_INFO("MOVING TO PICK POSITION!");
    }else if (mode == 2){       //work position
        position.position.x = defaultCartesianPosition[2][0];
        position.position.y = defaultCartesianPosition[2][1];
        position.position.z = defaultCartesianPosition[2][2];
        position.orientation.x = defaultCartesianPosition[2][3];
        position.orientation.y = defaultCartesianPosition[2][4];
        position.orientation.z = defaultCartesianPosition[2][5];
        position.orientation.w = defaultCartesianPosition[2][6];
        ROS_INFO("MOVING TO WORK POSITION!");
    }
    else if (mode == 3){        //place position
        if (number_of_place_position == 1){
            position.position.x = defaultCartesianPosition[3][0];
            position.position.y = defaultCartesianPosition[3][1];
            position.position.z = defaultCartesianPosition[3][2];
            position.orientation.x = defaultCartesianPosition[3][3];
            position.orientation.y = defaultCartesianPosition[3][4];
            position.orientation.z = defaultCartesianPosition[3][5];
            position.orientation.w = defaultCartesianPosition[3][6];
        }else if (number_of_place_position == 2){
            position.position.x = defaultCartesianPosition[4][0];
            position.position.y = defaultCartesianPosition[4][1];
            position.position.z = defaultCartesianPosition[4][2];
            position.orientation.x = defaultCartesianPosition[4][3];
            position.orientation.y = defaultCartesianPosition[4][4];
            position.orientation.z = defaultCartesianPosition[4][5];
            position.orientation.w = defaultCartesianPosition[4][6];
        }else if (number_of_place_position == 3){
            position.position.x = defaultCartesianPosition[5][0];
            position.position.y = defaultCartesianPosition[5][1];
            position.position.z = defaultCartesianPosition[5][2];
            position.orientation.x = defaultCartesianPosition[5][3];
            position.orientation.y = defaultCartesianPosition[5][4];
            position.orientation.z = defaultCartesianPosition[5][5];
            position.orientation.w = defaultCartesianPosition[5][6];
        }else if (number_of_place_position == 4){
            position.position.x = defaultCartesianPosition[6][0];
            position.position.y = defaultCartesianPosition[6][1];
            position.position.z = defaultCartesianPosition[6][2];
            position.orientation.x = defaultCartesianPosition[6][3];
            position.orientation.y = defaultCartesianPosition[6][4];
            position.orientation.z = defaultCartesianPosition[6][5];
            position.orientation.w = defaultCartesianPosition[6][6];
        }else if (number_of_place_position == 5){
            position.position.x = defaultCartesianPosition[7][0];
            position.position.y = defaultCartesianPosition[7][1];
            position.position.z = defaultCartesianPosition[7][2];
            position.orientation.x = defaultCartesianPosition[7][3];
            position.orientation.y = defaultCartesianPosition[7][4];
            position.orientation.z = defaultCartesianPosition[7][5];
            position.orientation.w = defaultCartesianPosition[7][6];
        }else if (number_of_place_position == 6){
            position.position.x = defaultCartesianPosition[8][0];
            position.position.y = defaultCartesianPosition[8][1];
            position.position.z = defaultCartesianPosition[8][2];
            position.orientation.x = defaultCartesianPosition[8][3];
            position.orientation.y = defaultCartesianPosition[8][4];
            position.orientation.z = defaultCartesianPosition[8][5];
            position.orientation.w = defaultCartesianPosition[8][6];
        }else if (number_of_place_position == 7){
            position.position.x = defaultCartesianPosition[9][0];
            position.position.y = defaultCartesianPosition[9][1];
            position.position.z = defaultCartesianPosition[9][2];
            position.orientation.x = defaultCartesianPosition[9][3];
            position.orientation.y = defaultCartesianPosition[9][4];
            position.orientation.z = defaultCartesianPosition[9][5];
            position.orientation.w = defaultCartesianPosition[9][6];
        }else if (number_of_place_position == 8){
            position.position.x = defaultCartesianPosition[10][0];
            position.position.y = defaultCartesianPosition[10][1];
            position.position.z = defaultCartesianPosition[10][2];
            position.orientation.x = defaultCartesianPosition[10][3];
            position.orientation.y = defaultCartesianPosition[10][4];
            position.orientation.z = defaultCartesianPosition[10][5];
            position.orientation.w = defaultCartesianPosition[10][6];
        }else {
            ROS_INFO("NOT DEFINED position");
        }
        ROS_INFO("MOVING TO PLACE POSITION!:%d",number_of_place_position);
    }else{
        ROS_INFO("NOT DEFINED mode");
    }

    while (numberOfAttempts<3) {
        if (move_group->setApproximateJointValueTarget(position, "tool0")) {
            ROS_INFO("found IK solution");
            break;
        } else
            ROS_INFO("only aproximate IK solution");
        numberOfAttempts++;
    }
    success = static_cast<bool>(move_group->plan(my_plan));
    // ROS_INFO_STREAM("PLAN:" << success);
    if(success){

        int size=my_plan.trajectory_.joint_trajectory.points.size();

        //ROS_INFO_STREAM("moje IK je");
        //ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points[size-1]);
    }
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 -back (pose goal) %s", success ? "GOOD" : "BAD");
    if (asyncMode){
        move_group->asyncExecute(my_plan);
        //move_group->asyncMove();
        ROS_INFO("only aproximate IK solution");
    }else{
        move_group->execute(my_plan);
        move_group->move();
    }


}
geometry_msgs::Pose getTargetCoordinates (moveit::planning_interface::MoveGroupInterface *move_group){

    geometry_msgs::PoseStamped ws=move_group->getCurrentPose();
    geometry_msgs::Pose target_pose;

    target_pose.position.x = ws.pose.position.x;
    target_pose.position.y = ws.pose.position.y;
    target_pose.position.z = ws.pose.position.z;
    target_pose.orientation.x = ws.pose.orientation.x;
    target_pose.orientation.y = ws.pose.orientation.y;
    target_pose.orientation.z = ws.pose.orientation.z;
    target_pose.orientation.w = ws.pose.orientation.w;
    //ROS_INFO("Effector pose : x=%f , y=%f , z=%f \n", ws.pose.position.x, ws.pose.position.y, ws.pose.position.z);

    return target_pose;
}
void printMenu(){
    ROS_INFO("\nPRESS number for:");
    ROS_INFO("1 - Set max velocity:");
    ROS_INFO("2 - Set max acceleration:");
    ROS_INFO("3 - Set planning time:");
    ROS_INFO("4 - Set number of planning attempts:");
    ROS_INFO("5 - Display information:");
    ROS_INFO("6 - Co-work with rotary table:");
    ROS_INFO("0 - EXIT:");

}
int menu(moveit::planning_interface::MoveGroupInterface *move_group){

    int number,num;

    while (1){
        ROS_INFO("\n");
        ROS_INFO("*************** SENSODRIVE SCARA ***************");
        ROS_INFO("**************       menu        ***************");
        ROS_INFO("**   Press 1 for:\t JOINT CONTROL     **");
        ROS_INFO("**   Press 2 for:\t POSITION CONTROL  **");
        ROS_INFO("**   Press 3 for:\t POSITION CONTROL  **");
        ROS_INFO("**   with IK calculation ahead            **");
        ROS_INFO("**   Press 4 for:\t INFO              **");
        ROS_INFO("**   Press 5 for:\t SET PARAMETERS    **");
        ROS_INFO("**   Press 9 for:\tEXIT               **");
        ROS_INFO("***********************************************");
        ROS_INFO("***********************************************");
        ROS_INFO("\nEnter number:");
        number = getchar() - 48;
       // getchar();
        if (number != -38) {
            ROS_INFO("number = %d", number);
        }
        if (number == 1){
            ROS_INFO("You have choosed %d - JOINT CONTROL",number);
            break;
        }else if (number == 2){
            ROS_INFO("You have choosed %d - POSITION CONTROL",number);
            break;
        }else if (number == 3){
            ROS_INFO("You have choosed %d - POSITION CONTROL with IK calculation ahead",number);
            break;
        }else if (number == 4){
            ROS_INFO("You have choosed %d - INFO",number);
            ROS_INFO("Reference frame: %s", move_group->getPlanningFrame().c_str());
            ROS_INFO("End effector link: %s", move_group->getEndEffectorLink().c_str());
            ROS_INFO("Active joints:");
            std::vector< std::string> activeJoints = move_group->getActiveJoints();
           for (int i =0;i<activeJoints.size();i++){
               ROS_INFO("Active link %d : %s",i, activeJoints[i].c_str());
                }
            ROS_INFO(".....Current robot model.......");
            ROS_INFO_STREAM(move_group->getRobotModel());
            ROS_INFO(".....Current pose.......");
            geometry_msgs::PoseStamped currentPose = move_group->getCurrentPose();
            ROS_INFO("X=%f Y=%f Z=%f \n roll=%f pitch=%f yaw=%f",currentPose.pose.position.x,currentPose.pose.position.y, currentPose.pose.position.z,
                     currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z);
            sleep(3);

            //pridat dalsie informacie


        }else if (number == 5){
            while (1){
                printMenu();
                ROS_INFO("\nEnter number:");
                num = getchar() - 48;

                if (num == 1){
                    float velocity;
                    ROS_INFO("enter velocity");
                    scanf("%f",&velocity);
                    ROS_INFO("entered %f",velocity);
                    move_group->setMaxVelocityScalingFactor(velocity);
                    break;
                }else if (num == 2){
                    float acceleration;
                    ROS_INFO("enter acceleration");
                    scanf("%f",&acceleration);
                    ROS_INFO("entered %f",acceleration);
                    move_group->setMaxAccelerationScalingFactor(acceleration);
                    break;
                }else if (num == 3){
                    float time;
                    ROS_INFO("enter planning time");
                    scanf("%f",&time);
                    ROS_INFO("entered %f",time);
                    move_group->setPlanningTime(time);
                    break;
                }else if (num == 4){
                    int numOfAttempts;
                    ROS_INFO("enter number of attempts");
                    scanf("%d",&numOfAttempts);
                    ROS_INFO("entered %d",numOfAttempts);
                    move_group->setNumPlanningAttempts(numOfAttempts);
                    break;
                }else if (num == 5){
                    int displayInfo;
                    ROS_INFO("1 - FULL information");
                    ROS_INFO("0 - PARTIAL information");
                    ROS_INFO("enter number");
                    scanf("%d",&displayInfo);
                    ROS_INFO("entered %d",displayInfo);
                    if (displayInfo == 0) {
                        displayMode = false;
                        ROS_INFO("PARTIAL information SET");
                    }else {
                        displayMode = true;
                        ROS_INFO("FULL information SET");
                    }
                    break;
                }else if (num == 6){
                    int cowork;
                    ROS_INFO("1 - Service feedback from RT");
                    ROS_INFO("0 - No feedback from RT");
                    ROS_INFO("enter number");
                    scanf("%d",&cowork);
                    ROS_INFO("entered %d",cowork);
                    if (cowork == 0) {
                        rtService = false;
                        ROS_INFO("NO service feedback from RT");
                    }else {
                        rtService = true;
                        ROS_INFO("Service feedback from RT SET");
                    }
                    break;

                }
                else if (num == 0){
                    break;
                }
            }

        }
        else if (number == 9){
            ROS_WARN("You have choosed %d - EXIT",number);
            return -1;
        }else if (number == -38){
            continue;
        }
        else{
            ROS_INFO("NOT VALID");
        }
        sleep(1);
    }

    ROS_INFO_STREAM(number);
    return number;
}
void shakeThread(){
    for (int i = 0; i < 5; ++i)
    {
        ROS_INFO("THREAD : waiting %d seconds", i);
        sleep(1);
    }
    threadExecution = true;
    ROS_INFO("thread end");
}
void forceFeedbackThread(){

    ROS_INFO("forcefeedback thread start");
    moveit::core::RobotStatePtr current_state;
    int i=0;

//    while (ros::ok()){
//        ROS_INFO("[forcefeedback thread] %d",i );
//        if (i<10){
//            ROS_INFO("[forcefeedback thread] STOP %d",stop);
//            sleep(1);
//        }else{
//            stop = true;
//            ROS_INFO("[forcefeedback thread] STOP %d",stop);
//            mg->stop();
//        }
//        i++;
//        if (i > 15000){
//            break;
//        }
//    }

    while (ros::ok()){
        //ROS_INFO("[forcefeedback thread] torque value=%f (max=%f)",torque_value, max_torque_value);

        if (torque_value >= max_torque_value){
            stop = true;
            ROS_INFO("[forcefeedback thread] STOP %d",stop);
            mg->stop();

        }

    }



}
bool servicesCheck(ros::ServiceClient *ik_service, ros::ServiceClient *rt_service, ros::ServiceClient *moveit_service){

//    while (!ik_service->exists()){
//        ROS_INFO("waiting for IK service [ %d ] seconds",waitingTime++);
//        ROS_INFO("Please check the moveit node!");
//        sleep(1);
//    }
//    waitingTime = 0;
    int waitTime = 0;
    ROS_INFO("servicesCheck function");


        while (!rt_service->exists()) {
            ROS_WARN("waiting for RT service [ %d ] seconds", waitTime++);
            ROS_WARN("Please check the rt node!");
            sleep(1.0);
            if (waitTime > 20) {
                ROS_ERROR("_____PROGRAM END!______");
                return false;
            }
        }
        waitTime = 0;


    while (!moveit_service->exists()) {
        ROS_WARN("waiting for RT service [ %d ] seconds",waitTime++);
        ROS_WARN("Please check the rt node!");
        sleep(1.0);
        if (waitTime >20){
            ROS_ERROR("_____PROGRAM END!______");
            return false;
        }
    }
    waitTime = 0;

    return true;

}
void trajectoryExecutionCallback(const moveit_msgs::ExecuteTrajectoryActionResult result){

    ROS_INFO("....TRAJECTORY EXECUTION CALLBACK Status = %d....\n",result.status.status);
    if (result.status.status == 1 || result.status.status == 3){
        executionOK = true;
    }
}
void torqueSensorCallback(const std_msgs::Float64 torqueValue){

    //ROS_INFO("torque CALLBACK %f",torqueValue);
    torque_value = torqueValue.data;
}

int main(int argc, char **argv){

    int counter = 1, mode = 0;
    int modeExecution = 100;
    int num ;
    bool success;
    static const std::string PLANNING_GROUP = "scara_arm";
    geometry_msgs::Pose target_pose1;
    geometry_msgs::PoseStamped ws1;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    geometry_msgs::Pose selfPosition;
    geometry_msgs::PoseStamped position;
    moveit_msgs::RobotState robot_state;
    bool moveToHome = false;
    scara_v2_moveit_api::SimpleService rt_srv;
    //help variables init
    bool initRT = false;
    bool asyncMode = true;
    executionOK = true;
    bool threadStart = true;
    //bool moveToHome = true;




    ros::init(argc, argv, "PICK_and_PLACE");
    ros::NodeHandle n, nn,n_rt;
    ros::NodeHandle n_gripper,n_sub_rt,n_torque;
    ros::NodeHandle n_rt_srv,n_moveit_srv;
    ros::Rate r(2);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    mg = &move_group;
    //move_group = new moveit::planning_interface::MoveGroupInterface (PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ros::Rate loop_rate(5);
    ros::ServiceClient service_client;


    do {
        num = menu(&move_group);
        if (num == 1) {
            //setDesiredAngles();
            if (setPositions(displayMode)) {
                ROS_INFO("default positions for Joint control OK");
            }
        } else if (num == 2) {
            if (setCartesianPositions(displayMode)) {
                ROS_INFO("default positions for Cartesian planning  OK");
                service_client = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
                while (!service_client.exists()) {
                    ROS_WARN("waiting for IK service [ %d ] seconds",waitingTime++);
                    sleep(1.0);
                }
                waitingTime = 0;
                position = move_group.getCurrentPose();
                selfPosition.position = position.pose.position;
                selfPosition.orientation = position.pose.orientation;
                moveit::core::robotStateToRobotStateMsg(*move_group.getCurrentState(), robot_state, true);
                moveToHome = true;
            }
        }else if (num == 3){
            ROS_ERROR("mode 3!");
            setCartesianPositions(true);
            setPositions(false);
            jointModeControll(&move_group, my_plan, 0, 0,false);
            ROS_INFO("wait!");
            sleep(2);
            getAnglesFromIK(&move_group,my_plan);
            setPositionsFromIK(displayMode);
            num = 1;    //for joint control
        }else if (num == -1) {
            ROS_WARN("program END!");
            ros::shutdown();
            spinner.stop();
            return 0;
        }
    }while (num > 3);

    //Topics init
    ros::Publisher gripperState_pub = n.advertise<std_msgs::String>("gripper_state_topic", 1000);
    ros::Publisher grip_topic_pub =  nn.advertise<scara_v2_moveit_api::pose_and_gripperState>("gripper_state", 1000);
    ros::Publisher rt_pub = n_rt.advertise<std_msgs::String>("commandForRotaryTable", 1000);
    //ros::Subscriber grip_topic_sub = n_gripper.subscribe("gripper_state_execution",1000,gripperExecutionCallback);
    ros::Subscriber executeTrajectory = nn.subscribe("execute_trajectory/result", 1000, trajectoryExecutionCallback);
    ros::Subscriber torqueSensor= n_torque.subscribe("torqueSensor",1000,torqueSensorCallback);
    //Services init
    ros::ServiceClient rt_client = n_rt_srv.serviceClient<scara_v2_moveit_api::SimpleService>("rt_service");
    ros::ServiceClient moveit_client = n_moveit_srv.serviceClient<roscpp::GetLoggers>("/joint_state_publisher/get_loggers");


    while (!rt_client.exists()) {
        ROS_WARN("waiting for RT service [ %d ] seconds", waitingTime++);
        sleep(1.0);
    }
    waitingTime = 0;
    while (!moveit_client.exists()) {
        ROS_WARN("waiting for moveit service [ %d ] seconds",waitingTime++);
        sleep(1.0);
    }
    waitingTime = 0;

    //gripper message init
    scara_v2_moveit_api::pose_and_gripperState gripperStates;
    gripperStates.gripperState = false;
    gripperStates.posX = 0.0;
    gripperStates.posY = 0.0;
    gripperStates.posZ = 0.0;
    std_msgs::String msg ;
    msg.data = "otoc_sa";

    boost::thread fft{forceFeedbackThread};

    while (ros::ok()) {

        if (!initRT) {
            rt_pub.publish(msg);
            ROS_INFO("[SCARA] ( =>RT ) : RT init : %s", msg.data.c_str());
            sleep(2);
            initRT = true;
        }
        if ( !servicesCheck(&service_client,&rt_client,&moveit_client)){    //if services dont work
            ROS_ERROR("FUCK1");
            return 0;
        }

        if (displayMode) {
            //Get current pose of tool0
            target_pose1 = getTargetCoordinates(&move_group);
            ROS_INFO("[SCARA]: Actual joint values : x=%f  y=%f  z=%f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
        }
        //move to WS1
        current_state = move_group.getCurrentState();
        current_state->copyJointGroupPositions(joint_model_group, joint_group_position);
        if (stop){
            ROS_ERROR("program end due to force feedback detection");
            break;
        }
        if (executionOK) {

            if (asyncMode) {
                executionOK = false;
                if (displayMode) {
                    ROS_INFO("STARTED EXECUTING TRAJECTORY (async mode)");
                }
            }else{
                if (displayMode) {
                    ROS_INFO("STARTED EXECUTING TRAJECTORY (normal mode)");
                }
            }
            if (displayMode){
                ROS_INFO("mode = %d  mode execution number = %d",mode, modeExecution);
            }

            if (modeExecution == 0){
                ROS_INFO("moving to home EXECUTED");
                mode = 1;
                sleep(2);
            }else if (modeExecution == 1){
                ROS_INFO("moving to pick EXECUTED");
                //modeExecution == 100;
                target_pose1 = getTargetCoordinates(&move_group);
                ROS_INFO("[SCARA] ( =>CUBE ): Gripper Pick! and publish");
                gripperStates.gripperState = true;
                gripperStates.posX = target_pose1.position.x;
                gripperStates.posY = target_pose1.position.y;
                gripperStates.posZ = target_pose1.position.z;
                grip_topic_pub.publish(gripperStates);
                mode = 2;
                sleep(2);
            }else if (modeExecution == 2){
                ROS_INFO("fcking thread");
                if (threadStart) {
                    ROS_INFO("moving to work EXECUTED");
                    //sleep(1);
                    rt_pub.publish(msg);
                    ROS_INFO("[SCARA] ( =>RT ): RT turn : %s", msg.data.c_str());
                }
                //shakeMode();

                if (threadStart){
                    ROS_INFO("thread start");
                    boost::thread t{shakeThread};
                    threadStart = false;
                }
                if (threadExecution){
                    ROS_INFO("thread finish");
                    mode = 3;
                    threadStart = true;
                    threadExecution = false;
                }else{
                    ROS_INFO("thread working");
                }

                sleep(2);
            }else if (modeExecution == 3){
                ROS_INFO("moving to place:%d EXECUTED",counter);
                //modeExecution == 100;
                counter++;
                if (counter > 8) {
                    counter = 1;
                }
                target_pose1 = getTargetCoordinates(&move_group);
                ROS_INFO("[SCARA]  ( =>CUBE ) : Gripper Place! and publish");
                gripperStates.gripperState = false;
                gripperStates.posX = target_pose1.position.x;
                gripperStates.posY = target_pose1.position.y;
                gripperStates.posZ = target_pose1.position.z;
                ROS_INFO_STREAM(gripperStates);
                grip_topic_pub.publish(gripperStates);
                sleep(2);
                mode = 0;
            }else{
                ROS_INFO("no mode executed");
                sleep(2);
            }


            if (mode == 0){
                ROS_INFO("MODE = %d",mode);
                if (num == 1) {
                    jointModeControll(&move_group, my_plan, mode, 0,asyncMode);
                } else if (num == 2) {
                    positionControll(&move_group, my_plan, mode, 0,asyncMode);
                } else {
                    ROS_ERROR("NOT VALID CONTROL MODE");
                }
                sleep(3);
                modeExecution = 0;
            }else if (mode == 1){
                ROS_INFO("MODE = %d",mode);
                if (num == 1) {
                    jointModeControll(&move_group, my_plan, mode, 0,asyncMode);
                } else if (num == 2) {
                    positionControll(&move_group, my_plan, mode, 0,asyncMode);
                } else {
                    ROS_ERROR("NOT VALID CONTROL MODE");
                }
                sleep(3);
                modeExecution = 1;
            }else if (mode == 2){
                if (threadStart) {
                    ROS_INFO("MODE = %d", mode);
                    if (num == 1) {
                        jointModeControll(&move_group, my_plan, mode, 0, asyncMode);
                    } else if (num == 2) {
                        positionControll(&move_group, my_plan, mode, 0, asyncMode);
                    } else {
                        ROS_ERROR("NOT VALID CONTROL MODE");
                    }

                    modeExecution = 2;
                }
            }else if (mode == 3){
                ROS_INFO("MODE = %d",mode);
                if (num == 1) {
                    jointModeControll(&move_group, my_plan, mode, counter,asyncMode);
                } else if (num == 2) {
                    positionControll(&move_group, my_plan, mode, counter,asyncMode);
                } else {
                    ROS_ERROR("NOT VALID CONTROL MODE");
                }
                sleep(1);
                modeExecution = 3;
            }else{
                ROS_INFO("no mode selected");
            }


        }
        else {
            if (threadExecution){
                executionOK = true;
            }
            if (torque_value>1.5){
                move_group.stop();

            }
            //ROS_INFO("****ASYNC EXECUTION OF TRAJECTORY******");
            ROS_INFO("torque value = %f ",torque_value);
            //sleep(1);
        }


        //ROS_INFO("mode = %d  mode execution number = %d  execution = %d",mode, modeExecution,executionOK);
        if (displayMode){
            ROS_INFO("waiting for message");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

