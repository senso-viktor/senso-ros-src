
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

#include "sensor_msgs/JointState.h"

std::vector<std::vector<double>> defaultCartesianPosition(2, std::vector<double>(7));
std::vector<std::vector<double>> anglesFromIK(2, std::vector<double>(3));
bool executionOK = false;
moveit::planning_interface::MoveItErrorCode mec;

void printDefaultInformation(moveit::planning_interface::MoveGroupInterface *moveGroup){

    ROS_INFO_NAMED("\nReference frame: %s", moveGroup->getPlanningFrame().c_str());
    ROS_INFO_NAMED("End effector link: %s\n", moveGroup->getEndEffectorLink().c_str());
}

void getAnglesFromIK(moveit::planning_interface::MoveGroupInterface *move_group, moveit::planning_interface::MoveGroupInterface::Plan my_plan){

    geometry_msgs::Pose position;
    bool success;
    int size;
    int mode = 0;

    ROS_INFO("def %d",defaultCartesianPosition.size());
    for (int i = 0;i<defaultCartesianPosition.size();i++){

            position.position.x = defaultCartesianPosition[mode][0];
            position.position.y = defaultCartesianPosition[mode][1];
            position.position.z = defaultCartesianPosition[mode][2];
            position.orientation.x = defaultCartesianPosition[mode][3];
            position.orientation.y = defaultCartesianPosition[mode][4];
            position.orientation.z = defaultCartesianPosition[mode][5];
            position.orientation.w = defaultCartesianPosition[mode][6];
        ROS_INFO("positions");
        ROS_INFO_STREAM(position);
        getchar();

        if (move_group->setApproximateJointValueTarget(position, "tool0")) {
            ROS_INFO("found IK solution");
        } else
            ROS_INFO("only aproximate IK solution");

        success = move_group->plan(my_plan);
        // ROS_INFO_STREAM("PLAN:" << success);
        if(success){
            size=my_plan.trajectory_.joint_trajectory.points.size();
            ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points[size-1]);

            anglesFromIK[mode][0] = my_plan.trajectory_.joint_trajectory.points[size-1].positions[0];
            anglesFromIK[mode][1] = my_plan.trajectory_.joint_trajectory.points[size-1].positions[1];
            anglesFromIK[mode][2] = my_plan.trajectory_.joint_trajectory.points[size-1].positions[2];
        } else
            ROS_INFO("not success");
        getchar();



       // ROS_INFO("angles");
       // ROS_INFO_STREAM(anglesFromIK);
        getchar();
        mode++;
    }

    for (int i=0;i<anglesFromIK.size();i++){
        ROS_INFO("final angle %d is %f %f %f",i,anglesFromIK[i][0],anglesFromIK[i][1],anglesFromIK[i][2]);
    }
    getchar();
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
    ROS_INFO("wait for character");
    getchar();
    success = move_group->plan(my_plan);
    mec = move_group->plan(my_plan);
    //overenie vytvorenia planu
    ROS_INFO("moveit error code");
    ROS_INFO_STREAM(mec);
    ROS_INFO("wait for character");
    getchar();


    // ROS_INFO_STREAM("PLAN:" << success);
    if(success){

        int size=my_plan.trajectory_.joint_trajectory.points.size();

        ROS_INFO_STREAM("moje IK je");
        ROS_INFO_STREAM(my_plan.trajectory_.joint_trajectory.points[size-1]);
    }
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 -back (pose goal) %s", success ? "GOOD" : "BAD");
    if (asyncMode){
        mec = move_group->asyncExecute(my_plan);
        //move_group->asyncMove();
//        ROS_INFO("moveit error code");
//        ROS_INFO_STREAM(mec);

    }else{
        move_group->execute(my_plan);
        move_group->move();
    }
    ROS_INFO("wait for character");
   getchar();


}
void trajectoryExecutionCallback(const moveit_msgs::ExecuteTrajectoryActionResult result){

    ROS_INFO("....TRAJECTORY EXECUTION CALLBACK Status = %d....\n",result.status.status);
    if (result.status.status == 1 || result.status.status == 3){
        executionOK = true;
    }
}
bool setCartesianPositions(bool showPositions){

//    defaultCartesianPosition.resize(11);
//    defaultCartesianPosition[0].resize(3);
    // position 1
    //xyz

    defaultCartesianPosition[0][0] = 0.4;
    defaultCartesianPosition[0][1] = 0.24;
    defaultCartesianPosition[0][2] = 1.01;
    //rpy+w
    defaultCartesianPosition[0][3] = 0.99;
    defaultCartesianPosition[0][4] = -0.08;
    defaultCartesianPosition[0][5] = -0.00077;
    defaultCartesianPosition[0][6] = 0.00012;

    // position 2
    //xyz
    defaultCartesianPosition[1][0] = 0.39;
    defaultCartesianPosition[1][1] = 0.93;
    defaultCartesianPosition[1][2] = 1.01;
    //rpy
    defaultCartesianPosition[1][3] = -0.095;
    defaultCartesianPosition[1][4] = 0.995;
    defaultCartesianPosition[1][5] = 0;
    defaultCartesianPosition[1][6] = -0.00078;


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

int main (int argc, char **argv) {

    bool success;
    static const std::string PLANNING_GROUP = "scara_arm";
    geometry_msgs::Pose target_pose;
    geometry_msgs::PoseStamped ws1;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    bool asyncMode = true;
    executionOK = true;
    int modeExecution = 100;
    int mode = 0;
    //IKFAST
    moveit_msgs::GetPositionIK IKposition;

    //IKposition.request.

    ros::init(argc, argv, "pose_control");
    ros::NodeHandle n, nn;
    ros::Rate r(2);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(5);

    setCartesianPositions(false);


    //nejak to osetrit
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(
            PLANNING_GROUP);

    printDefaultInformation(&move_group);
    ros::Subscriber executeTrajectory = nn.subscribe("execute_trajectory/result", 1000, trajectoryExecutionCallback);

    ros::ServiceClient service_client = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    while (!service_client.exists()) {
        ROS_INFO("waiting for service");
        sleep(1.0);
    }

    getAnglesFromIK(&move_group,my_plan);

    int counter = 0;
    while (ros::ok()) {


        if (executionOK) {
            ROS_INFO("STARTED EXECUTING TRAJECTORY");

            if (asyncMode) {
                executionOK = false;
                //ROS_INFO("STARTED EXECUTING TRAJECTORY (async mode)");
            } else {
                //ROS_INFO("STARTED EXECUTING TRAJECTORY (normal mode)");
            }
            ROS_INFO("mode execution number = %d", modeExecution);

            if (modeExecution == 0) {
                ROS_INFO("moving to home EXECUTED");
                mode = 1;
                sleep(2);
            } else if (modeExecution == 1) {
                ROS_INFO("moving to pick EXECUTED");

                mode = 0;
                sleep(2);
            } else {
                ROS_INFO("fck 1");
            }



            if (mode == 0) {
                ROS_INFO("MODE = %d", mode);
                positionControll(&move_group, my_plan, mode, 0, asyncMode);
                sleep(3);
                modeExecution = 0;

            } else if (mode == 1) {
                ROS_INFO("MODE = %d", mode);
                positionControll(&move_group, my_plan, mode, 0, asyncMode);
                sleep(3);
                modeExecution = 1;
            } else {
                ROS_INFO("fck 2");
            }

        }

        ROS_INFO("waiting for message");
        ros::spinOnce();
        loop_rate.sleep();
    }







    return 0;
}
