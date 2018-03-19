//
// Created by viktor on 02/05/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>




int main(int argc, char **argv){

    bool success;

    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "scara_arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //nastavenie poziciie 2
    geometry_msgs::PoseStamped ws1 = move_group.getCurrentPose();
    ROS_INFO("Effector x=%f , y=%f , z=%f ", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;


    //nastavenie pozicie 1
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 1.0;  // radians
    joint_group_positions[1] = 1.0;
    move_group.setJointValueTarget(joint_group_positions);
    success = static_cast<bool>(move_group.plan(my_plan));
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
    if (move_group.execute(my_plan)){
        ROS_INFO("uspesne execute");
    }
    else
        ROS_INFO("neuspesne execute");
    if (move_group.move())
        ROS_INFO("Uspesny move\n");
    else
        ROS_INFO("Neuspesny move\n");

    ROS_INFO("moved to place!!\n");

    ws1 = move_group.getCurrentPose();
    ROS_INFO("Effector x=%f , y=%f , z=%f \n", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);
    current_state = move_group.getCurrentState();
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    for (int i=0;i<current_joint_values.size();i++){
        ROS_INFO("Desired joint %d value %f",i,joint_group_positions[i]);
        ROS_INFO("Joint %d value %f",i,current_joint_values[i]);
    }

    getchar();

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = -0.1;  // radians
    joint_group_positions[1] = 0.75;
    sleep(2);
    move_group.setJointValueTarget(joint_group_positions);
    success = static_cast<bool>(move_group.plan(my_plan));
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
    if (move_group.execute(my_plan)){
        ROS_INFO("uspesne execute");
    }
    else
        ROS_INFO("neuspesne execute");
    if (move_group.move())
        ROS_INFO("Uspesny move\n");
    else
        ROS_INFO("Neuspesny move\n");

    ROS_INFO("moved to place!!\n");


    current_joint_values = move_group.getCurrentJointValues();
    for (int i=0;i<current_joint_values.size();i++){
        ROS_INFO("Desired joint %d value %f",i,joint_group_positions[i]);
        ROS_INFO("Joint %d value %f",i,current_joint_values[i]);
    }

    getchar();

    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.5;  // radians
    joint_group_positions[1] = 0.5;
    sleep(2);
    move_group.setJointValueTarget(joint_group_positions);
    success = static_cast<bool>(move_group.plan(my_plan));
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
    if (move_group.execute(my_plan)){
        ROS_INFO("uspesne execute");
    }
    else
        ROS_INFO("neuspesne execute");
    if (move_group.move())
        ROS_INFO("Uspesny move\n");
    else
        ROS_INFO("Neuspesny move\n");

    ROS_INFO("moved to place!!\n");


    current_joint_values = move_group.getCurrentJointValues();
    for (int i=0;i<current_joint_values.size();i++){
        ROS_INFO("Desired joint %d value %f",i,joint_group_positions[i]);
        ROS_INFO("Joint %d value %f",i,current_joint_values[i]);
    }

    getchar();


    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0;  // radians
    joint_group_positions[1] = 0;
    sleep(2);
    move_group.setJointValueTarget(joint_group_positions);
    success = static_cast<bool>(move_group.plan(my_plan));
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
    if (move_group.execute(my_plan)){
        ROS_INFO("uspesne execute");
    }
    else
        ROS_INFO("neuspesne execute");
    if (move_group.move())
        ROS_INFO("Uspesny move\n");
    else
        ROS_INFO("Neuspesny move\n");

    ROS_INFO("moved to place!!\n");


    current_joint_values = move_group.getCurrentJointValues();
    for (int i=0;i<current_joint_values.size();i++){
        ROS_INFO("Desired joint %d value %f",i,joint_group_positions[i]);
        ROS_INFO("Joint %d value %f",i,current_joint_values[i]);
    }

    getchar();



//    sleep(2);
//    move_group.setPlanningTime(10.0);
//    geometry_msgs::Pose target_pose1;
//    target_pose1.position.x = ws1.pose.position.x;
//    target_pose1.position.y = ws1.pose.position.y;
//    target_pose1.position.z = ws1.pose.position.z;
//    target_pose1.orientation.x = ws1.pose.orientation.x;
//    target_pose1.orientation.y = ws1.pose.orientation.y;
//    target_pose1.orientation.z = ws1.pose.orientation.z;
//    target_pose1.orientation.w = ws1.pose.orientation.w;
//
//    move_group.setPoseTarget(target_pose1);
//    success = move_group.plan(my_plan);
//    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 -back (pose goal) %s", success ? "" : "FAILED");
//
//    move_group.execute(my_plan);
//    if (move_group.execute(my_plan))
//        ROS_INFO("uspesne execute");
//    else
//        ROS_INFO("neuspesne execute");
//
//    ROS_INFO("end of 2nd planning");
//
//    current_joint_values = move_group.getCurrentJointValues();
//    for (int i=0;i<current_joint_values.size();i++){
//        ROS_INFO("Joint %d value %f",i,current_joint_values[i]);
//    }
//
//    getchar();




    return 0;
}
