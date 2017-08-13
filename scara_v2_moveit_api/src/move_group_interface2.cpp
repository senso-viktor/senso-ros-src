
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
std::vector<std::vector<double>> joint_group_positions(6, std::vector<double>(2));
const double DEG2RAD=0.01745329252;
const double RAD2DEG=57.295779513;

void setDesiredAngles (){

    ROS_INFO("VECTOR SIZE %d x %d \n", joint_group_positions.size(),joint_group_positions[0].size());

    joint_group_positions[0][0] = 80*DEG2RAD;
    joint_group_positions[0][1] = 90*DEG2RAD;
    joint_group_positions[1][0] = 45*DEG2RAD;
    joint_group_positions[1][1] = 120*DEG2RAD;
    joint_group_positions[2][0] = joint_group_positions[1][0] - 20*DEG2RAD;
    joint_group_positions[2][1] = 0;
    joint_group_positions[3][0] = -45*DEG2RAD;
    joint_group_positions[3][1] = 45*DEG2RAD;
    joint_group_positions[4][0] = -75*DEG2RAD;
    joint_group_positions[4][1] = 100*DEG2RAD;
    joint_group_positions[4][0] = 0;
    joint_group_positions[4][1] = 0;
    ROS_INFO("Vector filled up");
    //ROS_INFO("%f",joint_group_positions[2][1]);


}
int main(int argc, char **argv){

    bool success;
    static const std::string PLANNING_GROUP = "scara_arm";
    std::vector<double> joint_group_position;
    geometry_msgs::PoseStamped ws1;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;


    setDesiredAngles();
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    while (1) {

        for (int counter = 0; counter < joint_group_positions.size(); counter++) {
            //ROS_INFO("position 0=%f\n",joint_group_positions[counter][0]);
            //ROS_INFO("%f",joint_group_positions[2][1]);

            current_state = move_group.getCurrentState();
            current_state->copyJointGroupPositions(joint_model_group, joint_group_position);

            joint_group_position[0] = joint_group_positions[counter][0];  // radians
            joint_group_position[1] = joint_group_positions[counter][1];
            move_group.setJointValueTarget(joint_group_position);
            success = move_group.plan(my_plan);
            ROS_INFO("Going to pose number : %d", counter);
            //ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "" : "FAILED");
            move_group.execute(my_plan);
            move_group.move();
            /*
            if (move_group.execute(my_plan))
                ROS_INFO("uspesne execute");
            else
                ROS_INFO("neuspesne execute");
            if (move_group.move())
                ROS_INFO("Uspesny move\n");
            else
                ROS_INFO("Neuspesny move\n");
            ROS_INFO("moved to place!!\n");
*/
            ws1 = move_group.getCurrentPose();
            ROS_INFO("Effector pose : x=%f , y=%f , z=%f \n", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);

            sleep(2);


        }

    }







    return 0;
}