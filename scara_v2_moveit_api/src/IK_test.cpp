//
// Created by viktordluhos on 14/08/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "math.h"
#include "complex"
#include "iostream"
#include <tf/transform_listener.h>

using namespace std;

std::vector<double> joint_positions(3);
std::vector<double> link_dimensions(3);


bool getScaraDimensionsFromTF(){

    link_dimensions[0]=0.3;
    link_dimensions[1]=0.3;
    link_dimensions[2]=0.0;
    return 1;

}


bool countIK(double x, double y, double z){

    ROS_INFO("input numbers %f %f %f",x,y,z);
    ROS_INFO("input links %f %f %f",link_dimensions[0],link_dimensions[1],link_dimensions[2]);
    double c2 = (pow(x,2.0) + pow(y,2.0) - pow(link_dimensions[0],2.0) - pow(link_dimensions[1],2.0))/(2*link_dimensions[0]*link_dimensions[1]);

    if ((1-c2) > 0.0 ){
        joint_positions[1] = atan2(sqrt(1-c2), c2);
        joint_positions[0] =  atan2(y,x) - atan2(link_dimensions[1]*sin(joint_positions[1]),link_dimensions[0] + link_dimensions[1]*cos(joint_positions[1]));
    }else if ((0.00001 >=(1-c2)) && (-0.00001 <= (1-c2))){
        joint_positions[1] = atan2(0,c2);
        joint_positions[0] =  atan2(y,x) - atan2(link_dimensions[1]*sin(joint_positions[1]),link_dimensions[0] + link_dimensions[1]*cos(joint_positions[1]));
    }
    else{
        ROS_ERROR("Target is out of range");
    }

    ROS_INFO("output joint positions %f %f %f",joint_positions[0],joint_positions[1],joint_positions[2]);
    getchar();

}



int main(int argc, char **argv){

    bool success;


    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    //Ziskanie aktualne pozicie
    geometry_msgs::PoseStamped ws1 = move_group.getCurrentPose();
    ROS_INFO("End effector pose [ x=%f , y=%f , z=%f ]", ws1.pose.position.x, ws1.pose.position.y, ws1.pose.position.z);


   // ros::Sub

    tf::TransformListener listener;



    getScaraDimensionsFromTF();






    return 0;
}
