//
// Created by viktor on 15/08/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"


sensor_msgs::JointState jointState_msg_;

void jointValuesCallback(const geometry_msgs::Point jointValues){

    //ROS_INFO("subscribe");
    jointState_msg_.header.stamp = ros::Time::now();
    jointState_msg_.position[0] = jointValues.x;
    jointState_msg_.position[1] = jointValues.y;
    jointState_msg_.position[2] = jointValues.z;

    ROS_INFO_STREAM(jointState_msg_.header);
    ROS_INFO("position %f %f %f",jointState_msg_.position[0],jointState_msg_.position[1],jointState_msg_.position[2]);
}

void jointStatesCallback (const sensor_msgs::JointState jointStates){

    jointState_msg_.header.stamp = ros::Time::now();
    jointState_msg_.position[0] = jointStates.position[0];
    jointState_msg_.position[1] = jointStates.position[1];
    jointState_msg_.position[2] = jointStates.position[2];
    ROS_INFO("[Stamp]");
    ROS_INFO_STREAM(jointState_msg_.header.stamp);
    ROS_INFO("[Joint1 Joint2 Joint_GripperBase]");
    ROS_INFO("%f %f %f",jointState_msg_.position[0],jointState_msg_.position[1],jointState_msg_.position[2],jointState_msg_.header.stamp);

}
void initJointStates(){
    jointState_msg_.name.resize(3);
    jointState_msg_.position.resize(3);
    jointState_msg_.velocity.resize(3);
    jointState_msg_.effort.resize(3);

    jointState_msg_.name[0] = "Joint1";
    jointState_msg_.name[1] = "Joint2";
    jointState_msg_.name[2] = "Joint_GripperBase";
}

int main(int argc, char **argv){

    ros::init(argc, argv, "joint_state_node");
    ros::NodeHandle n_sub,n_sub1,n_pub;
    ros::Rate loop_rate(90);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    bool mode = false;

    ros::Publisher publish_jointStates = n_pub.advertise<sensor_msgs::JointState>("/sl_scara_joint_states",1000);
    if (mode){
        ros::Subscriber subscribe_jointValues = n_sub.subscribe("scara_jointStates",1000,jointValuesCallback);
    }else{
        ros::Subscriber subscribe_realJointValues = n_sub1.subscribe("scara_realJointStates",1000,jointStatesCallback);
    }


    initJointStates();

    while (ros::ok()){

        publish_jointStates.publish(jointState_msg_);
        //ROS_INFO("published");
        ros::spinOnce();
        loop_rate.sleep();



    }







    return 0;
}