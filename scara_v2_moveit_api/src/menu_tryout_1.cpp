//
// Created by viktor on 22/08/17.
//
#include <ros/ros.h>
#include "../include/menu_tryout_1.h"

using namespace std;

void buttonsCallback(const scara_v2_moveit_api::scara_button_commands command){

    ROS_INFO("Buttons callback!!");
    ROS_INFO_STREAM(command);

    startJOINT = command.jointcontrol_START;
    stopJOINT = command.jointcontrol_STOP;
    startDEMO1 = command.positioncontrol1_START;
    stopDEMO1 = command.positioncontrol1_STOP;
    startDEMO2 = command.positioncontrol2_START;
    stopDEMO2 = command.positioncontrol2_STOP;
    startCUSTOM = command.positioncontrolcustom_START;
    stopCUSTOM = command.positioncontrolcustom_STOP;
    getInfo = command.getinformation;
    velButton = command.setparameters_velocity;
    accButton = command.setparameters_acceleration;
    planTimeButton = command.setparameters_planningTime;
    numOfAttempsButton = command.setparameters_numberOfAttempts;

}
void jointValuesCallback(const scara_v2_moveit_api::scara_desired_joint_values joint_val){

    ROS_INFO("Joint values callback!!");
    ROS_INFO_STREAM(joint_val);
    joint1 = joint_val.joint1;
    joint2 = joint_val.joint2;
    joint3 = joint_val.joint3;
}
void setParametersCallback(const scara_v2_moveit_api::scara_set_parameters set_param){

    ROS_INFO("Set parameters callback!!");
    ROS_INFO_STREAM(set_param);

    inputVel = set_param.desired_velocity;
    inputAcc = set_param.desired_acceleration;
    inputPlanTime = set_param.desired_planning_time;
    inputNumOfAttemps = set_param.desired_number_of_planning_attempts;
}
void targetPoseCallback(const scara_v2_moveit_api::scara_target_pose pose){

    ROS_INFO("Target pose callback!!");
    ROS_INFO_STREAM(pose);
    desiredPosX = pose.poseX;
    desiredPosY = pose.poseY;
    desiredPosZ = pose.poseZ;
}
void numberCallback(const std_msgs::Int32 menu_number){

    ROS_INFO("Menu number callback!!  (%d)",menu_number);
    input_number = menu_number.data;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "gui_node");
    ros::NodeHandle n1,n2,n3,n4,n5,n6;
    ros::Rate loop_rate(10);

    //Declare publishers
    ros::Publisher infoPublisher = n1.advertise<scara_v2_moveit_api::scara_basic_info>("scara/basic_info", 1000);
    //Declare subscribers
    ros::Subscriber buttonSubscriber = n2.subscribe("scara/button_commands",1000,buttonsCallback);
    ros::Subscriber jointValuesSubscriber = n3.subscribe("scara/desired_joint_values",1000, jointValuesCallback);
    ros::Subscriber parameterSubscriber = n4.subscribe("scara/set_parameters",1000,setParametersCallback);
    ros::Subscriber poseSubscriber = n5.subscribe("scara/target_pose",1000,targetPoseCallback);
    ros::Subscriber numberSubscriber = n6.subscribe("scara/menu_number",1000, numberCallback);

//    printf("\nInput menu number:");
//    scanf("%d",&input_number);
    input_number = 0;

    while (1){
        ROS_INFO_ONCE("while start");

//        if (!input_recognition())
//        {
//            ROS_WARN("program ended because of central stop");
//            ros::shutdown();
//            return 0;
//        }

        ros::spinOnce();
        loop_rate.sleep();
    }




}
