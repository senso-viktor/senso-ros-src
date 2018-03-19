//
// Created by viktor on 17/10/17.
//

#include "../include/rotary_table_menu.h"
#include "scara_v2_moveit_api/pose_velocity_direction.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "menu_node");
    ros::NodeHandle n, nn;
    ros::Rate loop_rate(20);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    uint8_t data[8];
    can_frame frame;
    frame.can_id = 0x00;
    can = new Can_interface();
    std::vector<int > ids;
    ids.push_back(0x210);
    ids.push_back(0x211);
    ids.push_back(0x212);
    ids.push_back(0x21e);

    try {
        can->initCAN("can0", ids, 10);

    } catch(std::exception& e){
            ROS_ERROR("CAN NOT OPENED due to exeption : %s", e.what());
        return -1;
    }

    ROS_WARN("Init publishers:");
        currentRotationInDeg_pub = n.advertise<std_msgs::Int32>("currentAngleDeg_RT",1000);
        ROS_INFO("currentAngleDeg_RT");
        currentVelocityInDeg_pub = n.advertise<std_msgs::Int32>("currentVelocityPerMinute_RT",1000);
        ROS_INFO("currentAngleDeg_RT");
        currentWorkingState_pub = n.advertise<std_msgs::Int32>("currentWorkingState_RT",1000);
        ROS_INFO("currentWorkingState_RT");
        currentError_pub = n.advertise<std_msgs::Int32>("currentWorkingError_RT",1000);
        ROS_INFO("currentWorkingError_RT");
        tempAndCurrentStatus_pub = n.advertise<scara_v2_moveit_api::status_rt>("currentStatus_RT",1000);

    ROS_WARN("Init subscribers:");
        rotateCommand_sub = nn.subscribe("rotate_DEC_RT",1000,rotateCommandCallback);
        ROS_INFO("rotate_DEC_RT");
        workingStateCommand_sub = nn.subscribe("set_working_mode_RT",1000,workingStateCommandCallback);
        ROS_INFO("set_working_mode_RT");
        temperatureAndCurrent_sub = nn.subscribe("requestTemperatureAndCurrent", 1000, tempAndCurrCallback);
        ROS_INFO("requestTemperatureAndCurrent");
        exitProgram_sub = nn.subscribe("exitProgram_RT",1000,exitProgramCallback);
        ROS_INFO("exitProgram_RT");



    while (ros::ok()){

        if(exit_program)
            break;

        int a= can->readCAN(&frame);
        decodeCANmsg(&frame);

        ros::spinOnce();
        loop_rate.sleep();
    }

    can->closeCAN();


    return 0;
}