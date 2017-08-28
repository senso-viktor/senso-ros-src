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

int current_mode = 0;



void modeSelectCallback(const std_msgs::Int32 mode){

    //ROS_INFO("Mode select callback");
    current_mode = mode.data;

}



#endif //PROJECT_SCARA_MENU_H

