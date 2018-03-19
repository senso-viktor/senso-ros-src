#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include "torque_sensor_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(ros::Time* msgPtr, SL_Bus_torque_sensor_ros_time_Time const* busPtr);
void convertToBus(SL_Bus_torque_sensor_ros_time_Time* busPtr, ros::Time const* msgPtr);

void convertFromBus(sensor_msgs::JointState* msgPtr, SL_Bus_torque_sensor_sensor_msgs_JointState const* busPtr);
void convertToBus(SL_Bus_torque_sensor_sensor_msgs_JointState* busPtr, sensor_msgs::JointState const* msgPtr);

void convertFromBus(std_msgs::Float64* msgPtr, SL_Bus_torque_sensor_std_msgs_Float64 const* busPtr);
void convertToBus(SL_Bus_torque_sensor_std_msgs_Float64* busPtr, std_msgs::Float64 const* msgPtr);

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_torque_sensor_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_torque_sensor_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr);


#endif
