#ifndef _SLROS_INITIALIZE_H_
#define _SLROS_INITIALIZE_H_

#include "slros_busmsg_conversion.h"
#include "slros_generic.h"

extern ros::NodeHandle * SLROSNodePtr;
extern const std::string SLROSNodeName;

// For Block torque_sensor/Subscribe
extern SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_torque_sensor_sensor_msgs_JointState> Sub_torque_sensor_1;

// For Block torque_sensor/Publish
extern SimulinkPublisher<std_msgs::Float64, SL_Bus_torque_sensor_std_msgs_Float64> Pub_torque_sensor_2;

void slros_node_init(int argc, char** argv);

#endif
