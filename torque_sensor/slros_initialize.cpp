#include "slros_initialize.h"

ros::NodeHandle * SLROSNodePtr;
const std::string SLROSNodeName = "torque_sensor";

// For Block torque_sensor/Subscribe
SimulinkSubscriber<sensor_msgs::JointState, SL_Bus_torque_sensor_sensor_msgs_JointState> Sub_torque_sensor_1;

// For Block torque_sensor/Publish
SimulinkPublisher<std_msgs::Float64, SL_Bus_torque_sensor_std_msgs_Float64> Pub_torque_sensor_2;

void slros_node_init(int argc, char** argv)
{
  ros::init(argc, argv, SLROSNodeName);
  SLROSNodePtr = new ros::NodeHandle();
}

