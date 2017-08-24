//
// Created by viktor on 16/08/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/String.h"
#include <sstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/Point.h"
#include <tf/transform_listener.h>

geometry_msgs::Point getPoseFromTF(std::string source, std::string target) {


    geometry_msgs::Point point;
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform(source, target, ros::Time(0), ros::Duration(1));
        listener.lookupTransform(source, target, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_WARN("OSM planner: %s. Can't update pose from TF, for that will be use the latest source point.",
                 ex.what());
    }

    tf::pointTFToMsg(transform.getOrigin(), point);

    return point;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "PICK_and_PLACE");
    ros::NodeHandle node_handle;
    ros::Rate rate(10.0);
    geometry_msgs::Point point;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    double x_offset = 0, y_offset = 0, z_offset = 0;

    point = getPoseFromTF("world","BaseBox");
    x_offset = x_offset + point.x;
    y_offset = y_offset + point.y;
    //ROS_INFO_STREAM(point);
    ROS_INFO("x=%f y=%f z=%f",x_offset, y_offset, z_offset);
    point = getPoseFromTF("BaseBox","ScaraBase");
    x_offset = x_offset + point.x;
    y_offset = y_offset + point.y;
    //ROS_INFO_STREAM(point);
    ROS_INFO("x=%f y=%f z=%f",x_offset, y_offset, z_offset);
    point = getPoseFromTF("world","tool0");
    z_offset = point.z;
    ROS_INFO("x=%f y=%f z=%f",x_offset, y_offset, z_offset);

//    while (ros::ok()){
//        rate.sleep(); :
//    }


    return 0;
}


//
//geometry_msgs::Point getPoseFromTF(std::string source, std::string target) {
//
//    geometry_msgs::Point point;
//    tf::TransformListener listener;
//    tf::StampedTransform transform;
//
//    try {
//        listener.waitForTransform(source, target, ros::Time(0), ros::Duration(1));
//        listener.lookupTransform(source, target, ros::Time(0), transform);
//    } catch (tf::TransformException ex) {
//        ROS_WARN("OSM planner: %s. Can't update pose from TF, for that will be use the latest source point.",
//                 ex.what());
//    }
//    tf::pointTFToMsg(transform.getOrigin(), point);
//    return point;
//
//}
//void getOffsets(){
//
//
//
//    point = getPoseFromTF("world","BaseBox");
//    x_offset = x_offset + point.x;
//    y_offset = y_offset + point.y;
//    //ROS_INFO("x=%f y=%f z=%f",x_offset, y_offset, z_offset);
//
//    point = getPoseFromTF("BaseBox","ScaraBase");
//    x_offset = x_offset + point.x;
//    y_offset = y_offset + point.y;
//    //ROS_INFO("x=%f y=%f z=%f",x_offset, y_offset, z_offset);
//
//    point = getPoseFromTF("world","tool0");
//    z_offset = point.z;
//    ROS_INFO("final : x=%f y=%f z=%f",x_offset, y_offset, z_offset);
//
//    sleep(5);
//
//}