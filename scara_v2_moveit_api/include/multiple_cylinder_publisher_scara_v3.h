//
// Created by viktor on 24/04/18.
//

#ifndef PROJECT_MULTIPLE_CYLINDER_PUBLISHER_SCARA_V3_H
#define PROJECT_MULTIPLE_CYLINDER_PUBLISHER_SCARA_V3_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Bool.h"
#include <geometric_shapes/solid_primitive_dims.h>
#include <visualization_msgs/Marker.h>

const double pi_value = 3.14159265359;
int index_of_picked_cube = 0;
bool attach_to_tool = false, hide_and_reset=false;


//! \Brief Generates the parent frame of pick cylinders
std::string generate_pick_frame (int number){

    if (number == 1){
        return "pickHole1";
    }else if (number == 2){
        return "pickHole8";
    }else if (number == 3){
        return "pickHole7";
    }else if (number == 4){
        return "pickHole6";
    }else if (number == 5){
        return "pickHole5";
    }else if (number == 6){
        return "pickHole4";
    }else if (number == 7){
        return "pickHole3";
    }else if (number == 8){
        return "pickHole2";
    }else{
        return "No valid index of cube!";
    }

}

//! \Brief Generates the parent frame of place cylinders
std::string generate_place_frame(int number){

    if (number == 1){
        return "placeHole1";
    }else if (number == 2){
        return "placeHole2";
    }else if (number == 3){
        return "placeHole3";
    }else if (number == 4){
        return "placeHole4";
    }else if (number == 5){
        return "placeHole5";
    }else if (number == 6){
        return "placeHole6";
    }else if (number == 7){
        return "placeHole7";
    }else if (number == 8){
        return "placeHole8";
    }else{
        return "No valid index of cube!";
    }
}

//! \Brief Generates the parent frame of cylinder attached to tool0
std::string generate_tool_frame(){
    return "tool0";
}

//! \Brief Generates namespace for published cylinders (0 : cylinder attached to tool0, 1-8 : pick cylinders, 11-18 : place cylinders)
std::string generateNamespace (int number){

    if (number == 0){
        return "namespace0";
    }else if (number == 1){
        return "namespace1";
    }else if (number == 2){
        return "namespace2";
    }else if (number == 3){
        return "namespace3";
    }else if (number == 4){
        return "namespace4";
    }else if (number == 5){
        return "namespace5";
    }else if (number == 6){
        return "namespace6";
    }else if (number == 7){
        return "namespace7";
    }else if (number == 8){
        return "namespace8";
    }else if (number == 11){
        return "namespace11";
    }else if (number == 12){
        return "namespace12";
    }else if (number == 13){
        return "namespace13";
    }else if (number == 14){
        return "namespace14";
    }else if (number == 15){
        return "namespace15";
    }else if (number == 16){
        return "namespace16";
    }else if (number == 17){
        return "namespace17";
    }else if (number == 18){
        return "namespace18";
    }else{
        return "No valid number of cube!";
    }

}

//! \Brief This function publishes and visualizes desired pick cylinders
void generate_pick_cylinders(ros::Publisher *marker_pub, int index_of_cylinder, int index_of_picked_cylinder){

    visualization_msgs::Marker marker;
    marker.header.frame_id = generate_pick_frame(index_of_cylinder);
    marker.header.stamp = ros::Time();
    marker.ns = generateNamespace(index_of_cylinder);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //Mozno pridat offset v Z
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.03;
    if (index_of_cylinder>index_of_picked_cylinder){
        marker.color.a = 1.0;
    }else{
        marker.color.a = 0.0;
    }

    if (hide_and_reset){
        marker.color.a = 0.0;
    }

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker_pub->publish( marker );

}

//! \Brief This function publishes and vizualizes desired place cylinders
void generate_place_cylinders(ros::Publisher *marker_pub, int index_of_cylinder, int index_of_placed_cylinder){

    visualization_msgs::Marker marker;
    marker.header.frame_id = generate_place_frame(index_of_cylinder);
    marker.header.stamp = ros::Time();
    marker.ns = generateNamespace(index_of_cylinder+10);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //Mozno pridat offset v Z
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.03;
    if (index_of_cylinder<=index_of_placed_cylinder){
        marker.color.a = 1.0;
    }else{
        marker.color.a = 0.0;
    }
    if ((index_of_cylinder==index_of_placed_cylinder) && attach_to_tool){
        marker.color.a = 0.0;
    }else if ((index_of_cylinder==index_of_placed_cylinder) && !attach_to_tool){
        marker.color.a = 1.0;
    }
    if (hide_and_reset){
        marker.color.a = 0.0;
    }
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker_pub->publish( marker );
}

//! \Brief This function publishes and vizualizes desired cylinder attached to tool0
void generate_attached_cylinder(ros::Publisher *marker_pub, bool attach_state){

    visualization_msgs::Marker marker;
    marker.header.frame_id = generate_tool_frame();
    marker.header.stamp = ros::Time();
    marker.ns = generateNamespace(0);
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    //Mozno pridat offset v Z
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 1;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.03;
    if  (attach_state){
        marker.color.a = 1.0;
    }else{
        marker.color.a = 0.0;
    }
    if (hide_and_reset){
        marker.color.a = 0.0;
    }
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker_pub->publish( marker );

}

//! \Brief Callback from gripper state
void gripperCallback(const std_msgs::Byte gripper_state){

    //ROS_INFO("Gripper state callback [%d]",gripper_state.data);
    if (gripper_state.data == 1){
        index_of_picked_cube++;
        attach_to_tool=true;
        hide_and_reset=false;
    }else if (gripper_state.data ==0) {
        attach_to_tool = false;
        hide_and_reset = false;
    }else{
        attach_to_tool=false;
        index_of_picked_cube=0;
        hide_and_reset = true;
    }

}

#endif //PROJECT_MULTIPLE_CYLINDER_PUBLISHER_SCARA_V3_H
