//
// Created by viktor on 10/09/17.
//

#ifndef PROJECT_SCARA_COLISION_OBJECT_H
#define PROJECT_SCARA_COLISION_OBJECT_H

#include "ros/ros.h"
#include <geometric_shapes/solid_primitive_dims.h>
#include <visualization_msgs/Marker.h>
#include "moveit_msgs/CollisionObject.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"

const double SIZE_Z = 0.26, POS_Z = 1.1;

bool custom_object_enabled = false, real_object_enabled = false;
double pos_x_real = 0.0, pos_y_real = 0.0, pos_x_cust = 0.0, pos_y_cust = 0.0;
double size_x_real = 0.1, size_y_real = 0.01, size_x_cust = 0.05, size_y_cust = 0.05;

visualization_msgs::Marker markerCustom, markerReal;
std::vector<moveit_msgs::CollisionObject> collision_objects_custom, collision_objects_real;
std::vector<std::string> collision_object_custom_ids, collision_object_real_ids;


void customPosCallback (const geometry_msgs::Point positionValue){

    //ROS_INFO("Custom object position callback [x=%f y=%f]",positionValue.x, positionValue.y);
    pos_x_cust = positionValue.x;
    pos_y_cust = positionValue.y;

}

void customSizeCallback (const geometry_msgs::Point sizeValue){

    //ROS_INFO("Custom object size callback [x=%f y=%f]",sizeValue.x, sizeValue.y);
    size_x_cust = sizeValue.x;
    size_y_cust = sizeValue.y;

}

void realPosCallback (const geometry_msgs::Pose positionValue){

    //ROS_INFO("Custom object position callback [x=%f y=%f]",positionValue.x, positionValue.y);
    pos_x_real = positionValue.position.x;
    pos_y_real = positionValue.position.y;

}

void realSizeCallback (const geometry_msgs::Point sizeValue){

    //ROS_INFO("Custom object size callback [x=%f y=%f]",sizeValue.x, sizeValue.y);
    size_x_real = sizeValue.x;
    size_y_real = sizeValue.y;

}

void customObjEnabledCallback (const std_msgs::Bool enabled){

    //ROS_INFO("Custom object enabled callback");
    custom_object_enabled = enabled.data;

}

void realObjEnabledCallback (const std_msgs::Bool enabled){

    //ROS_INFO("Real object enabled callback");
    real_object_enabled = enabled.data;

}

void customObjectPositionChangeCallback (const std_msgs::Int32 movementCommand){

    //ROS_INFO("Custom object movement callback");

    switch (movementCommand.data){
        case 0:         //Reset
            pos_x_cust = 0.0;
            pos_y_cust = 0.0;
            break;
        case 1:         //Up
            pos_x_cust -= 0.05;
            break;
        case 2:         //Left
            pos_y_cust -= 0.05;
            break;
        case 3:         //Down
            pos_x_cust +=0.05;
            break;
        case 4:         //Right
            pos_y_cust +=0.05;
            break;
        default:
            ROS_ERROR("not valid command");
            break;
    }

}

void publishCustomVisualObject(ros::Publisher *marker_pub){

    //ROS_INFO("Position x=%f y=%f z=%f",pos_x_cust, pos_y_cust, POS_Z);
    //ROS_INFO("Size x=%f y=%f z=%f", size_x_cust, size_y_cust, SIZE_Z);
    ROS_INFO("Display custom %d",custom_object_enabled);

    visualization_msgs::Marker markerCustom;
    markerCustom.header.frame_id = "world" ;
    markerCustom.header.stamp = ros::Time();
    markerCustom.ns = "custon_object";
    markerCustom.id = 0;
    markerCustom.type = visualization_msgs::Marker::CUBE;

    if (custom_object_enabled){
        markerCustom.action = visualization_msgs::Marker::ADD;
        //0.47512 ; 0.23225; 1.0198 pick place
        markerCustom.pose.position.x = pos_x_cust;
        markerCustom.pose.position.y = pos_y_cust;
        markerCustom.pose.position.z = POS_Z;
        //
        markerCustom.pose.orientation.x = 0.0;
        markerCustom.pose.orientation.y = 0.0;
        markerCustom.pose.orientation.z = 0.0;
        markerCustom.pose.orientation.w = 1.0;
        markerCustom.scale.x = size_x_cust;
        markerCustom.scale.y = size_y_cust;
        markerCustom.scale.z = SIZE_Z;  //0.02;
        markerCustom.color.a = 1.0; // Don't forget to set the alpha!
        markerCustom.color.r = 0.0;
        markerCustom.color.g = 1.0;
        markerCustom.color.b = 0.0;
    }else{
        markerCustom.action = visualization_msgs::Marker::DELETE;
    }

    markerCustom.lifetime = ros::Duration();
    marker_pub->publish(markerCustom);
    //ROS_INFO("published a cube");
}

void publishRealVisualObject(ros::Publisher *marker_pub){

    //ROS_INFO("Position x=%f y=%f z=%f",pos_x_real, pos_y_real, POS_Z);
    //ROS_INFO("Size x=%f y=%f z=%f", size_x_real, size_y_real, SIZE_Z);
    ROS_INFO("Display real %d",real_object_enabled);

    visualization_msgs::Marker markerReal;
    markerReal.header.frame_id = "world" ;
    markerReal.header.stamp = ros::Time();
    markerReal.ns = "real_object";
    markerReal.id = 0;
    markerReal.type = visualization_msgs::Marker::CUBE;

    if (real_object_enabled){
        markerReal.action = visualization_msgs::Marker::ADD;
        markerReal.pose.position.x = pos_x_real;
        markerReal.pose.position.y = pos_y_real;
        markerReal.pose.position.z = POS_Z;
        markerReal.pose.orientation.x = 0.0;
        markerReal.pose.orientation.y = 0.0;
        markerReal.pose.orientation.z = 0.0;
        markerReal.pose.orientation.w = 1.0;
        markerReal.scale.x = size_x_real;
        markerReal.scale.y = size_y_real;
        markerReal.scale.z = SIZE_Z;
        markerReal.color.a = 1.0;
        markerReal.color.r = 0.0;
        markerReal.color.g = 1.0;
        markerReal.color.b = 0.0;
    }else{
        markerReal.action = visualization_msgs::Marker::DELETE;
    }

    markerReal.lifetime = ros::Duration();
    marker_pub->publish(markerReal);

}

void publishCustomColisionObject(moveit::planning_interface::MoveGroupInterface *mg, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface){

    moveit_msgs::CollisionObject collision_object_custom;
    collision_object_custom.header.frame_id = mg->getPlanningFrame();
    collision_object_custom.id = "custom_collision_object";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size_x_cust;
    primitive.dimensions[1] = size_y_cust;
    primitive.dimensions[2] = SIZE_Z;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = pos_x_cust;
    box_pose.position.y = pos_y_cust;
    box_pose.position.z = POS_Z;

    collision_object_custom.primitives.push_back(primitive);
    collision_object_custom.primitive_poses.push_back(box_pose);
    collision_object_custom.operation = collision_object_custom.ADD;

    collision_objects_custom[0] = collision_object_custom;

    //atach collision object to to planning scene
    if (custom_object_enabled){
        planning_scene_interface->addCollisionObjects(collision_objects_custom);
        //ROS_INFO("Custom: added");
        usleep(500000);
    }else{
        collision_object_custom_ids[0] = collision_object_custom.id;
        planning_scene_interface->removeCollisionObjects(collision_object_custom_ids);
        //ROS_INFO("Custom: removed");
        usleep(500000);
    }
}

void publishRealColisionObject(moveit::planning_interface::MoveGroupInterface *mg, moveit::planning_interface::PlanningSceneInterface *planning_scene_interface){

    moveit_msgs::CollisionObject collision_object_real;
    collision_object_real.header.frame_id = mg->getPlanningFrame();
    collision_object_real.id = "real_collision_object";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = size_x_real;
    primitive.dimensions[1] = size_y_real;
    primitive.dimensions[2] = SIZE_Z;
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = pos_x_real;
    box_pose.position.y = pos_y_real;
    box_pose.position.z = POS_Z;

    collision_object_real.primitives.push_back(primitive);
    collision_object_real.primitive_poses.push_back(box_pose);
    collision_object_real.operation = collision_object_real.ADD;

    collision_objects_real[0] = collision_object_real;

    //atach collision object to to planning scene
    if (real_object_enabled){
        usleep(1000000);
        planning_scene_interface->addCollisionObjects(collision_objects_real);
        //ROS_INFO("Real: added");
        usleep(1000000);
    }else{
        collision_object_real_ids[0] = collision_object_real.id;
        planning_scene_interface->removeCollisionObjects(collision_object_real_ids);
        //ROS_INFO("Real: removed");
        usleep(1000000);
    }
}
#endif //PROJECT_SCARA_COLISION_OBJECT_H
