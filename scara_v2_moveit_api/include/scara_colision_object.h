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
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"
#include "scara_v2_moveit_api/pose_and_gripperState.h"

const double SIZE_Z = 0.26, POS_Z = 1.1;
const double VIRTUALCUBE_X_POS = 0.587, VIRTUALCUBE_Y_POS = 0.58, VIRTUALCUBE_X_SIZE = 0.15, VIRTUALCUBE_Y_SIZE = 0.3;

bool custom_object_enabled = false, real_object_enabled = false, virtual_cube_enabled = false, gripperState = false, lastGripperState = false, cubes_enabled = false;
int number_of_cubes = 0, global_counter = 0, j=0;
double pos_x_real = 0.0, pos_y_real = 0.0, pos_x_cust = 0.0, pos_y_cust = 0.0;
double size_x_real = 0.1, size_y_real = 0.01, size_x_cust = 0.05, size_y_cust = 0.05;
double cube_position_x = 0.0, cube_position_y = 0.0, cube_position_z = 0.0;
double r,g,b;
std::vector<std::vector<double>> cube_id_and_pos, default_cube_id_and_pos;


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

void virtualCubeEnabledCallback(const std_msgs::Bool enabled){

    //ROS_INFO("Virtual cube enabled callback");
    virtual_cube_enabled = enabled.data;

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

void gripperCallback (const scara_v2_moveit_api::pose_and_gripperState gripperInfo){

    ROS_INFO("Heard message : gripperState=%d  posX=%f  posY=%f  posZ=%f", gripperInfo.gripperState, gripperInfo.posX,  gripperInfo.posY,  gripperInfo.posZ);
    gripperState = gripperInfo.gripperState;
    cube_position_x = gripperInfo.posX;
    cube_position_y = gripperInfo.posY;
    cube_position_z = gripperInfo.posZ;

}

void numOfCubesCallback (const std_msgs::Int32 numberOfAllCubes){

    ROS_INFO("number of all cubes = %d", numberOfAllCubes.data);
    number_of_cubes = numberOfAllCubes.data;

    cube_id_and_pos.clear();
    cube_id_and_pos.resize(number_of_cubes);
    for (int i=0;i<number_of_cubes;i++){
        cube_id_and_pos[i].resize(4);
    }
    //Memory
    default_cube_id_and_pos.clear();
    default_cube_id_and_pos.resize(number_of_cubes);
    for (int i=0;i<number_of_cubes;i++){
        default_cube_id_and_pos[i].resize(4);
    }

    ROS_INFO("New vector size %d x %d",cube_id_and_pos.size(), cube_id_and_pos[0].size());
    global_counter = 0;

}

void cubePositionsFromTeachCallback (const geometry_msgs::Point position){

    if (global_counter < number_of_cubes){
        ROS_INFO("new position for cube %f %f %f",position.x,position.y,position.z);
        cube_id_and_pos[global_counter][0] = 0.0;
        cube_id_and_pos[global_counter][1] = position.x;
        cube_id_and_pos[global_counter][2] = position.y;
        cube_id_and_pos[global_counter][3] = position.z;
        default_cube_id_and_pos[global_counter][0] = 0.0;
        default_cube_id_and_pos[global_counter][1] = position.x;
        default_cube_id_and_pos[global_counter][2] = position.y;
        default_cube_id_and_pos[global_counter][3] = position.z;
        global_counter++;
    }


}

void displayCubesCallback (const std_msgs::Bool enabled){

    ROS_INFO("Display cubes %d",enabled.data);
    cubes_enabled = enabled.data;

    while (ros::ok()){


        if (global_counter < number_of_cubes){
            ROS_INFO("Not enought input cubes! %d/%d",global_counter,number_of_cubes);

        }else{
            //cubes_enabled = enabled.data;
            for (int i=0;i<number_of_cubes;i++){
                ROS_INFO("%d [%f %f %f %f]",i,cube_id_and_pos[i][0],cube_id_and_pos[i][1],
                         cube_id_and_pos[i][2],cube_id_and_pos[i][3]);
            }
            sleep(1);
            break;
        }
        sleep(0.1);
        ros::spinOnce();
    }


}

void gripperStateCallback (const scara_v2_moveit_api::pose_and_gripperState gripperInfo){

    ROS_INFO("Heard message : gripperState=%d  posX=%f  posY=%f  posZ=%f", gripperInfo.gripperState, gripperInfo.posX,  gripperInfo.posY,  gripperInfo.posZ);
    gripperState = gripperInfo.gripperState;
    if (!gripperState && lastGripperState){
        ROS_INFO("cube place");
        cube_id_and_pos[j][0] = 0.0;
        cube_id_and_pos[j][1] = gripperInfo.posX;
        cube_id_and_pos[j][2] = gripperInfo.posY;
        cube_id_and_pos[j][3] = gripperInfo.posZ;

        j++;
    }else if (gripperState && !lastGripperState){
        ROS_INFO("cube pick");
        cube_id_and_pos[j][0] = 1.0;
    }
    lastGripperState = gripperState;


}

void publishCustomVisualObject(ros::Publisher *marker_pub){

    //ROS_INFO("Position x=%f y=%f z=%f",pos_x_cust, pos_y_cust, POS_Z);
    //ROS_INFO("Size x=%f y=%f z=%f", size_x_cust, size_y_cust, SIZE_Z);
    //ROS_INFO("Display custom %d",custom_object_enabled);

    visualization_msgs::Marker markerCustom;
    markerCustom.header.frame_id = "world" ;
    markerCustom.header.stamp = ros::Time();
    markerCustom.ns = "custon_object";
    markerCustom.id = 0;
    markerCustom.type = visualization_msgs::Marker::CUBE;

    if (virtual_cube_enabled){
        //ROS_INFO("Virtual cube enabled");
        markerCustom.action = visualization_msgs::Marker::ADD;
        //0.47512 ; 0.23225; 1.0198 pick place
        markerCustom.pose.position.x = VIRTUALCUBE_X_POS;
        markerCustom.pose.position.y = VIRTUALCUBE_Y_POS;
        markerCustom.pose.position.z = POS_Z;
        //
        markerCustom.pose.orientation.x = 0.0;
        markerCustom.pose.orientation.y = 0.0;
        markerCustom.pose.orientation.z = 0.0;
        markerCustom.pose.orientation.w = 1.0;
        markerCustom.scale.x = VIRTUALCUBE_X_SIZE;
        markerCustom.scale.y = VIRTUALCUBE_Y_SIZE;
        markerCustom.scale.z = SIZE_Z;  //0.02;
        markerCustom.color.a = 0.3; // Don't forget to set the alpha!
        markerCustom.color.r = 0.0;
        markerCustom.color.g = 1.0;
        markerCustom.color.b = 1.0;
    }else{
        //ROS_INFO("Virtual cube disabled");
        markerCustom.action = visualization_msgs::Marker::DELETE;
    }

    markerCustom.lifetime = ros::Duration();
    marker_pub->publish(markerCustom);
}

void publishRealVisualObject(ros::Publisher *marker_pub){

    //ROS_INFO("Position x=%f y=%f z=%f",pos_x_real, pos_y_real, POS_Z);
    //ROS_INFO("Size x=%f y=%f z=%f", size_x_real, size_y_real, SIZE_Z);
    //ROS_INFO("Display real %d",real_object_enabled);

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
        //usleep(500000);
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
        //usleep(1000000);
        planning_scene_interface->addCollisionObjects(collision_objects_real);
        //ROS_INFO("Real: added");
        usleep(1000000);
    }else{
        collision_object_real_ids[0] = collision_object_real.id;
        planning_scene_interface->removeCollisionObjects(collision_object_real_ids);
        //ROS_INFO("Real: removed");
        //usleep(1000000);
    }
}

void changeColorOfCube(int number){

    switch (number){
        case 0:
        {
            r = 1;
            g = 0;
            b = 0;
            break;
        }
        case 1:
        {
            r = 0;
            g = 1;
            b = 0;
            break;
        }
        case 2:
        {
            r = 0;
            g = 0;
            b = 1;
            break;
        }
        case 3:
        {
            r = 1;
            g = 1;
            b = 0;
            break;
        }
        case 4:
        {
            r = 1;
            g = 0;
            b = 1;
            break;
        }
        case 5:
        {
            r = 0;
            g = 1;
            b = 1;
            break;
        }
        case 6:
        {
            r = 1;
            g = 0.5;
            b = 0;
            break;
        }
        case 7:
        {
            r = 1;
            g = 0;
            b = 0.5;
            break;
        }
        case 8:
        {
            r = 0.5;
            g = 0;
            b = 1;
            break;
        }
        case 9:
        {
            r = 0.5;
            g = 1;
            b = 0;
            break;
        }
        case 10:
        {
            r = 0;
            g = 1;
            b = 0.5;
            break;
        }
        default:
        {
            r = 1;
            g = 1;
            b = 1;
            break;
        }

    }



}

bool generateCube(visualization_msgs::Marker *vis_marker,ros::Publisher *marker_pub, int number){

    if (cube_id_and_pos[number][0] == 0.0){
        vis_marker->header.frame_id = "world";
    }else if (cube_id_and_pos[number][0] == 1.0){
        vis_marker->header.frame_id = "tool0";
    }else{
        ROS_INFO("[ERROR] : Not valid mode in generateCube");
        return false;
    }
    vis_marker->header.stamp = ros::Time();
    std::string name_space = "cube" + number;
    vis_marker->ns = name_space;
    vis_marker->id = 0;
    vis_marker->type = visualization_msgs::Marker::CYLINDER;
    if (cubes_enabled){
        vis_marker->action = visualization_msgs::Marker::ADD;
        //0.47512 ; 0.23225; 1.0198 pick place

        if (cube_id_and_pos[number][0] == 1.0){ //pick
            vis_marker->pose.position.x = 0;
            vis_marker->pose.position.y = 0;
            vis_marker->pose.position.z = 0;
        }else{  //place
            vis_marker->pose.position.x = cube_id_and_pos[number][1];
            vis_marker->pose.position.y = cube_id_and_pos[number][2];
//            vis_marker->pose.position.z = cube_id_and_pos[number][3] - 0.06;
            vis_marker->pose.position.z = 0.98;
        }
        vis_marker->pose.orientation.x = 0.0;
        vis_marker->pose.orientation.y = 0.0;
        vis_marker->pose.orientation.z = 0.0;
        vis_marker->pose.orientation.w = 1.0;
        vis_marker->scale.x = 0.02;
        vis_marker->scale.y = 0.02;
        vis_marker->scale.z = 0.025;
        changeColorOfCube(number);
        vis_marker->color.a = 1.0; // Don't forget to set the alpha!
        vis_marker->color.r = r;
        vis_marker->color.g = g;
        vis_marker->color.b = b;

    }else{
        vis_marker->action = visualization_msgs::Marker::DELETE;
    }

    vis_marker->lifetime = ros::Duration();
    marker_pub->publish( *vis_marker );
    //ROS_INFO("displayed a cube");

}   //Just for simulation
#endif //PROJECT_SCARA_COLISION_OBJECT_H
