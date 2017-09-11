//
// Created by viktor on 10/09/17.
//

#include "../include/scara_colision_object.h"



int main(int argc, char **argv) {

    bool defaultPosition = true;
    ros::init(argc, argv, "scara_colision_object");
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9;
    ros::Rate loop_rate(5);

    moveit::core::RobotStatePtr current_state;
    static const std::string PLANNING_GROUP = "scara_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //Create publisher for real and custom colision object
    ros::Publisher customColObj_pub = n1.advertise<visualization_msgs::Marker>("custom_colision_object", 1000 );
    ros::Publisher realColObj_pub = n2.advertise<visualization_msgs::Marker>("real_colision_object", 1000 );

    //Create subscriber for position and size of colision objects
    ros::Subscriber customPos_sub = n3.subscribe("CustomObjectPosition",1000,customPosCallback);
    ros::Subscriber customSize_sub = n4.subscribe("CustomObjectSize",1000,customSizeCallback);
    ros::Subscriber realPos_sub = n5.subscribe("collisionPose",1000,realPosCallback);
    ros::Subscriber realSize_sub = n6.subscribe("RealObjectSize",1000,realSizeCallback);
    ros::Subscriber customObjEnabled_sub = n7.subscribe("displayCustomColisionObject",1000,customObjEnabledCallback);
    ros::Subscriber realbjEnabled_sub = n8.subscribe("displayRealColisionObject",1000,realObjEnabledCallback);
    ros::Subscriber customObjectPositionChange_sub = n9.subscribe("colisionObjectMovement", 1000, customObjectPositionChangeCallback);

    collision_objects_custom.resize(1);
    collision_object_custom_ids.resize(1);
    collision_objects_real.resize(1);
    collision_object_real_ids.resize(1);



    while (ros::ok()){

        ROS_WARN_ONCE("Publishing collision object node has started!");
        //publishCustomVisualObject(&customColObj_pub);
        //publishRealVisualObject(&realColObj_pub);

        publishCustomColisionObject(&move_group, &planning_scene_interface);
        publishRealColisionObject(&move_group, &planning_scene_interface);

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}