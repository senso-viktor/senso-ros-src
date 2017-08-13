//
// Created by viktor on 26/05/17.
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometric_shapes/solid_primitive_dims.h>
#include <visualization_msgs/Marker.h>
#include "scara_v2_moveit_api/pose_and_gripperState.h"

double positionX,positionY,positionZ;
bool gripper = false;
std::vector<std::vector<double>> cubeModes(9, std::vector<double>(4));

std::string cubeParentFrane (int number){

    if (number == 1){
        return "hole1";
    }else if (number == 2){
        return "hole2";
    }else if (number == 3){
        return "hole3";
    }else if (number == 4){
        return "hole4";
    }else if (number == 5){
        return "hole5";
    }else if (number == 6){
        return "hole6";
    }else if (number == 7){
        return "hole7";
    }else if (number == 8){
        return "hole8";
    }else{
        return "No valid number of cube!";
    }

}
std::string cubePlaceFrame(int number){

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
        return "No valid number of cube!";
    }
}
std::string generateNamespace (int number){

    if (number == 1){
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
    }else{
        return "No valid number of cube!";
    }

}
void generateCube(ros::Publisher *marker_pub, int number){

    visualization_msgs::Marker marker;

    if (cubeModes[number][0] == 0){
        marker.header.frame_id = cubeParentFrane(number);
    }else if (cubeModes[number][0] == 1){
        marker.header.frame_id = "tool0";
    }else if (cubeModes[number][0] == 2){
        marker.header.frame_id = cubePlaceFrame(number);
        ROS_INFO("Place position : %f %f %f",cubeModes[number][1],cubeModes[number][2],cubeModes[number][3]);
    }else{
        ROS_INFO("[ERROR] : Not valid mode in generateCube");
    }
        marker.header.stamp = ros::Time();
        marker.ns = generateNamespace(number);
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        //0.47512 ; 0.23225; 1.0198 pick place
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        //
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.015;
        marker.scale.y = 0.015;
        marker.scale.z = 0.015;
        if (number == 1){
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }else {
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker.lifetime = ros::Duration();
        marker_pub->publish( marker );

}

void gripperCallback (const scara_v2_moveit_api::pose_and_gripperState gripperInfo){
    ROS_INFO("Heard message : gripperState=%d  posX=%f  posY=%f  posZ=%f", gripperInfo.gripperState, gripperInfo.posX,  gripperInfo.posY,  gripperInfo.posZ);
    gripper = gripperInfo.gripperState;
    positionX = gripperInfo.posX;
    positionY = gripperInfo.posY;
    positionZ = gripperInfo.posZ;
}

int main(int argc, char **argv) {


    ros::init(argc, argv, "cube_node");
    ros::NodeHandle n, node_gripper;
    //ros::Subscriber sub = n.subscribe("gripper_state_topic", 1000, chatterCallback);
    ros::Rate loop_rate(5);

    ros::NodeHandle nn, node_handle;
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    ros::Subscriber grip_topic_sub = node_gripper.subscribe("gripper_state",1000,gripperCallback);

    int num = 0;
    bool lastGripperState = false;
    int numLast=0;

    while(ros::ok()) {

        ROS_INFO_ONCE("[CUBE INFO]: Started publishing cubes");
        /*       for (int i = 1;i<=8;i++) {
                   if (i == 5){
                       generateCube(&vis_pub, i, 1, 0, 0, 0);
                   }else if (i == 2){
                       generateCube(&vis_pub, i, 2, 0, 0, 0.01);
                   }else{
                       generateCube(&vis_pub, i, 0, 0, 0, 0);
                   }

                   ros::spinOnce();
               }*/
        for (int i=1;i<=8;i++){
            generateCube(&vis_pub,i);
        }

        if (lastGripperState==false && gripper==true){
            ROS_INFO("[CUBE] ( <=SCARA ): Gripper pick");
            num++;
            //Nastavenie modu na parentframe = tool0
            if (num <=8) {
                cubeModes[num][0] = 1;
            }

        }else if (lastGripperState==true && gripper==false){
            ROS_INFO("[CUBE] ( <=SCARA ): Gripper Place");
            //Nastavenie modu na parentframe = place table (+ pozicie)
            if (num <= 8) {
                cubeModes[num][0] = 2;
                cubeModes[num][1] = positionX;
                cubeModes[num][2] = positionY;
                cubeModes[num][3] = positionZ;
            }

        }
        lastGripperState = gripper;
//        if (num <= 8) {
//            if (numLast != num) {
//                for (int j = 0; j <= num; j++) {
//                    ROS_INFO("[CUBE INFO]: cube j=%d = ", j);
//                    for (int k = 0; k < 4; k++) {
//                        ROS_INFO_STREAM(cubeModes[j][k]);
//                    }
//                }
//            }
//        }
        //Osetrenie hazadrnych stavov
        if (num <=8) {
            numLast = num;
        }else{
            numLast = 0;
            num = 7;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }




    return 0;
}