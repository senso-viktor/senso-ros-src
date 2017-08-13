//
// Created by viktor on 10/05/17.
//
//
// Created by viktor on 10/05/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometric_shapes/solid_primitive_dims.h>
#include <visualization_msgs/Marker.h>
#include "scara_v2_moveit_api/pose_and_gripperState.h"

double positionX,positionY,positionZ;
bool gripper = false;


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
void gripperCallback (const scara_v2_moveit_api::pose_and_gripperState gripperInfo){
    ROS_INFO("Heard message : gripperState=%d  posX=%f  posY=%f  posZ=%f", gripperInfo.gripperState, gripperInfo.posX,  gripperInfo.posY,  gripperInfo.posZ);
    gripper = gripperInfo.gripperState;
    positionX = gripperInfo.posX;
    positionY = gripperInfo.posY;
    positionZ = gripperInfo.posZ;
}



void generateShape ( ros::Publisher *marker_pub , uint32_t shape){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    // Publish the marker
    while (marker_pub->getNumSubscribers() < 1)
    {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    marker_pub->publish(marker);

}
void generateCube(ros::Publisher *marker_pub, std::string ParentFrame,int cubeNumber, double x_offset, double y_offset, double z_offset){

    visualization_msgs::Marker marker;
    marker.header.frame_id = ParentFrame ;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    //0.47512 ; 0.23225; 1.0198 pick place
    marker.pose.position.x = x_offset;
    marker.pose.position.y = y_offset;
    marker.pose.position.z = z_offset;
    //
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker_pub->publish( marker );
    //ROS_INFO("published a cube");
}

int main(int argc, char **argv)
{
    bool defaultPosition = true;
    ros::init(argc, argv, "cube_node");
    ros::NodeHandle n, node_gripper;
    ros::Subscriber sub = n.subscribe("gripper_state_topic", 1000, chatterCallback);
    ros::Rate loop_rate(10);

    ros::NodeHandle nn, node_handle;
    //Pub1 1 pre vizualizaciu
    //ros::Publisher marker_pub = nn.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    //Pub 2 pre vizualizaciu
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    //reading gripper state
    ros::Subscriber grip_topic_sub = node_gripper.subscribe("gripper_state",1000,gripperCallback);

    //ros::Publisher grip_topic_pub =  nn.advertise<scara_v2_moveit_api::pose_and_gripperState>("gripper_state", 1000);





    while(ros::ok()) {

        ROS_INFO_ONCE("zacal som publikovat marker");
       if (defaultPosition){
            generateCube(&vis_pub, "world",1, 0.388509 , 0.160504 , 0.969805);
           if (gripper){
               defaultPosition = false;

           }
        }else {
           if (gripper) {
               generateCube(&vis_pub, "tool0",1, 0.0, 0.0, 0.0);
               ROS_INFO_ONCE("Gripper pick");
           } else {
               generateCube(&vis_pub, "world",1, positionX, positionY, positionZ);
               ROS_INFO_ONCE("Gripper place");
               sleep(5);
               defaultPosition = true;
           }
       }


        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
