//
// Created by viktor on 24/04/18.
//

#include "../include/multiple_cylinder_publisher_scara_v3.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "cylinder_visualizer_node");
    ros::NodeHandle n, nn;
    ros::Rate loop_rate(30);

    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    ros::Subscriber gripperState_sub = nn.subscribe("attachToGripper",1000, gripperCallback);


    while(ros::ok()) {
        ROS_INFO_ONCE("[CYLINDER PUBLISHER NODE]: Started publishing cylinders");

        for (int i=1;i<=8;i++){
            generate_pick_cylinders(&vis_pub,i,index_of_picked_cube);
            generate_place_cylinders(&vis_pub,i,index_of_picked_cube);
            generate_attached_cylinder(&vis_pub,attach_to_tool);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}