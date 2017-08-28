//
// Created by viktordluhos on 28/08/17.
//

#include "../include/scara_menu.h"


int main(int argc, char **argv){

    ros::init(argc, argv, "menu_node");
    ros::NodeHandle n1,n2,n3,n4,n5,n6;
    ros::NodeHandle nn1,nn2,nn3,nn4,nn5,nn6;
    ros::Rate loop_rate(5);

    //Publishers
//    ros::Publisher infoPublisher = n1.advertise<scara_v2_moveit_api::scara_basic_info>("scara/basic_info", 1000);
    //Subscriber
    ros::Subscriber modeSelect_sub = nn1.subscribe("modeSelect",1000,modeSelectCallback);
    ros::Subscriber startState_sub = nn2.subscribe("startState",1000, startStateCallback);
    ros::Subscriber gripperState_sub = nn3.subscribe("gripperState",1000, gripperStateCallback);
    ros::Subscriber jointControl_sub = nn4.subscribe("jointControl",1000, jointControlCallback);


    while (ros::ok()){
        ROS_INFO_ONCE("while start");

    switch (current_mode){
        case 0:
            while (ros::ok()){
                //Break while loop when the mode has changed
                if (current_mode != 0)
                    break;
                ROS_INFO("Information tab - do nothing");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;

        case 1:
            while (ros::ok()){
                //Break while loop when the mode has changed
                if (current_mode != 1)
                    break;
                if (start_state){
                    ROS_INFO("Joint control tab started");
                    ROS_INFO("Desired joints : %f %f %f",jointControl_jointValues[0],jointControl_jointValues[1],jointControl_jointValues[2]);
                    ROS_INFO("gripper state %d",gripper_state);
                }else{
                    ROS_INFO("Joint control tab stopped");
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 2:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 2)
                    break;

                ROS_INFO("Position control tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 3:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 3)
                    break;

                ROS_INFO("DEMO control tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 4:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 4)
                    break;

                ROS_INFO("Information tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 5:
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 5)
                    break;

                ROS_INFO("Set parameters tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        case 6:         //Ked budem dorabat veci do GUI
            while (ros::ok()){

                //Break while loop when the mode has changed
                if (current_mode != 7)
                    break;

                ROS_INFO("testing tab ...");
                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        default:
            ROS_INFO("No mode selected!");
            break;
    }

        ros::spinOnce();
        loop_rate.sleep();
    }





    return 0;
}