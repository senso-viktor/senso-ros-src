//
// Created by viktor on 23/08/17.
//

#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include "sensor_msgs/JointState.h"

double max_torque_value = 10.0, torque_value = 0.0;
bool colision_detection = false, colision_detection_ack = false;
bool pushbuttonInterupt = false;
int currentMode = 7;
bool matlabState = false;

sensor_msgs::JointState currentJointStates;

void forceFeedbackThread(){

    ROS_INFO("forcefeedback thread start");
    int i=0;

    while (ros::ok()){

//        if (i == 50000) {
//            ROS_INFO("[t]J1_torq=%f J2_torq=%f (max=%f)",currentJointStates.effort[0],currentJointStates.effort[1], max_torque_value);
//            i = 0;
//        }
//        i++;

        if ((currentJointStates.effort[0] >= max_torque_value) || (currentJointStates.effort[1] >= max_torque_value)){
            ROS_ERROR("[forcefeedback thread] STOP");
            colision_detection = true;
            ROS_ERROR("J1_torq=%f J2_torq=%f",currentJointStates.effort[0],currentJointStates.effort[1]);
            break;
        }
        ros::spinOnce();
    }
}

void pushbuttonCallback(const std_msgs::Byte pushbuttonValue){

    ROS_WARN("pushbutton CALLBACK %d",pushbuttonValue);
    if (pushbuttonValue.data == 1){
        ROS_INFO("If OK");
        pushbuttonInterupt = true;
        if (torque_value < max_torque_value){
            if (pushbuttonInterupt){
                colision_detection = false;
            }
        }else{
            pushbuttonInterupt = false;
        }
    }
}

void jointStatesCallback (const sensor_msgs::JointState jointStates){
    //ROS_WARN("joint states callback");
    currentJointStates = jointStates;
    //ROS_INFO_STREAM(currentJointStates);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "PICK_and_PLACE");
    ros::NodeHandle n1, n2,n3, n4;
    ros::Rate r(2);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(500);

    std_msgs::Byte current_mode;
    current_mode.data = 7;

    ros::Publisher modeSelect = n1.advertise<std_msgs::Byte>("modeSelect",1000);
    ros::Subscriber pushbutton = n2.subscribe("scara_pushbutton",1000,pushbuttonCallback);
    ros::Subscriber scaraJointStates = n3.subscribe("scara_jointStates",1000,jointStatesCallback);

    printf("\nSelect max_torque_value : ");
    scanf("%lf",&max_torque_value);
    ROS_WARN("Max torque value has been set to %f", max_torque_value);
    getchar();
    getchar();

    //Init matlab subscriber

    current_mode.data = 10;
    while (currentJointStates.position.size() <3){
        ROS_INFO("waiting for joint states %d",currentJointStates.position.size());
        modeSelect.publish(current_mode);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_ERROR("matlab init OK!!! %d (sleep 12s)", currentJointStates.position.size());

    sleep(12);

    //referencing done !!!!


    current_mode.data = 7;
    ROS_INFO("Starting mode 7");
    for (int i=0;i<50;i++){
        modeSelect.publish(current_mode);    //pre istotu vypublikujem 50x
        sleep(0.1);
    }
    ROS_INFO("Mode 7 started");
    sleep(2);

    boost::thread fft{forceFeedbackThread}; //zapnutie paralelneho vlakna ktore bude sledovat torque_value

    while (ros::ok()){
        ROS_INFO_ONCE("started while");

        if (colision_detection){
            current_mode.data = 6;
           // ROS_INFO("mode 6");
            for (int i=0;i<3;i++){
                modeSelect.publish(current_mode);
                //sleep(0.1);
            }
            modeSelect.publish(current_mode);
        }
        if (pushbuttonInterupt){
           // ROS_INFO("restart mode 7!!");
            current_mode.data = 7;
            for (int i=0;i<3;i++){
                modeSelect.publish(current_mode);    //pre istotu vypublikujem 10x
            }
            boost::thread fft{forceFeedbackThread};
            pushbuttonInterupt = false;
            //sleep(2);
        }




        ros::spinOnce();
        //loop_rate.sleep();

    }






    return 0;
}