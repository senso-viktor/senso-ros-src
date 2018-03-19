//
// Created by viktor on 22/08/17.
//

#ifndef PROJECT_MENU_TRYOUT_1_H
#define PROJECT_MENU_TRYOUT_1_H

#include <ros/ros.h>
#include "cstdio"
#include "stdio.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "scara_v2_moveit_api/scara_basic_info.h"
#include "scara_v2_moveit_api/scara_button_commands.h"
#include "scara_v2_moveit_api/scara_desired_joint_values.h"
#include "scara_v2_moveit_api/scara_set_parameters.h"
#include "scara_v2_moveit_api/scara_target_pose.h"

int input_number, last_input_number=999;
double joint1,joint2,joint3;
double desiredPosX,desiredPosY,desiredPosZ;
bool startJOINT = false, stopJOINT = false;
bool startDEMO1 = false, stopDEMO1 = false;
bool startDEMO2 = false, stopDEMO2 = false;
bool startCUSTOM = false, stopCUSTOM = false;
bool getInfo = false;
bool velButton = false, accButton = false, planTimeButton = false ,numOfAttempsButton = false;
double inputVel, inputAcc, inputPlanTime, inputNumOfAttemps;

bool jointControlMenu(){

    if (input_number != last_input_number){
        last_input_number = last_input_number;
        ROS_ERROR("Joint controll");
    }


    while (1){
//        printf("\nInput joint 1 value:");
//        scanf("%lf",&joint1);
//        printf("Input joint 2 value:");
//        scanf("%lf",&joint2);
//        printf("Input joint 3 value:");
//        scanf("%lf",&joint3);
//        ROS_WARN("The input values are J1=%f \tJ2=%f \tJ3=%f",joint1,joint2,joint3);
//        printf("Input menu number[1]:");
//        scanf("%d",&input_number);
//        ROS_WARN("\n Menu number is : %d",input_number);


        if (input_number == 10){
            return false;
        }else if (input_number != 1){
            return true;
        }

        if (startJOINT){
            startJOINT = false;
            ROS_INFO("started joint control ...");
            //Tu nakodit aby vyuzival ten joint control regulator
            //
            //
            //***************************************************

        }else if (stopJOINT){
            //tu zastavit joint control regulator
            //
            //
            //***************************************************
        }


        ROS_INFO("The code is running and waiting for message");

        //loop_rate.sleep();
        ros::spinOnce();
    }
}

bool positionControlDEMO1(){

    ROS_ERROR("Positon controll DEMO 1");

    while (1){
        printf("\nEnter Start (1/0):");
        scanf("%d",&startDEMO1);
        printf("START:%s", startDEMO1 ? "true" : "false");
        printf("\nEnter Stop (1/0):");
        scanf("%d",&stopDEMO1);
        printf("STOP:%s", stopDEMO1 ? "true" : "false");

//        printf("\nInput menu number[2]:");
//        scanf("%d",&input_number);

        if (input_number == 10){
            return false;
        }else if (input_number != 2){
            return true;
        }

        if (startDEMO1 && !stopDEMO1){
            ROS_INFO("Started DEMO 1 !");
            // Tu nakodit to demo 1
            //
            //
            //***********************
        }else if (stopDEMO1){
            ROS_WARN("DEMO 1 STOPPED!");
            // Tu nvynutit zastavenie vykonavania pohybu
            //
            //
            //*****************************************
        }

    }
}

bool positionControlDEMO2(){

    ROS_ERROR("Positon controll DEMO 2");

    while (1){
        printf("\nEnter Start (1/0):");
        scanf("%d",&startDEMO2);
        printf("\nSTART:%s", startDEMO2 ? "true" : "false");
        printf("\nEnter Stop (1/0):");
        scanf("%d",&stopDEMO2);
        printf("\nSTOP:%s", stopDEMO2 ? "true" : "false");

//        printf("\nInput menu number[3]:");
//        scanf("%d",&input_number);

        if (input_number == 10){
            return false;
        }else if (input_number != 3){
            return true;
        }

        if (startDEMO2 && !stopDEMO2){
            ROS_INFO("Started DEMO 2 !");
            // Tu nakodit to demo 1
            //
            //
            //***********************
        }else if (stopDEMO2){
            ROS_WARN("DEMO 2 STOPPED!");
            // Tu nvynutit zastavenie vykonavania pohybu
            //
            //
            //****************************************
        }

    }

}
bool positionControlCustom(){

    ROS_ERROR("Positon controll custom");

    while (1){

        printf("\nInput desired position X:");
        scanf("%lf",&desiredPosX);
        printf("Input desired position Y:");
        scanf("%lf",&desiredPosY);
        printf("Input desired position Z:");
        scanf("%lf",&desiredPosZ);
        printf("\nEnter Start (1/0):");
        scanf("%d",&startCUSTOM);
        printf("\nSTART:%s", startCUSTOM ? "true" : "false");
        printf("\nEnter Stop (1/0):");
        scanf("%d",&stopCUSTOM);
        printf("\nSTOP:%s", stopCUSTOM ? "true" : "false");

//        printf("\nInput menu number[4]:");
//        scanf("%d",&input_number);


        if (input_number == 10){
            return false;
        }else if (input_number != 4){
            return true;
        }

        if (startCUSTOM && !stopCUSTOM){
            ROS_INFO("Started custom positioning !");
            // Tu nakodit to custom positioning
            //
            //
            //***********************
        }else if (stopCUSTOM){
            ROS_WARN("Custom positioning STOPPED!");
            // Tu nvynutit zastavenie vykonavania pohybu
            //
            //
            //****************************************
        }
    }
}
bool getInfoMenu(){

    ROS_ERROR("Get basic information");


    while (1){
        printf("\nGet info? (1/0):");
        scanf("%d",&getInfo);
        printf("\nSTART:%s", getInfo ? "true" : "false");

//        printf("\nInput menu number[5]:");
//        scanf("%d",&input_number);

        if (input_number == 10){
            return false;
        }else if (input_number != 5){
            return true;
        }

        if (getInfo){

            // Tu sa nacitaju data z modelu
            // a poslu sa spat do GUI
            //
            //
            //
            //******************************
            ROS_INFO("Succesfully got info\n and sent back to GUI");
        }


    }


}

bool setParams(){

    if (input_number != last_input_number){
        last_input_number = input_number;
        ROS_ERROR("Set parameters");
    }



    while (1){

//        printf("\nInput desired velocity: ");
//        scanf("%lf",&inputVel);
//        printf("Display? (1/0): ");
//        scanf("%d",&velButton);
//        printf("START:%s", velButton ? "true" : "false");
//        printf("\nInput desired acceleration: ");
//        scanf("%lf",&inputAcc);
//        printf("Display? (1/0):");
//        scanf("%d",&accButton);
//        printf("START:%s", accButton ? "true" : "false");
//        printf("\nInput desired planning time: ");
//        scanf("%lf",&inputPlanTime);
//        printf("Display? (1/0):");
//        scanf("%d",&planTimeButton);
//        printf("START:%s", planTimeButton ? "true" : "false");
//        printf("\nInput desired number of attempts: ");
//        scanf("%lf",&inputNumOfAttemps);
//        printf("Display? (1/0):");
//        scanf("%d",&numOfAttempsButton);
//        printf("START:%s", numOfAttempsButton ? "true" : "false");

//        printf("\nInput menu number[5]:");
//        scanf("%d",&input_number);

        if (input_number == 10){
            return false;
        }else if (input_number != 5){
            return true;
        }

        if (velButton)
            ROS_INFO("Desired velocity is %f m/s",inputVel);
        if (accButton)
            ROS_INFO("Desired acceleration is %f m/s2",inputAcc);
        if (planTimeButton)
            ROS_INFO("Desired planning time is %f s",inputPlanTime);
        if (numOfAttempsButton)
            ROS_INFO("Desired number of attempts is %f ",inputNumOfAttemps);

    }


}

bool infoMenu (){

    ROS_INFO("info menu");
    if (input_number != last_input_number){
        ROS_INFO("In IF");
        last_input_number = input_number;
        ROS_INFO("You are in info menu");
        ROS_INFO("\nWellcome to SCARA\n"
                 "Graphical User Interface\n\n"
                 "Select:\n"
                 "Mode 1 : Joint controll - Apk\n"
                 "Mode 2 : Position controll -Apk\n"
                 "Mode 3 : Position controll (custom IK) - Apk\n"
                 "Mode 4 : Position controll custom move\n"
                 "Mode 5 : Get basic information\n"
                 "Mode 6 : Set parameters\n\n"
                 "This GUI was provided by SENSODRIVE and Viktor Dluhos");

        while (1){
//          printf("\nInput menu number:");
//          scanf("%d",&input_number);

            ROS_WARN("\n Menu number is : %d",input_number);
            if (input_number == 10){
                return false;
            }else if (input_number != 0){
                return true;
            }
            //loop_rate.sleep();
            ros::spinOnce();
        }
    }

}

bool input_recognition(){

    ROS_INFO("Input number is :%d",input_number);

    switch (input_number){
        case 0:
            ROS_INFO("1");
            if (infoMenu())
                return true;
            else
                return false;

        case 1:
            if (jointControlMenu())
                return true;
            else
                return false;

        case 2:
            if (positionControlDEMO1())
                return true;
            else
                return false;

        case 3:
            if (positionControlDEMO2())
                return true;
            else
                return false;

        case 4:
            if (positionControlCustom())
                return true;
            else
                return false;
        case 5:
            if (getInfoMenu())
                return true;
            else
                return false;

        case 6:
            if (setParams())
                return true;
            else
                return false;


        default:
            return false;
    }

}



#endif //PROJECT_MENU_TRYOUT_1_H
