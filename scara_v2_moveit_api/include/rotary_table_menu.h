//
// Created by viktor on 17/10/17.
//

#ifndef PROJECT_ROTARY_TABLE_MENU_H
#define PROJECT_ROTARY_TABLE_MENU_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "scara_v2_moveit_api/pose_velocity_direction.h"
#include "scara_v2_moveit_api/pose_and_gripperState.h"
#include "scara_v2_moveit_api/status_rt.h"
#include <can_interface/can_interface.h>

//*********************************** Global constants ****************************************//
const int MIN_VELOCITY = 0, MAX_VELOCITY = 60;


//*********************************** Global Variables ****************************************//
bool exit_program = 0;
int i = 0;
std_msgs::Int32 int32_msg;
scara_v2_moveit_api::pose_velocity_direction posVelDir_msg;
scara_v2_moveit_api::status_rt status_msg;
ros::Publisher currentRotationInDeg_pub, currentVelocityInDeg_pub, currentWorkingState_pub, currentError_pub, tempAndCurrentStatus_pub;
ros::Subscriber rotateCommand_sub, workingStateCommand_sub, exitProgram_sub, temperatureAndCurrent_sub;
Can_interface *can;

//************************************* Functions *********************************************//
//! \Brief Sets 0 to every uint8_t elemet of array of size size_of_array
void clearArray(uint8_t *array, int size_of_array){
    for (int i=0;i<size_of_array;i++){
        array[i] = 0;
    }

}



//! \Brief According to inputNumber this function sends hexadecimal number with id=0x200 via CAN to RT
void sendDesiredWorkingState(int inputNumber){

    /** Description of each inputNumber is explained in workingStateCommandCallback*/

    uint8_t data_to_send[8];            //Create data array to be send
    can_frame frame;                    //Create CAN frame
    frame.can_id = 0x200;               //Define ID of frame
    frame.can_dlc = 0x1;                  //Define Length of frame

    switch (inputNumber){
        case 1: //OFF
        {
            ROS_INFO("desired state OFF");
            frame.data[0] = 0x0;
            for (int i = 1; i < 8; i++) frame.data[i] = 0;
            //ROS_WARN("Zapis CAN:  id %x dlc %x", frame.can_id, frame.can_dlc);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);

               int a = can->writeCAN(&frame);
               if (a > -1) {                              //Write CAN message to CAN bus
                   ROS_INFO("CAN write OK");
               } else
                   ROS_INFO("CAN write not OK");
            break;
        }
        case 2: //READY
        {
            ROS_INFO("desired state READY");
           frame.data[0] = 0x12;
            for (int i = 1; i < 8; i++) frame.data[i] = 0;
            //ROS_WARN("Zapis CAN:  id %x dlc %x", frame.can_id, frame.can_dlc);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);
            can->writeCAN(&frame);                                                  //Write CAN message to CAN bus
            break;
        }
        case 3: //ON
        {
            ROS_INFO("desired state ON");
            frame.data[0] = 0x14;
            for (int i = 1; i < 8; i++) frame.data[i] = 0;
            //ROS_WARN("Zapis CAN:  id %x dlc %x", frame.can_id, frame.can_dlc);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);
            can->writeCAN(&frame);                                                  //Write CAN message to CAN bus
            break;
        }
        case 4: //ERROR
        {
            ROS_INFO("desired state ERROR");
            frame.data[0] = 0x1f;
            for (int i = 1; i < 8; i++) frame.data[i] = 0;
            //ROS_WARN("Zapis CAN:  id %x dlc %x", frame.can_id, frame.can_dlc);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);
            can->writeCAN(&frame);                                                  //Write CAN message to CAN bus
            break;
        }
        default:
        {
            ROS_ERROR("Invalid number");
            frame.data[0] = 0x12;
            for (int i = 1; i < 8; i++) frame.data[i] = 0;
            //ROS_WARN("Zapis CAN:  id %x dlc %x", frame.can_id, frame.can_dlc);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);
            can->writeCAN(&frame);                                                  //Write CAN message to CAN bus
            sleep(1);
            can->writeCAN(&frame);
            sleep(1);

            frame.data[0] = 0x10;
            for (int i = 1; i < 8; i++) frame.data[i] = 0;
            //ROS_WARN("Zapis CAN:  id %x dlc %x", frame.can_id, frame.can_dlc);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);
            can->writeCAN(&frame);                                                  //Write CAN message to CAN bus
            sleep(1);
            can->writeCAN(&frame);
            sleep(1);
            ROS_ERROR("Rotary table is now in OFF state ( due to bad input number)");
            break;
        }
    }

}

//! \Brief According to ID of input frame it decodes the incomming CAN message and sends it to GUI
void decodeCANmsg(can_frame *frame){

    //ROS_INFO("input id %x",frame->can_id);
    switch (frame->can_id){
        case 0x210:     //Status answer
        {
            //ROS_INFO("*** Received CAN msg *** [id = %x]",frame->can_id);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame->data[i]);
            int32_msg.data = 0;                                         //Send STATUS
            memcpy(&int32_msg.data,frame->data,2*sizeof(uint8_t));    //Posibility 2 (Status msg)
            currentWorkingState_pub.publish(int32_msg);                       //Send status msg
            //ROS_INFO("Sending STATUS msg dec=%d (hex=%x)",int32_msg.data,int32_msg.data);

            int32_msg.data = 0;                                         //Send ERROR
            memcpy(&int32_msg.data,frame->data+2,2*sizeof(uint8_t));  //Posibility 2 (Error msg)
            currentError_pub.publish(int32_msg);
            //ROS_INFO("Sending ERROR msg dec=%d (hex=%x)",int32_msg.data,int32_msg.data);
            break;
        }
        case 0x211:     //Position and Velocity answer
        {
            //ROS_INFO("*** Received CAN msg *** [id = %x]",frame->can_id);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame->data[i]);
            int32_msg.data = 0;                                         //Current Position
            memcpy(&int32_msg.data,frame->data,2*sizeof(uint8_t));    //Posibility 2
            currentRotationInDeg_pub.publish(int32_msg);
            //ROS_INFO("Sending current POSITION dec=%d (hex=%x) [inc]",int32_msg.data,int32_msg.data);

            int32_msg.data = 0;                                         //Current Difference to desired angle
            memcpy(&int32_msg.data,frame->data+2,2*sizeof(uint8_t));  //Posibility 2
            //ROS_INFO("Differece between current and desired angle dec=%d (hex=%x) [inc]",int32_msg.data,int32_msg.data);

            int32_msg.data = 0;                                         //Desired max velocity
            memcpy(&int32_msg.data,frame->data+4,2*sizeof(uint8_t));  //Posibility 2
            //currentRotationInDeg_pub.publish(int32_msg);
            //ROS_INFO("Desired max velocity dec=%d (hex=%x) [1/min]",int32_msg.data,int32_msg.data);

            int32_msg.data = 0;                                         //Current velocity
            memcpy(&int32_msg.data,frame->data+6,2*sizeof(uint8_t));  //Posibility 2
            currentVelocityInDeg_pub.publish(int32_msg);
            //ROS_INFO("Desired max velocity dec=%d (hex=%x) [1/min]",int32_msg.data,int32_msg.data);
            break;
        }
        case 0x212:     //Basic Position and Revolution answer
        {
            //ROS_INFO("*** Received CAN msg *** [id = %x]",frame->can_id);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame->data[i]);
            int actualBasicPosition;                                    //Actual Basic Position
            memcpy(&actualBasicPosition, frame->data, 4*sizeof(uint8_t));
            //ROS_INFO("Actual Basic Position is %d",actualBasicPosition);

            int revolutionCounter;                                      //Revolution counter (not implemented in CAN)
            memcpy(&revolutionCounter,frame->data+4,4*sizeof(uint8_t));
            //ROS_INFO("Revolution counter is %d  [rev]",actualBasicPosition);
            break;
        }
        case 0x21e:
        {
            //ROS_INFO("*** Received CAN msg *** [id = %x]",frame->can_id);
            //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame->data[i]);
            memcpy(&status_msg.power_stage_temperature, frame->data, 2*sizeof(uint8_t));
            memcpy(&status_msg.microprocessor_temperature, frame->data+2, 2*sizeof(uint8_t));
            memcpy(&status_msg.chopper_temperature, frame->data+4, 2*sizeof(uint8_t));
            memcpy(&status_msg.filtered_motor_current, frame->data+6, 2*sizeof(uint8_t));
            tempAndCurrentStatus_pub.publish(status_msg);
            break;
        }
        default:
        {
            break;
        }
    }

}

//! \Brief Sends a request message to RT to get response of current temperatures
void requestTemperature(){

        uint8_t data[8];
        clearArray(data,8);
        can_frame frame;                                //Create CAN frame
        frame.can_id = 0x20e;                           //Define header of CAN message
        frame.can_dlc = 0;                              //Define lenght of CAN message
        memcpy(&frame.data, data, sizeof(data));        //Copy data to CAN frame
        for (int i = 0; i < 8; i++) frame.data[i] = 0;  //Set zeros to data
        //for (int i = 0; i < 8; i++) ROS_INFO("%X", frame.data[i]);
        can->writeCAN(&frame);                          //Send message via CAN

}               //******************************************** DOKONCIT !!!!!!! *****************************************//

//! \Brief Check and modify inputNumber(float) between down and up limit
float inLimits_float(float inputNumber, float downLimit, float upLimit){

    if (inputNumber < downLimit){
        ROS_WARN("down limit(%f) reached !",downLimit);
        return downLimit;
    }else if (inputNumber >= upLimit){
        ROS_WARN("up limit(%f) reached !",upLimit);
        return upLimit;
    }else{
        return inputNumber;
    }

}

//! \Brief Check and modify inputNumber(int) between down and up limit
int inLimits_int(int inputNumber, int downLimit, int upLimit){

    if (inputNumber <= downLimit){
        ROS_WARN("down limit(%d) reached !",downLimit);
        return downLimit;
    }else if (inputNumber >= upLimit){
        ROS_WARN("up limit(%d) reached !",upLimit);
        return upLimit;
    }else{
        return inputNumber;
    }

}

int normalizeToRange2PI(int inputNumber){

    int modifiedAngleInt = inputNumber, k=0;

    if (modifiedAngleInt < 0){
        ROS_INFO("Current angle less than 0 (%d)",modifiedAngleInt);
        k = -(modifiedAngleInt/3600);
        modifiedAngleInt = (k+1)*3600 + modifiedAngleInt;
        ROS_INFO("Current angle modified to (%d)",modifiedAngleInt);
    }

    if (modifiedAngleInt >= 3600){
        ROS_INFO("Angle is over 3600 (%d)",modifiedAngleInt);
        modifiedAngleInt = modifiedAngleInt % 3600;
        ROS_INFO("Angle is over 3600, modif angle is %d",modifiedAngleInt);
    }else{
        ROS_INFO("Angle OK %d",modifiedAngleInt);
    }

    return modifiedAngleInt;

}

//************************************** Callbacks ********************************************//
//! \Brief This function is called immediately after an incomming message of rotation+velocity+direction has arrived from GUI
void rotateCommandCallback(const scara_v2_moveit_api::pose_velocity_direction desiredPositionVelocityDirection){

    /**_____________Description of input values:_____________**/
    /**      rotation is in [increment]                      **/
    /**      1 increment == 0.1 degrees                      **/
    /**      velocity is in [rotations per minute]           **/
    /**      direction- Clockwise (0) Anticlockwise(1)       **/
    /**********************************************************/

    //ROS_INFO("des rot =%d , des vel=%d des dir=%d", desiredPositionVelocityDirection.rotation, desiredPositionVelocityDirection.velocity, desiredPositionVelocityDirection.direction);
    int rot = normalizeToRange2PI(desiredPositionVelocityDirection.rotation);
    int vel = inLimits_int(desiredPositionVelocityDirection.velocity, 0, 500);
    bool dir = desiredPositionVelocityDirection.direction;
    ROS_INFO("des rot =%d , des vel=%d des dir=%d", rot, vel, dir);
    uint8_t data[8];

    if (!dir){   //change the direction of the rotation
        rot = -rot;
    }
    //ROS_INFO("rot = %d (%x)",rot,rot);

    memcpy(data,&rot,2*sizeof(uint8_t));
    memcpy(data+2,&vel,2*sizeof(uint8_t));

    can_frame frame;                                //Create CAN frame
    frame.can_id = 0x201;                           //Define header of CAN message
    frame.can_dlc = 4;                              //Define lenght of CAN message
    memcpy(&frame.data, data, sizeof(data));        //Copy data to CAN frame
    for (int i = 4; i < 8; i++) frame.data[i] = 0;  //Set 0 to unwanted bytes
    //for (int i = 0; i < 8; i++)
    //    ROS_INFO("%X", frame.data[i]);              //Display CAN data to send
    can->writeCAN(&frame);                          //Send message via CAN


}   /**************    Direction to solve (225) !!!!   ****************/

//! \Brief This function is called immediately after an incomming message of change of working state has arrived from GUI
void workingStateCommandCallback(const std_msgs::Int32 mode){

    /**_____________Description of mode:_____________________**/
    /**      mode == 1  =>  0x10                             **/
    /**      mode == 2  =>  0x12                             **/
    /**      mode == 3  =>  0x14                             **/
    /**      mode == 4  =>  0x1f                             **/
    /**********************************************************/

    ROS_INFO("workingStateCommandCallback : desired mode=%d",mode.data);
    sendDesiredWorkingState(mode.data);

}

void tempAndCurrCallback(const std_msgs::Bool mode){

    if (mode.data)
        requestTemperature();

}

//! \Brief This function is called immediately after an incomming message of exit program has arrived from GUI
void exitProgramCallback(const std_msgs::Bool exitCommand){

    /**_____________Description of mode:_____________________**/
    /**      exitCommand == true  =>  exit main              **/
    /**********************************************************/
    ROS_INFO("%s",exitCommand.data ? "Exit program":"Not exit program");
    exit_program = exitCommand.data;

}



#endif //PROJECT_ROTARY_TABLE_MENU_H
