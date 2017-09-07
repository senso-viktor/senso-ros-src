#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"
#include "scara_msgs/robot_info.h"


const double RAD_TO_DEG = 57.2957795130;

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void jointControlCallback(const geometry_msgs::PointStamped pointStamped);

    void jointStatesCallback(const sensor_msgs::JointState jointState);

    void getInfoCallback(const scara_msgs::robot_info robotInfo);

    void actualPoseCallback(const geometry_msgs::Pose pose);

    void actualAccCallback(const geometry_msgs::Pose pose);

    void errorCodeCallback(const std_msgs::Int32 errorCode);

    void kktinaCallback(const geometry_msgs::Pose pose);

private slots:
    void on_jointControl_Start_PushButton_3_clicked();

    void on_jointControl_J1_Slider_3_actionTriggered(int action);

    void on_jointControl_J2_Slider_3_actionTriggered(int action);

    void on_jointControl_J3_Slider_3_actionTriggered(int action);

    void on_jointControl_Gripper_Checkbox_3_toggled(bool checked);

    void on_jointControl_Reset_PushButton_3_clicked();

    void on_workingModes_3_tabBarClicked(int index);

    void on_positionControl2_Start_PushButton_3_clicked();

    void on_positionControl2_Stop_PushButton_3_clicked();

    void on_positionControlCustom_Start_PushButton_3_clicked();

    void on_basicInfo_GetInfo_PushButton_3_clicked();

    void on_setParameters_Velocity_PushButton_3_clicked();

    void on_setParameters_Acceleration_PushButton_3_clicked();

    void on_setParameters_PlanningTime_PushButton_3_clicked();

    void on_setParameters_NumOfAttempts_PushButton_3_clicked();

    void on_positionControlCustom_Stop_PushButton_4_clicked();

    void on_jointControl_Stop_PushButton_4_clicked();

    void on_positionControlCustom_Reset_PushButton_5_clicked();

    void on_positionControl_Gripper_Checkbox_4_toggled(bool checked);

    void on_teachMode_teachButton_clicked();

    void on_teachMode_stopTeachButton_clicked();

    void on_teachMode_tabWidget_tabBarClicked(int index);

    void on_teachModeRun_start_pushbutton_clicked();

    void on_teachModeRun_stop_pushbutton_clicked();

    void on_teachMode_tabWidget_2_tabBarClicked(int index);

    void on_teachModeRun_startHand_pushbutton_4_clicked();

    void on_teachModeRun_stopHand_pushbutton_4_clicked();

    void on_teachMode_teachButtonHand_4_clicked();

    void on_teachMode_stopTeachButtonHand_4_clicked();

    void on_centralStop_clicked();

    void on_moveit_checkBox_toggled(bool checked);

    void on_colisionObject_CustomObj_checkButton_toggled(bool checked);

    void on_colisionObject_RealObj_checkButton_toggled(bool checked);

    void on_colisionObject_Reset_pushbutton_clicked();

    void on_colisionObject_Up_pushbutton_clicked();

    void on_colisionObject_Left_pushbutton_clicked();

    void on_colisionObject_Down_pushbutton_clicked();

    void on_colisionObject_Right_pushbutton_clicked();

    void on_setParameters_Torque_PushButton_3_clicked();

    void on_colisionObject_CustomObj_enterpushButton_clicked();

    void on_colisionObject_RealObj_enterPushButton_clicked();

private:
    Ui::MainWindow *ui;

    ros::AsyncSpinner *aspinner;

    bool gripperState = false;
    bool newJointStates = false;
    int teachModeIndex = 0;
    int teachModeIndexHand = 0;

    std_msgs::Bool gripperState_msg, startState_msg, demoState_msg, getInfoState_msg, teachModeState_msg, moveitMode_msg, dispRealObj_msg, dispCustomObj_msg;
    std_msgs::Float64 setParamFloat_msg, realObjSize_msg, customObjSize_msg;
    std_msgs::Int32 setParamInt_msg, modeSelect_msg,teachModeSelect_msg, centralStop_msg, arrows_msg;
    scara_msgs::robot_info robotInfo;
    geometry_msgs::PointStamped jointControl_Values_msg;
    geometry_msgs::Point positionControl_Values_msg;
    geometry_msgs::Pose actualPose_msg;
    sensor_msgs::JointState actualJointStates;

    ros::Publisher jointControl_pub, positionControl_pub, demo_pub, getInfo_pub, setTorq_pub, setVel_pub, setAcc_pub, setPlanTime_pub, setNumOfAttempts_pub;
    ros::Publisher gripperState_pub, start_pub, mode_pub, teachMode_pub, teachMode_startState, centralStop_pub, moveitMode_pub,colObjArrows_pub;
    ros::Publisher setRealColObjSize_pub, setCustomColObjSize_pub, displayRealColObj_pub, displayCustomColObj_pub;
    ros::Subscriber jointControlValues_sub, positionControlValues_sub, demoValues_sub, getInfo_sub, jointStates_sub, actualPose_sub, actualAcc_sub, errorMessage_sub, shit_sub, teachModeTeach_sub, teachModeStopTeach_sub;



};

#endif // MAINWINDOW_H