#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros/ros.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"
#include "scara_msgs/robot_info.h"


const double PI = 3.14159265;

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

private:
    Ui::MainWindow *ui;

    ros::AsyncSpinner *aspinner;

    bool gripperState = false;
    bool newJointStates = false;

    std_msgs::Bool gripperState_msg, stopState_msg, demoState_msg,getInfoState_msg;
    std_msgs::Float64 setParamFloat_msg;
    std_msgs::Int32 setParamInt_msg, modeSelect_msg;
    scara_msgs::robot_info robotInfo;
    geometry_msgs::PointStamped jointControl_Values_msg;
    geometry_msgs::Point positionControl_Values_msg;
    geometry_msgs::Pose actualPose_msg;
    sensor_msgs::JointState actualJointStates;

    ros::Publisher jointControl_pub, positionControl_pub, demo_pub, getInfo_pub, setVel_pub, setAcc_pub, setPlanTime_pub, setNumOfAttempts_pub, gripperState_pub, stop_pub, mode_pub;
    ros::Subscriber jointControlValues_sub, positionControlValues_sub, demoValues_sub, getInfo_sub, jointStates_sub, actualPose_sub, actualAcc_sub;



};

#endif // MAINWINDOW_H
