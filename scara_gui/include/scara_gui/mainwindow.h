#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
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
#include "std_msgs/Byte.h"
#include "scara_msgs/robot_info.h"
#include <stdlib.h>

#define WIDTH 3
#define HEIGHT 20

const double RAD_TO_DEG = 57.2957795130;
const double DEG_TO_RAD = 0.0174532925;
const double MIN_DISPLAY_VALUE = 0.0001;


namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QTimer *timer;

    void jointControlCallback(const geometry_msgs::PointStamped pointStamped);

    void posesAndVelocitiesCallback(const geometry_msgs::Pose poseAndVelocity);

    void jointStatesCallback(const sensor_msgs::JointState jointState);

    void moveitJointStatesCallback(const sensor_msgs::JointState jointState);

    void getInfoCallback(const scara_msgs::robot_info robotInfo);

    void actualPoseCallback(const geometry_msgs::Pose pose);

    void actualAccCallback(const geometry_msgs::Point accValues);

    void errorCodeCallback(const std_msgs::Int32 errorCode);

    void kktinaCallback(const geometry_msgs::Pose pose);

    void pushButtonCallback(const std_msgs::Byte pushButtonState);

    void lightBarrierCallback(const std_msgs::Byte lightBarrierState);

    void gripperCommandCallback(const std_msgs::Byte gripperCommandState);

    void desiredPoseCallback(const geometry_msgs::Point desiredPose);

    void torqueJ1Callback(const std_msgs::Float64 torque);

    void torqueJ2Callback(const std_msgs::Float64 torque);

    bool filterValues (double inputValue);


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

    void on_moveit_gripper_checkBox_toggled(bool checked);

    void on_colisionObject_CustomObj_posChangeenterpushButton_2_clicked();

    void on_colisionObject_CustomObj_SizeenterpushButton_clicked();

    void on_colisionObject_RealObj_SizeenterpushButton_2_clicked();

    void on_setParameters_Precision_PushButton_4_clicked();

    void on_teachModeRun_collision_checkbox_toggled(bool checked);

    void on_positionControl2_collision_checkbox_toggled(bool checked);

    void on_teachModeRun_collisiongui_checkbox_toggled(bool checked);

    void on_positionControlCustom_collision_checkbox_toggled(bool checked);

    void on_jointControl_collision_checkbox_toggled(bool checked);

    void displayValues();

    void decodeErrorMessage(uint8_t error_message);

    //void init_shared_variables();

private:
    Ui::MainWindow *ui;

    ros::AsyncSpinner *aspinner;

    bool gripperState = false;
    bool newJointStates = false;
    int teachModeIndex = 0;
    int teachModeIndexHand = 0;
    int lastErrorCode = 999;
    double lastValueJ1 = 9.99 , lastValueJ2 = 9.99 ,lastValueJ3 = 9.99;
    int j = 0;

    //Shared variables
    double disp_curr_joint1_pos=9.99, disp_curr_joint2_pos=9.99, disp_curr_joint3_pos=9.99;
    double disp_curr_joint1_vel=9.99, disp_curr_joint2_vel=9.99, disp_curr_joint3_vel=9.99;
    double disp_curr_joint1_acc=9.99, disp_curr_joint2_acc=9.99, disp_curr_joint3_acc=9.99;
    double disp_curr_joint1_torq=9.99, disp_curr_joint2_torq=9.99, disp_curr_joint3_torq=9.99;
    double disp_des_cart_pos_x=9.99, disp_des_cart_pos_y=9.99, disp_des_cart_pos_z=9.99;
    double disp_curr_cart_pos_x=9.99, disp_curr_cart_pos_y=9.99, disp_curr_cart_pos_z=9.99;
    double disp_rob_stat_x=9.99, disp_rob_stat_y=9.99, disp_rob_stat_z=9.99, disp_rob_stat_rx=9.99,disp_rob_stat_ry=9.99, disp_rob_stat_rz=9.99,disp_rob_stat_rw=9.99;
    uint8_t disp_gripper_state=9, disp_light_barrier=9, disp_push_button, disp_err_code=9;
    std::string disp_rob_model="9.99", disp_refer_frame="9.99", disp_effect_link="9.99", disp_active_joints="9.99";


    //Shared variables
    //std::vector<double> disp_curr_joint_pos_DEG, disp_curr_joint_vel, disp_curr_joint_acc, disp_curr_joint_torq;
    //std::vector<double> disp_des_carthesian_pos, disp_curr_carthesian_pos, disp_curr_rob_status;
//    uint8_t disp_gripper_state, disp_light_barrier, disp_push_button, disp_err_code;
//    std::string disp_rob_model, disp_refer_frame, disp_effect_link, disp_active_joints;

    std_msgs::Bool gripperState_msg, startState_msg, demoState_msg, getInfoState_msg, teachModeState_msg, moveitMode_msg, dispRealObj_msg, dispCustomObj_msg;
    std_msgs::Float64 setParamFloat_msg, realObjSize_msg, customObjSize_msg;
    std_msgs::Int32 setParamInt_msg, modeSelect_msg,teachModeSelect_msg, centralStop_msg, arrows_msg;
    scara_msgs::robot_info robotInfo;
    geometry_msgs::PointStamped jointControl_Values_msg;
    geometry_msgs::Point positionControl_Values_msg, posCustomObj_msg, sizeCustomObj_msg, sizeRealObj_msg;
    geometry_msgs::Pose actualPose_msg;
    sensor_msgs::JointState actualJointStates;

    ros::Publisher jointControl_pub, positionControl_pub, demo_pub, getInfo_pub, setTorq_pub, setVel_pub, setAcc_pub, setPlanTime_pub, setNumOfAttempts_pub, setPrecision_pub;
    ros::Publisher gripperState_pub, start_pub, mode_pub, teachMode_pub, teachMode_startState, centralStop_pub, moveitMode_pub,colObjArrows_pub;
    ros::Publisher setCustomObjPos_pub, setRealColObjSize_pub, setCustomColObjSize_pub, displayRealColObj_pub, displayCustomColObj_pub;
    ros::Subscriber jointControlValues_sub, positionControlValues_sub, demoValues_sub, getInfo_sub, jointStates_sub, actualPose_sub, actualAcc_sub, errorMessage_sub;
    ros::Subscriber shit_sub, teachModeTeach_sub, teachModeStopTeach_sub, pushButton_sub, lightBarrier_sub, gripperCommand_sub, desiredPose_sub, moveit_jointStates, torqJ1_sub, torqJ2_sub;




};

#endif // MAINWINDOW_H