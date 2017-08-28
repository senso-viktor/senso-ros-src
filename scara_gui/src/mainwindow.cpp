#include "../include/scara_gui/mainwindow.h"
#include "ui_mainwindow.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <std_msgs/Int32.h>

#include "ros/ros.h"





MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    int argc;
    char **argv;
    ros::init(argc, argv, "scara_gui_node");
    ros::NodeHandle n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11;
    ros::NodeHandle nn1,nn2,nn3,nn4;
    ros::Rate loop_rate(10);

    ROS_INFO("spinner start");
    aspinner = new ros::AsyncSpinner(2);
    aspinner->start();

    ROS_INFO("GUI init start!");
    //Publishers
    jointControl_pub = n1.advertise<geometry_msgs::PointStamped>("jointControl",1000);
    ROS_INFO("Init publisher jointControl");
    positionControl_pub = n2.advertise<geometry_msgs::Point>("positionControl",1000);
    ROS_INFO("Init publisher positionControl");
    demo_pub = n3.advertise<std_msgs::Bool>("demoControl",1000);
    ROS_INFO("Init publisher demo");
    getInfo_pub = n4.advertise<std_msgs::Bool>("getInfo",1000);
    ROS_INFO("Init publisher getInfo");
    setVel_pub = n5.advertise<std_msgs::Float64>("setVelocity",1000);
    ROS_INFO("Init publisher setVel");
    setAcc_pub = n6.advertise<std_msgs::Float64>("setAcceleration",1000);
    ROS_INFO("Init publisher setAcc");
    setPlanTime_pub = n7.advertise<std_msgs::Float64>("setPlanningTime",1000);
    ROS_INFO("Init publisher setPlanTime");
    setNumOfAttempts_pub = n8.advertise<std_msgs::Int32>("setNumberOfAttempts",1000);
    ROS_INFO("Init publisher setNumOfAttempts");
    gripperState_pub = n9.advertise<std_msgs::Bool>("gripperState",1000);
    ROS_INFO("Init publisher gripperState");
    start_pub = n10.advertise<std_msgs::Bool>("startState",1000);
    ROS_INFO("Init publisher stop");
    mode_pub = n11.advertise<std_msgs::Int32>("modeSelect",1000);
    ROS_INFO("Init publisher mode");

    ROS_INFO("...............................................");
    ROS_INFO("...............................................");

    //Subscribers
    //demo_rviz.launch
    jointStates_sub = nn1.subscribe("move_group/fake_controller_joint_states",1000,&MainWindow::jointStatesCallback, this);
    ROS_INFO("Init subscriber jointStates");
    //demo_matlab_mfile
    //jointStates_sub = nn1.subscribe("scara_jointStates",1000,&MainWindow::jointStatesCallback, this);

    actualAcc_sub = nn2.subscribe("actualAcceleration",1000,&MainWindow::actualAccCallback, this);

    getInfo_sub = nn3.subscribe("getInfoValues",1000,&MainWindow::getInfoCallback, this);
    ROS_INFO("Init subscriber getInfo");

    actualPose_sub = nn4.subscribe("actualPose",1000,&MainWindow::actualPoseCallback, this);
    ROS_INFO("Init subscriber actualPose");



    ROS_WARN("GUI init complete");
    ROS_WARN("STARTING NOW");
}

MainWindow::~MainWindow()
{
    delete ui;
}


//**************************** JOINT CONTROL ***************************************//
void MainWindow::on_jointControl_Start_PushButton_3_clicked(){
    //Display on GUI
    ui->jointControl_J1_LineEdit->setText(QString::number(ui->jointControl_J1_Slider_3->value() / 100.0) + "rad");
    ui->jointControl_J2_LineEdit->setText(QString::number(ui->jointControl_J2_Slider_3->value() / 100.0) + "rad");
    ui->jointControl_J3_LineEdit->setText(QString::number(ui->jointControl_J3_Slider_3->value() / 1000.0) + "m");
    ui->status_gripper_OnOff_3->display(gripperState);

    //Send to ROS
    jointControl_Values_msg.point.x = ui->jointControl_J1_Slider_3->value() / 100.0;
    jointControl_Values_msg.point.y = ui->jointControl_J2_Slider_3->value() / 100.0;
    jointControl_Values_msg.point.z = ui->jointControl_J3_Slider_3->value() / 1000.0;
    if (gripperState)
        gripperState_msg.data = true;
    else
        gripperState_msg.data = false;
    startState_msg.data = true;

    for (int i=0;i<100;i++){
        jointControl_pub.publish(jointControl_Values_msg);
        gripperState_pub.publish(gripperState_msg);
        start_pub.publish(startState_msg);
    }

}

void MainWindow::on_jointControl_J1_Slider_3_actionTriggered(int action){
    ui->jointControl_J1_LineEdit->setText(QString::number(ui->jointControl_J1_Slider_3->value() / 100.0) + "rad");

}

void MainWindow::on_jointControl_J2_Slider_3_actionTriggered(int action){
    ui->jointControl_J2_LineEdit->setText(QString::number(ui->jointControl_J2_Slider_3->value() / 100.0) + "rad");
}

void MainWindow::on_jointControl_J3_Slider_3_actionTriggered(int action){
    ui->jointControl_J3_LineEdit->setText(QString::number(ui->jointControl_J3_Slider_3->value() / 1000.0) + "m");
}

void MainWindow::on_jointControl_Gripper_Checkbox_3_toggled(bool checked){
    //GUI
    if (checked){
        ui->jointControl_gripper_LineEdit->setText("ON!");
    }else{
        ui->jointControl_gripper_LineEdit->setText("OFF!");
    }
    gripperState = checked;
    ui->status_gripper_OnOff_3->display(gripperState);

    //ROS
    if (checked)
        gripperState_msg.data = true;
    else
        gripperState_msg.data = false;

    for (int i=0;i<100;i++){
        gripperState_pub.publish(gripperState_msg);
    }

}

void MainWindow::on_jointControl_Stop_PushButton_4_clicked(){
    startState_msg.data = false;
    for (int i=0;i<100;i++){
        start_pub.publish(startState_msg);
    }

}

void MainWindow::on_jointControl_Reset_PushButton_3_clicked(){
    //GUI
    ui->jointControl_J1_Slider_3->setValue(0);
    ui->jointControl_J2_Slider_3->setValue(0);
    ui->jointControl_J3_Slider_3->setValue(0);
    ui->jointControl_J1_LineEdit->setText(QString::number(0.0) + "rad");
    ui->jointControl_J2_LineEdit->setText(QString::number(0.0) + "rad");
    ui->jointControl_J3_LineEdit->setText(QString::number(0.0) + "m");
    ui->jointControl_Gripper_Checkbox_3->setChecked(false);
    ui->jointControl_gripper_LineEdit->setText("OFF!");

    //ROS
    //Send to ROS
    jointControl_Values_msg.point.x = 0.0;
    jointControl_Values_msg.point.y = 0.0;
    jointControl_Values_msg.point.z = 0.0;
    gripperState_msg.data = false;
    startState_msg.data = true;
    for (int i=0;i<100;i++){
        jointControl_pub.publish(jointControl_Values_msg);
        gripperState_pub.publish(gripperState_msg);
        start_pub.publish(startState_msg);
    }


}
//********************************************************************************//


//***************************** Position Control custom **************************//
void MainWindow::on_positionControlCustom_Start_PushButton_3_clicked(){
    //ROS
    positionControl_Values_msg.x = ui->positionControlCustom_X_LineEdit_3->text().toDouble();
    positionControl_Values_msg.y = ui->positionControlCustom_Y_LineEdit_3->text().toDouble();
    positionControl_Values_msg.z = ui->positionControlCustom_Z_LineEdit_3->text().toDouble();
    if (gripperState)
        gripperState_msg.data = true;
    else
        gripperState_msg.data = false;
    //stopState_msg.data = false;

    for (int i=0;i<100;i++){
        positionControl_pub.publish(positionControl_Values_msg);
        gripperState_pub.publish(gripperState_msg);
        //stop_pub.publish(stopState_msg);
    }



}

void MainWindow::on_positionControlCustom_Stop_PushButton_4_clicked(){
    //stopState_msg.data = true;
    for (int i=0;i<100;i++){
        //stop_pub.publish(stopState_msg);
    }
}

void MainWindow::on_positionControlCustom_Reset_PushButton_5_clicked(){
    //GUI
    ui->positionControlCustom_X_LineEdit_3->setText(QString::number(0.704));
    ui->positionControlCustom_Y_LineEdit_3->setText(QString::number(0.58));
    ui->positionControlCustom_Z_LineEdit_3->setText(QString::number(1.0196));
    ui->positionControl_Gripper_Checkbox_4->setChecked(false);
    //ROS
    //Start Pose of SCARA
    positionControl_Values_msg.x = 0.704;
    positionControl_Values_msg.y = 0.58;
    positionControl_Values_msg.z = 1.0196;
    gripperState_msg.data = false;
    //stopState_msg.data = false;
    for (int i=0;i<100;i++){
        positionControl_pub.publish(positionControl_Values_msg);
        gripperState_pub.publish(gripperState_msg);
        //stop_pub.publish(stopState_msg);
    }
}

void MainWindow::on_positionControl_Gripper_Checkbox_4_toggled(bool checked){

    //GUI
    ui->status_gripper_OnOff_3->display(checked);

    gripperState = checked;
    //ROS
    if (checked)
        gripperState_msg.data = true;
    else
        gripperState_msg.data = false;

    for (int i=0;i<100;i++){
        gripperState_pub.publish(gripperState_msg);
    }
}
//*******************************************************************************//


//******************************* DEMO APK *************************************//
void MainWindow::on_positionControl2_Start_PushButton_3_clicked(){
    //GUI
    ui->positionControl2_LineEdit->setText("DEMO application 2 RUNNING!");

    //ROS
    demoState_msg.data = true;
    for (int i=0;i<100;i++) {
        demo_pub.publish(demoState_msg);
    }
}

void MainWindow::on_positionControl2_Stop_PushButton_3_clicked(){
    //GUI
    ui->positionControl2_LineEdit->setText("DEMO application 2 STOPPED!");

    //ROS
    demoState_msg.data = false;
    for (int i=0;i<100;i++) {
        demo_pub.publish(demoState_msg);
    }
}
//*****************************************************************************//


//*************************** Get information ********************************//
void MainWindow::on_basicInfo_GetInfo_PushButton_3_clicked(){
    //ROS
    //Publish
    getInfoState_msg.data = true;
    for (int i=0;i<100;i++) {
        getInfo_pub.publish(getInfoState_msg);
    }
}
//****************************************************************************//


//*************************** Set information *******************************//
void MainWindow::on_setParameters_Velocity_PushButton_3_clicked(){
    //ROS
    setParamFloat_msg.data = ui->setParameters_Velocity_LineEdit_3->text().toFloat();
    for (int i=0;i<100;i++) {
        setVel_pub.publish(setParamFloat_msg);
    }

}

void MainWindow::on_setParameters_Acceleration_PushButton_3_clicked(){
    //ROS
    setParamFloat_msg.data = ui->setParameters_Acceleration_LineEdit_3->text().toFloat();
    for (int i=0;i<100;i++) {
        setAcc_pub.publish(setParamFloat_msg);
    }
}

void MainWindow::on_setParameters_PlanningTime_PushButton_3_clicked(){
    //ROS
    setParamFloat_msg.data = ui->setParameters_PlanningTime_LineEdit_3->text().toFloat();
    for (int i=0;i<100;i++) {
        setPlanTime_pub.publish(setParamFloat_msg);
    }
}

void MainWindow::on_setParameters_NumOfAttempts_PushButton_3_clicked(){
    //ROS
    setParamInt_msg.data = ui->setParameters_NumOfAttempts_LineEdit_3->text().toInt();
    for (int i=0;i<100;i++) {
        setNumOfAttempts_pub.publish(setParamInt_msg);
    }
}
//************************************************************************//


//***************************** Tab widget ******************************//
void MainWindow::on_workingModes_3_tabBarClicked(int index){
    //GUI
    if (index == 0){
        ui->currentWorkingMode_LineEdit->setText("Info");
        modeSelect_msg.data = 0;
    }else{
        ui->currentWorkingMode_LineEdit->setText(QString::number(index));
        modeSelect_msg.data = index;
    }

    //ROS
    for (int i=0;i<100;i++) {
        mode_pub.publish(modeSelect_msg);

    }

}
//**********************************************************************//



//********************** Callbacks ************************************//
void MainWindow::jointStatesCallback(const sensor_msgs::JointState jointState){

    //ROS_INFO("Subscribe jointStates");
    ui->status_joint1pos_rad_3->display(jointState.position[1]);
    ui->status_joint2pos_rad_3->display(jointState.position[2]);
    ui->status_joint3pos_rad_3->display(jointState.position[3]);
    ui->status_joint1pos_deg_3->display(jointState.position[1]*180/PI);
    ui->status_joint2pos_deg_3->display(jointState.position[2]*180/PI);
    ui->status_joint3pos_deg_3->display(jointState.position[3]*180/PI);

    //Neskor doplnit rychlosti a momenty
    if (jointState.velocity.size() == 4){
        ui->status_joint2vel_3->display(jointState.velocity[1]);
        ui->status_joint2vel_3->display(jointState.velocity[2]);
        ui->status_joint3vel_3->display(jointState.velocity[3]);
    }else{
        ui->status_joint1vel_3->display(0.1);
        ui->status_joint2vel_3->display(0.1);
        ui->status_joint3vel_3->display(0.1);
    }

    if (jointState.effort.size() == 4){
        ui->status_joint1torq_3->display(jointState.effort[1]);
        ui->status_joint2torq_3->display(jointState.effort[2]);
        ui->status_joint3torq_3->display(jointState.effort[3]);
    }else{
        ui->status_joint1torq_3->display(0.3);
        ui->status_joint2torq_3->display(0.3);
        ui->status_joint3torq_3->display(0.3);
    }

    //...........dorobit aj primanie acceleration..........//

    ui->status_joint1acc_3->display(0.2);
    ui->status_joint2acc_3->display(0.2);
    ui->status_joint3acc_3->display(0.2);

}

void MainWindow::jointControlCallback(const geometry_msgs::PointStamped pointStamped){
    ROS_INFO("joint control callback");
}

void MainWindow::getInfoCallback(const scara_msgs::robot_info robotInfo){

    ROS_INFO("robot info callback");
    std::string robot_model = robotInfo.robot_model;
    std::string reference_frame = robotInfo.reference_frame;
    std::string effector_link = robotInfo.efector_link;
    std::string active_joints = robotInfo.active_joints;
    ROS_INFO_STREAM(robot_model);
    ROS_INFO_STREAM(reference_frame);
    ROS_INFO_STREAM(effector_link);
    ROS_INFO_STREAM(active_joints);

//    ui->basicInfo_RobotModel_TextBrowser_3->setText(QString::fromStdString(robotInfo.robot_model));
//    ui->basicInfo_ReferenceFrame_TextBrowser_3->setText(QString::fromStdString(robotInfo.reference_frame));
//    ui->basicInfo_EffectorLink_TextBrowser_3->setText(QString::fromStdString(robotInfo.efector_link));
//    ui->basicInfo_ActiveJoints_TextBrowser_3->setText(QString::fromStdString(robotInfo.active_joints));

    ui->basicInfo_X_LCDnum_3->display(robotInfo.position_x);
    ui->basicInfo_Y_LCDnum_6->display(robotInfo.position_y);
    ui->basicInfo_Z_LCDnum_3->display(robotInfo.position_z);
    ui->basicInfo_R_LCDnum_3->display(robotInfo.orientation_x);
    ui->basicInfo_P_LCDnum_3->display(robotInfo.orientation_y);
    ui->basicInfo_Y_LCDnum_5->display(robotInfo.orientation_z);
    ui->basicInfo_W_LCDnum_3->display(robotInfo.orientation_w);
}   //String to QString !!!!!!

void MainWindow::actualPoseCallback(const geometry_msgs::Pose pose){
    ROS_INFO("Pose callback");

    ui->status_pose_X->display(pose.position.x);
    ui->status_pose_Y->display(pose.position.y);
    ui->status_pose_Z->display(pose.position.z);
}

void MainWindow::actualAccCallback(const geometry_msgs::Pose pose){

}