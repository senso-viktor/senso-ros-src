#include "../include/scara_gui/mainwindow.h"
#include "ui_mainwindow.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>

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
    ros::NodeHandle nn1,nn2,nn3;
    ros::Rate loop_rate(10);

    aspinner = new ros::AsyncSpinner(2);
    aspinner->start();

    //Publishers
    jointControl_pub = n1.advertise<geometry_msgs::PointStamped>("jointControl",1000);
    positionControl_pub = n2.advertise<geometry_msgs::Point>("positionControl",1000);
    demo_pub = n3.advertise<std_msgs::Bool>("demoControl",1000);
    getInfo_pub = n4.advertise<std_msgs::Bool>("getInfo",1000);
    setVel_pub = n5.advertise<std_msgs::Float64>("setVelocity",1000);
    setAcc_pub = n6.advertise<std_msgs::Float64>("setAcceleration",1000);
    setPlanTime_pub = n7.advertise<std_msgs::Float64>("setPlanningTime",1000);
    setNumOfAttempts_pub = n8.advertise<std_msgs::Float64>("setNumberOfAttempts",1000);
    gripperState_pub = n9.advertise<std_msgs::Bool>("gripperState",1000);
    stop_pub = n10.advertise<std_msgs::Bool>("stopState",1000);

    //Subscribers
        //demo_rviz.launch
        jointStates_sub = nn1.subscribe("/move_group/fake_controller_joint_states",1000,&MainWindow::jointStatesCallback, this);
        //demo_matlab_mfile
            //jointStates_sub = nn1.subscribe("scara_jointStates",1000,&MainWindow::jointStatesCallback, this);

    //jointControlValues_sub = nn1.subscribe("jointControlFeedbackValues",1000,&MainWindow::jointControlCallback,this);


}

MainWindow::~MainWindow()
{
    delete ui;
}


//********************************************JOINT CONTROL****************************************************//
void MainWindow::on_jointControl_Start_PushButton_3_clicked()
{
    //Display on GUI
    ui->jointControl_J1_LineEdit->setText(QString::number(ui->jointControl_J1_Slider_3->value() / 100.0) + "rad");
    ui->jointControl_J2_LineEdit->setText(QString::number(ui->jointControl_J2_Slider_3->value() / 100.0) + "rad");
    ui->jointControl_J3_LineEdit->setText(QString::number(ui->jointControl_J3_Slider_3->value() / 1000.0) + "m");
    ui->status_gripper_OnOff_3->display(jointControl_gripperState);

    //Send to ROS
    jointControl_Values_msg.point.x = ui->jointControl_J1_Slider_3->value() / 100.0;
    jointControl_Values_msg.point.y = ui->jointControl_J2_Slider_3->value() / 100.0;
    jointControl_Values_msg.point.z = ui->jointControl_J3_Slider_3->value() / 1000.0;
    if (jointControl_gripperState)
        gripperState_msg.data = true;
    else
        gripperState_msg.data = false;
    stopState_msg.data = false;

    for (int i=0;i<100;i++){
        jointControl_pub.publish(jointControl_Values_msg);
        gripperState_pub.publish(gripperState_msg);
        stop_pub.publish(stopState_msg);
    }

}
void MainWindow::on_jointControl_J1_Slider_3_actionTriggered(int action)
{
    ui->jointControl_J1_LineEdit->setText(QString::number(ui->jointControl_J1_Slider_3->value() / 100.0) + "rad");

}
void MainWindow::on_jointControl_J2_Slider_3_actionTriggered(int action)
{
    ui->jointControl_J2_LineEdit->setText(QString::number(ui->jointControl_J2_Slider_3->value() / 100.0) + "rad");
}
void MainWindow::on_jointControl_J3_Slider_3_actionTriggered(int action)
{
    ui->jointControl_J3_LineEdit->setText(QString::number(ui->jointControl_J3_Slider_3->value() / 1000.0) + "m");
}
void MainWindow::on_jointControl_Gripper_Checkbox_3_toggled(bool checked)
{
    //GUI
    if (checked){
        ui->jointControl_gripper_LineEdit->setText("ON!");
    }else{
        ui->jointControl_gripper_LineEdit->setText("OFF!");
    }
    jointControl_gripperState = checked;
    ui->status_gripper_OnOff_3->display(jointControl_gripperState);

    //ROS
    if (checked)
        gripperState_msg.data = true;
    else
        gripperState_msg.data = false;

    for (int i=0;i<100;i++){
        gripperState_pub.publish(gripperState_msg);
    }

}
void MainWindow::on_jointControl_Stop_PushButton_4_clicked()
{
    stopState_msg.data = true;
    for (int i=0;i<100;i++){
        stop_pub.publish(stopState_msg);
    }

}
void MainWindow::on_jointControl_Reset_PushButton_3_clicked()
{
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
    stopState_msg.data = false;
    for (int i=0;i<100;i++){
        jointControl_pub.publish(jointControl_Values_msg);
        gripperState_pub.publish(gripperState_msg);
        stop_pub.publish(stopState_msg);
    }


}
//********************************************************************************//


//***************************** Position Control custom **************************//
void MainWindow::on_positionControlCustom_Start_PushButton_3_clicked()
{
    //ROS
    positionControl_Values_msg.x = ui->positionControlCustom_X_LineEdit_3->text().toDouble();
    positionControl_Values_msg.y = ui->positionControlCustom_Y_LineEdit_3->text().toDouble();
    positionControl_Values_msg.z = ui->positionControlCustom_Z_LineEdit_3->text().toDouble();




}
void MainWindow::on_positionControlCustom_Stop_PushButton_4_clicked()
{

}

void MainWindow::on_positionControlCustom_Reset_PushButton_5_clicked()
{

}

//*******************************************************************************//


//******************************* DEMO APK *************************************//
void MainWindow::on_positionControl2_Start_PushButton_3_clicked()
{
    ui->positionControl2_LineEdit->setText("DEMO application 2 RUNNING!");
}

void MainWindow::on_positionControl2_Stop_PushButton_3_clicked()
{
    ui->positionControl2_LineEdit->setText("DEMO application 2 STOPPED!");
}
//*****************************************************************************//


//*************************** Get information ********************************//
void MainWindow::on_basicInfo_GetInfo_PushButton_3_clicked()
{
    //Tu budem citat hodnoty z rosu a zapisovat do nizzsich..

    //Len test
    ui->basicInfo_RobotModel_TextBrowser_3->setText("Sem pride robot model");
    ui->basicInfo_ReferenceFrame_TextBrowser_3->setText("Sem pride reference frame");
    ui->basicInfo_EffectorLink_TextBrowser_3->setText("sem pride efector link");
    ui->basicInfo_ActiveJoints_TextBrowser_3->setText("Sem pride active joints");

    ui->basicInfo_X_LCDnum_3->display(0.1);
    ui->basicInfo_Y_LCDnum_6->display(0.2);
    ui->basicInfo_Z_LCDnum_3->display(0.3);
    ui->basicInfo_R_LCDnum_3->display(0.4);
    ui->basicInfo_P_LCDnum_3->display(0.5);
    ui->basicInfo_Y_LCDnum_5->display(0.6);
    ui->basicInfo_W_LCDnum_3->display(0.7);

}
//****************************************************************************//


//*************************** Set information *******************************//
void MainWindow::on_setParameters_Velocity_PushButton_3_clicked()
{
    double vel = ui->setParameters_Velocity_LineEdit_3->text().toDouble();

    //Len test
    ui->status_joint1acc_3->display(vel);

}

void MainWindow::on_setParameters_Acceleration_PushButton_3_clicked()
{
    double acc = ui->setParameters_Acceleration_LineEdit_3->text().toDouble();

    //Len test
    ui->status_joint1acc_3->display(acc);
}

void MainWindow::on_setParameters_PlanningTime_PushButton_3_clicked()
{
    double planningTime = ui->setParameters_PlanningTime_LineEdit_3->text().toDouble();

    //Len test
    ui->status_joint1acc_3->display(planningTime);
}

void MainWindow::on_setParameters_NumOfAttempts_PushButton_3_clicked()
{
    int num = ui->setParameters_NumOfAttempts_LineEdit_3->text().toInt();

    //Len test
    ui->status_joint1acc_3->display(num);
}
//************************************************************************//


//***************************** Tab widget ******************************//
void MainWindow::on_workingModes_3_tabBarClicked(int index)
{
    if (index == 0){
        ui->currentWorkingMode_LineEdit->setText("Info");
    }else{
        ui->currentWorkingMode_LineEdit->setText(QString::number(index));
    }

}
//**********************************************************************//


//********************** Callbacks ************************************//
void MainWindow::jointStatesCallback(const sensor_msgs::JointState jointState){

    ROS_INFO("Subscribe jointStates");
    ui->status_joint1pos_rad_3->display(jointState.position[1]);
    ui->status_joint2pos_rad_3->display(jointState.position[2]);
    ui->status_joint3pos_rad_3->display(jointState.position[3]);
    ui->status_joint1pos_deg_3->display(jointState.position[1]*180/PI);
    ui->status_joint2pos_deg_3->display(jointState.position[2]*180/PI);
    ui->status_joint3pos_deg_3->display(jointState.position[3]*180/PI);

    //Neskor doplnit rychlosti a momenty
    //ROS_INFO("%f %f %f",jointState.velocity[1],jointState.velocity[2],jointState.velocity[3]);
    //ui->status_joint2vel_3->display(jointState.velocity[2]);
    //ui->status_joint3vel_3->display(jointState.velocity[3]);
//    ui->status_joint1torq_3->display(jointState.effort[1]);
//    ui->status_joint2torq_3->display(jointState.effort[2]);
//    ui->status_joint3torq_3->display(jointState.effort[3]);
    ui->status_joint1vel_3->display(0.1);
    ui->status_joint2vel_3->display(0.1);
    ui->status_joint3vel_3->display(0.1);
    ui->status_joint1vel_3->display(0.1);
    ui->status_joint1acc_3->display(0.2);
    ui->status_joint2acc_3->display(0.2);
    ui->status_joint3acc_3->display(0.2);
    ui->status_joint1torq_3->display(0.3);
    ui->status_joint2torq_3->display(0.3);
    ui->status_joint3torq_3->display(0.3);
}
void MainWindow::jointControlCallback(const geometry_msgs::PointStamped pointStamped){
    ROS_INFO("joint control callback");
}
