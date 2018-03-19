#include "../include/rotary_table_gui/mainwindow.h"
#include "ui_mainwindow.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QtCore>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "ros/ros.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    int argc;
    char **argv;
    ros::init(argc, argv, "scara_gui_node");
    ros::NodeHandle n,nn;
    ros::Rate loop_rate(10);

    ROS_INFO("spinner for GUI start\n");
    aspinner = new ros::AsyncSpinner(2);
    aspinner->start();

    ROS_WARN("Init publishers:");
        rotate_DEC_pub = n.advertise<scara_msgs::pose_velocity_direction>("rotate_DEC_RT",1000);
        ROS_INFO("rotate_DEC_RT");
        rotate_HEX_pub = n.advertise<std_msgs::String>("rotate_HEX_RT",1000);
        ROS_INFO("rotate_HEX_RT");
        workingState_pub = n.advertise<std_msgs::Int32>("set_working_mode_RT",1000);
        ROS_INFO("set_working_mode_RT");
        exitProgram_pub = n.advertise<std_msgs::Bool>("exitProgram_RT",1000);
        ROS_INFO("exitProgram_RT");
        temperatureAndCurrent_pub = n.advertise<std_msgs::Bool>("requestTemperatureAndCurrent",1000);
        ROS_INFO("requestTemperatureAndCurrent");

    ROS_WARN("Init subscribers:");
        currentAngleDeg_sub = nn.subscribe("currentAngleDeg_RT",1000,&MainWindow::CurrentAngleCallback, this);
        ROS_INFO("currentAngleDeg_RT");
        currentState_sub = nn.subscribe("currentWorkingState_RT",1000,&MainWindow::CurrentWorkingStateCallback, this);
        ROS_INFO("currentWorkingState_RT");
        currentError_sub = nn.subscribe("currentWorkingError_RT",1000,&MainWindow::CurrentWorkingErrorCallback, this);
        ROS_INFO("currentWorkingError_RT");
        currentVelocityPerMinute_sub = nn.subscribe("currentVelocityPerMinute_RT",1000,&MainWindow::CurrentVelocityCallback, this);
        ROS_INFO("currentVelocityPerMinute_RT");
        status_sub = nn.subscribe("currentStatus_RT",1000,&MainWindow::CurrentStatusCallback, this);
        ROS_INFO("currentStatus_RT");
    ROS_WARN("");
    ROS_WARN("************************* Init end  ********************************");
    ROS_WARN("");

    pose_velocity_direction_msg.velocity = (int) 360;
    ui->maxVelocityDeg_LCD->display(360);
    ui->maxVelocityRad_LCD->display(360*DEGREES_per_SECOND_TO_ROTATIONperMINUTE);

    ui->direction_RIGHT_CB->setChecked(true);
    ui->direction_LEFT_CB->setChecked(false);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::displayCurrentValues);
    timer->start(100);


}

MainWindow::~MainWindow(){

    //killTimer(timerId);
    timer->stop();
    delete ui;
}

/******************* Config tab *********************/
void MainWindow::on_config_OFF_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state OFF - Check working state");
    sendWorkingMode(1);

}

void MainWindow::on_config_READY_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state READY - Check working state");
    sendWorkingMode(2);

}

void MainWindow::on_config_ON_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state ON - Check working state");
    sendWorkingMode(3);

}

void MainWindow::on_config_ERROR_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state ERROR - Check working state");
    sendWorkingMode(4);

}
/*****************************************************/

/************ Request temp and curr *****************/
void MainWindow::on_pushButton_clicked(){

    bool_msg.data = true;
    temperatureAndCurrent_pub.publish(bool_msg);

}
/*****************************************************/

/**************** Relative Control *******************/
void MainWindow::on_relativeControl_slider_SLIDER_actionTriggered(int action){

    //GUI
    ui->relativeControl_slider_LE->setText(QString::number(ui->relativeControl_slider_SLIDER->value()/10.0) + " deg");

}

void MainWindow::on_relativeControl_slider_PB_clicked(){

    if (directionOfRotation){
    desiredAngleInt += ui->relativeControl_slider_SLIDER->value();
    }else{
        desiredAngleInt -= ui->relativeControl_slider_SLIDER->value();
    }
    desiredAngleInt = normalizeToRange2PI(desiredAngleInt);


    //GUI
    ui->relativeControl_slider_LE->setText(QString::number(ui->relativeControl_slider_SLIDER->value()/10.0) + " deg");
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0);
    ui->desiredPositionRad_LCD->display(desiredAngleInt/10.0 * DEG_TO_RAD);
    ui->status_TE->append("Moving[relative] to " + QString::number(ui->relativeControl_slider_SLIDER->value()/10.0) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));
    //ROS
    pose_velocity_direction_msg.rotation = (int)(ui->relativeControl_slider_SLIDER->value());  //deg
    pose_velocity_direction_msg.velocity = floor((ui->MaxVelocity_input_LE->text().toDouble())*DEGREES_per_SECOND_TO_ROTATIONperMINUTE); // deg/s -> rots/min
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

    rotateImg(desiredAngleInt/10.0);

}

void MainWindow::on_relativeControl_input_PB_clicked(){

    //ROS_INFO("double = %f , int = %d",ui->relativeControl_input_LE->text().toDouble()*10.0,(int)(ui->relativeControl_input_LE->text().toDouble()*10.0));
    if (directionOfRotation){
        desiredAngleInt += (int)(ui->relativeControl_input_LE->text().toDouble()*10.0);
    }else{
        desiredAngleInt -= (int)(ui->relativeControl_input_LE->text().toDouble()*10.0);
    }
    desiredAngleInt = normalizeToRange2PI(desiredAngleInt);

    //GUI
    ui->relativeControl_slider_LE->setText(QString::number(desiredAngleInt/10.0) + " deg");
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0);
    ui->desiredPositionRad_LCD->display(desiredAngleInt/10.0 * DEG_TO_RAD);
    ui->status_TE->append("Moving[relative] to " + QString::number(desiredAngleInt/10.0) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));
    //ROS
    pose_velocity_direction_msg.rotation = (int)(ui->relativeControl_input_LE->text().toDouble()*10); //deg
    pose_velocity_direction_msg.velocity = (int)(ui->MaxVelocity_input_LE->text().toDouble()*DEGREES_per_SECOND_TO_ROTATIONperMINUTE); //deg
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

    rotateImg(desiredAngleInt/10.0);

}
/*****************************************************/


/**************** Absolute Control ******************/
void MainWindow::on_absoluteControl_slider_SLIDER_actionTriggered(int action){

    //GUI
    ui->absoluteControl_slider_LE->setText(QString::number(ui->absoluteControl_slider_SLIDER->value()/10.0) + " deg");

}

void MainWindow::on_absoluteControl_slider_PB_clicked(){

    int desiredAngleAbsolute = ui->absoluteControl_slider_SLIDER->value();
    desiredAngleAbsolute = normalizeToRange2PI(desiredAngleAbsolute);

    //GUI
    ui->absoluteControl_slider_LE->setText(QString::number(desiredAngleAbsolute/10.0) + " deg");
    ui->desiredPositionDeg_LCD->display(desiredAngleAbsolute/10.0);
    ui->desiredPositionRad_LCD->display(desiredAngleAbsolute/10.0 * DEG_TO_RAD);
    ui->status_TE->append("Moving[absolute] to " + QString::number(desiredAngleAbsolute/10.0) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));

    if (directionOfRotation){
        if (desiredAngleAbsolute >= desiredAngleInt){
            pose_velocity_direction_msg.rotation = desiredAngleAbsolute - desiredAngleInt ;
        }else if (desiredAngleAbsolute < desiredAngleInt){
            pose_velocity_direction_msg.rotation = 3600 - (desiredAngleInt - desiredAngleAbsolute);
        }
    }else{
        if (desiredAngleAbsolute <= desiredAngleInt){
            pose_velocity_direction_msg.rotation = desiredAngleInt - desiredAngleAbsolute;
        }else if (desiredAngleAbsolute > desiredAngleInt){
            pose_velocity_direction_msg.rotation = 3600 - (desiredAngleAbsolute - desiredAngleInt);
        }
    }

    //ROS_INFO("desired angle= %d current angle = %d angle to rotate = %d",desiredAngleAbsolute, desiredAngleInt, pose_velocity_direction_msg.rotation);
    desiredAngleInt = desiredAngleAbsolute;
    pose_velocity_direction_msg.velocity = (int)((ui->MaxVelocity_input_LE->text().toDouble())*DEGREES_per_SECOND_TO_ROTATIONperMINUTE); // deg/s -> rots/min
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

    rotateImg(desiredAngleAbsolute/10.0);



}

void MainWindow::on_absoluteControl_input_PB_clicked(){

    //GUI
    double temp1 = abs(ui->absoluteControl_input_LE->text().toDouble());
    while (temp1 >=360){
        temp1-=360;
    }
    int desiredAngleAbsolute = floor(temp1*10);

    ui->absoluteControl_slider_LE->setText(QString::number(temp1) + " deg");
    ui->desiredPositionDeg_LCD->display(temp1);
    ui->desiredPositionRad_LCD->display(temp1*DEG_TO_RAD);
    ui->status_TE->append("Moving[absolute] to " + QString::number(temp1) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));


    if (directionOfRotation){
        if (desiredAngleAbsolute >= desiredAngleInt){
            pose_velocity_direction_msg.rotation = desiredAngleAbsolute - desiredAngleInt ;
        }else if (desiredAngleAbsolute < desiredAngleInt){
            pose_velocity_direction_msg.rotation = 3600 - (desiredAngleInt - desiredAngleAbsolute);
        }
    }else{
        if (desiredAngleAbsolute <= desiredAngleInt){
            pose_velocity_direction_msg.rotation = desiredAngleInt - desiredAngleAbsolute;
        }else if (desiredAngleAbsolute > desiredAngleInt){
            pose_velocity_direction_msg.rotation = 3600 - (desiredAngleAbsolute - desiredAngleInt);
        }
    }
    //ROS_INFO("desired angle= %d current angle = %d angle to rotate = %d",desiredAngleAbsolute, desiredAngleInt, pose_velocity_direction_msg.rotation);
    //ROS_INFO("desired rotation msg %d",pose_velocity_direction_msg.rotation);
    desiredAngleInt = desiredAngleAbsolute;
    pose_velocity_direction_msg.velocity = (int)((ui->MaxVelocity_input_LE->text().toDouble())*DEGREES_per_SECOND_TO_ROTATIONperMINUTE); // deg/s -> rots/min
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

    rotateImg(abs(ui->absoluteControl_input_LE->text().toDouble()));

}
/*****************************************************/


/**************** Smooth tune ************************/
void MainWindow::on_smooth_plusHalf_PB_clicked(){

    desiredAngleInt = currentAngleInt + 10;
    desiredAngleInt = normalizeToRange2PI(desiredAngleInt);

    //GUI
    //ROS_INFO("desiredAngleInt = %d",desiredAngleInt);
    ui->status_TE->append("Moving[relative] to " + QString::number(0.1) + " deg in direction RIGHT");
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0);
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0 * DEG_TO_RAD);
    //ROS
    pose_velocity_direction_msg.rotation = 10; //deg=step*10
    pose_velocity_direction_msg.velocity = 60; //deg/s -> ot/min
    pose_velocity_direction_msg.direction = true;
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

}   //step by 1 degree

void MainWindow::on_smooth_plusOne_PB_clicked(){

    desiredAngleInt = currentAngleInt + 100;
    desiredAngleInt = normalizeToRange2PI(desiredAngleInt);

    //GUI
    //ROS_INFO("desiredAngleInt = %d",desiredAngleInt);
    ui->status_TE->append("Moving[relative] to " + QString::number(0.1) + " deg in direction RIGHT");
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0);
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0 * DEG_TO_RAD);
    //ROS
    pose_velocity_direction_msg.rotation = 100; //deg=step*10
    pose_velocity_direction_msg.velocity = 60; //deg/s -> ot/min
    pose_velocity_direction_msg.direction = true;
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

}   //step by 10 degree

void MainWindow::on_smooth_minusHalf_PB_clicked(){

    desiredAngleInt = currentAngleInt - 10;
    desiredAngleInt = normalizeToRange2PI(desiredAngleInt);

    //GUI
    //ROS_INFO("desiredAngleInt = %d",desiredAngleInt);
    ui->status_TE->append("Moving[relative] to " + QString::number(0.1) + " deg in direction RIGHT");
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0);
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0 * DEG_TO_RAD);
    //ROS
    pose_velocity_direction_msg.rotation = 10; //deg=step*10
    pose_velocity_direction_msg.velocity = 60; //deg/s -> ot/min
    pose_velocity_direction_msg.direction = false;
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

}

void MainWindow::on_smooth_minusOne_PB_clicked(){


    desiredAngleInt = currentAngleInt - 100;
    desiredAngleInt = normalizeToRange2PI(desiredAngleInt);

    //GUI
    //ROS_INFO("desiredAngleInt = %d",desiredAngleInt);
    ui->status_TE->append("Moving[relative] to " + QString::number(0.1) + " deg in direction RIGHT");
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0);
    ui->desiredPositionDeg_LCD->display(desiredAngleInt/10.0 * DEG_TO_RAD);
    //ROS
    pose_velocity_direction_msg.rotation = 100; //deg=step*10
    pose_velocity_direction_msg.velocity = 60; //deg/s -> ot/min
    pose_velocity_direction_msg.direction = false;
    rotate_DEC_pub.publish(pose_velocity_direction_msg);
    //ROS_INFO_STREAM(pose_velocity_direction_msg);

}
/*****************************************************/



/******************** Velocity **********************/
void MainWindow::on_MaxVelocity_input_PB_clicked(){

    //GUI
    ui->maxVelocityDeg_LCD->display((int)(ui->MaxVelocity_input_LE->text().toDouble()));
    ui->maxVelocityRad_LCD->display((int)((ui->MaxVelocity_input_LE->text().toDouble())*DEGREES_per_SECOND_TO_ROTATIONperMINUTE));
    ui->status_TE->append("Max velocity set to " + ui->MaxVelocity_input_LE->text() + " deg/s");

    pose_velocity_direction_msg.velocity = (int)((ui->MaxVelocity_input_LE->text().toDouble())*DEGREES_per_SECOND_TO_ROTATIONperMINUTE); // deg/s -> rots/min
    //ROS_INFO("rots per minute = %d",(int)(pose_velocity_direction_msg.velocity));


}
/*****************************************************/


/******************** Direction **********************/
void MainWindow::on_direction_LEFT_PB_clicked(){

    //GUI
    ui->direction_RIGHT_CB->setChecked(false);
    ui->direction_LEFT_CB->setChecked(true);
    ui->direction_LE->setText("Left");

    //ROS
    pose_velocity_direction_msg.direction = false;
    directionOfRotation = false;

}

void MainWindow::on_direction_RIGHT_PB_clicked(){

    //GUI
    ui->direction_LEFT_CB->setChecked(false);
    ui->direction_RIGHT_CB->setChecked(true);
    ui->direction_LE->setText("Right");

    //ROS
    pose_velocity_direction_msg.direction = true;
    directionOfRotation = true;
}

void MainWindow::on_direction_LEFT_CB_toggled(bool checked){

    if(checked){
        ui->direction_LE->setText("Left");
        ui->direction_RIGHT_CB->setChecked(false);
        pose_velocity_direction_msg.direction = false;
        directionOfRotation = false;
    }

}

void MainWindow::on_direction_RIGHT_CB_toggled(bool checked){

    if(checked){
        ui->direction_LE->setText("Right");
        ui->direction_LEFT_CB->setChecked(false);
        pose_velocity_direction_msg.direction = true;
        directionOfRotation = true;
    }

}
/*****************************************************/


/*********************** STOP ************************/
void MainWindow::on_stop_PB_clicked(){

    //GUI
    ui->status_TE->setTextColor(QColor("red"));
    ui->status_TE->append("Movement STOPPED!");
    ui->status_TE->setTextColor(QColor("black"));

    //ROS
    pose_velocity_direction_msg.rotation = 0;
    pose_velocity_direction_msg.velocity = 0;
    rotate_DEC_pub.publish(pose_velocity_direction_msg);


}

void MainWindow::on_centralStop_PB_clicked(){

    //GUI
    display_exit_of_program();
    pose_velocity_direction_msg.rotation = 0;
    pose_velocity_direction_msg.velocity = 0;
    rotate_DEC_pub.publish(pose_velocity_direction_msg);

    //kill ROS
    system("pkill roslaunch");
    bool_msg.data = true;
    exitProgram_pub.publish(bool_msg);

    //Kill GUI
    if (centralStopCounter == 1)
        QApplication::quit();
    else centralStopCounter++;

}
/*****************************************************/



/***************** Custom functons *******************/
void MainWindow::display_exit_of_program(){

    ui->status_TE->setTextColor(QColor("red"));
    ui->status_TE->append("***************************************");
    ui->status_TE->append("CENTRAL STOP -> EXITING PROGRAM!");
    ui->status_TE->append("***************************************");
    ui->status_TE->append("");
    ui->status_TE->append("Please click on this button once again");
    ui->status_TE->append("");


}

void MainWindow::sendWorkingMode(const int mode){

    int32_msg.data = mode;
    workingState_pub.publish(int32_msg);

}

int MainWindow::hex2dec(char hex_value[]){

    int length = std::strlen(hex_value);
    int power = 1;
    int dec_value;

    for (int i=length-1;i>=0;i--){

        if (hex_value[i]>='0' && hex_value[i]<='9'){
            dec_value += (hex_value[i] - 48) * power;
            power = power * 16;
        }else if (hex_value[i]>='A' && hex_value[i]<='F'){
            dec_value += (hex_value[i] - 55) * power;
            power = power * 16;
        }
    }

    return dec_value;
}

void MainWindow::rotateImg(double angle) {

    QPixmap pixmap("/home/viktordluhos/catkin_ws/src/rotary_table_gui/pictures/rt/rt_gif.gif");
    QMatrix rm;
    rm.rotate(angle);
    pixmap = pixmap.transformed(rm);
    ui->label_14->setPixmap(pixmap);

}

void MainWindow::displayCurrentWorkingStatus(int num1, int num2, int num3, int num4, int numberOfMessage){

    switch (num4){
        case 0xe:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" RSG Mode: CAN and Test CPU Watchdog Successful"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" CAN and CPU"));
            //ROS_INFO("RSG Mode: CAN and Test CPU Watchdog Successful");
            break;
        }
        case 0xc:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" RSG Mode: CAN and Test CPU Watchdog Successful"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" CAN and CPU"));
            //ROS_INFO("RSG Mode: CAN and Test CPU Watchdog Successful");
            break;
        }
        case 0xa:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" RSG Mode: CAN and Referencing Successful"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" CAN and ref.success"));
            //ROS_INFO("RSG Mode: CAN and Referencing Successful");
            break;
        }
        case 0x8:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" RSG Mode: CAN"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" RSG Mode: CAN"));
            //ROS_INFO("RSG Mode: CAN");
            break;
        }
        case 0x4:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Test CPU Watchdog Successful"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Test CPU Watchdog Successful"));
            //ROS_INFO("Test CPU Watchdog Successful");
            break;
        }
        case 0x2:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Referencing Successful"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Referencing Successful"));
            //ROS_INFO("Referencing Successful");
            break;
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            ROS_ERROR("Not specified 4");
            break;
        }
    }




    switch (num3){
        case 0x7:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Position reached and "
                                                                                                                               "Warning of 90% overload level before error"
                                                                                                                               "and State: Ready"));
            ui->status_workingState_TE->setText(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" pos.reach,90%overload,Ready"));
            //ROS_INFO(" Position reached and Warning of 90% overload level before error and State: Ready");
            break;
        }
        case 0x6:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Position reached and "
                                                                                                                               "Warning of 90% overload level before error"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" pos.reach,90%overload"));
            //ROS_INFO("Position reached and Warning of 90% overload level before error");
            break;
        }
        case 0x5:
        {

            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Position reached and State: Ready"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" pos.reach,Ready"));
            //ROS_INFO("Position reached and State: Ready");
            break;
        }
        case 0x4:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Position reached"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Position reached"));
            //ROS_INFO("Position reached");
            break;
        }
        case 0x3:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Warning of 90% overload level before error and"
                                                                                                                               "State: Ready"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" 90%overload,Ready"));
            //ROS_INFO("Warning of 90% overload level before error and State: Ready");
            break;
        }
        case 0x2:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Warning of 90% overload level before error"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Warning of 90% overload level before error"));
            //ROS_INFO("Warning of 90% overload level before error");
            break;
        }
        case 0x1:
        {
            ui->config_workingState_TE->setTextColor(QColor("orange"));
            ui->status_workingState_TE->setTextColor(QColor("orange"));
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Ready"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Ready"));
            //ROS_INFO("State: Ready");
            break;
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            //ROS_ERROR("Not specified 3");
            break;
        }
    }





    switch (num2){
        case 0x8:
        {
            ui->config_workingState_TE->setTextColor(QColor("red"));
            ui->status_workingState_TE->setTextColor(QColor("red"));
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Error"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Error"));
            //ROS_INFO("State: Error");
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            //ROS_ERROR("Not specified 2");
            break;
        }
    }




    switch (num1){
        case 0x8:
        {
            ui->config_workingState_TE->setTextColor(QColor("green"));
            ui->status_workingState_TE->setTextColor(QColor("green"));
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Running"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Running"));
            //ROS_INFO("State: Running");
            break;
        }
        case 0x4:
        {
            ui->config_workingState_TE->setTextColor(QColor("orange"));
            ui->status_workingState_TE->setTextColor(QColor("orange"));
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Homing"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Homing"));
            //ROS_INFO("State: Homing");
            break;
        }
        case 0x2:
        {
            ui->config_workingState_TE->setTextColor(QColor("orange"));
            ui->status_workingState_TE->setTextColor(QColor("orange"));
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Referencing"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Referencing"));
            //ROS_INFO("State: Homing");
            break;
        }
        case 0x0:
        {
            ui->config_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Start"));
            ui->status_workingState_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" State: Start"));
            //ROS_INFO("State: Homing");
            break;
        }
        default:
        {
            //ROS_ERROR("Not specified 1");
            break;
        }
    }


}

void MainWindow::displayCurrentWorkingError(int num1, int num2, int num3, int num4, int numberOfMessage){


    switch (num4){
        case 0x1:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" RSG delayed more than 1 rotation"));
            //ROS_WARN("RSG delayed more than 1 rotation");
            break;
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            //ROS_ERROR("Not specified 4 (%x)",num4);
            break;
        }
    }



    switch (num3){
        case 0x8:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Overtemperature in Brake Chopper"));
            //ROS_WARN("Overtemperature in Brake Chopper");
            break;
        }
        case 0x4:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Referencing error"));
            //ROS_WARN("Referencing error");
            break;
        }
        case 0x2:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Motor Overload Error"));
            //ROS_WARN("Motor Overload Error");
            break;
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            //ROS_WARN("Not specified 3 (%x)",num3);
            break;
        }
    }



    switch (num2){
        case 0x4:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Hall Sensors swapped / Encoder fault (reset only by reboot)"));
            //ROS_WARN("Hall Sensors swapped / Encoder fault (reset only by reboot)");
            break;
        }
        case 0x2:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" CAN Bus Error"));
            //ROS_WARN("CAN Bus Error");
            break;
        }
        case 0x1:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Hall Sensor Fault"));
            //ROS_WARN("Hall Sensor Fault");
            break;
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            //ROS_ERROR("Not specified 2  (%x)",num2);
            break;
        }
    }



    switch (num1){
        case 0x8:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Over temperature in Output Stage"));
            //ROS_WARN("Over temperature in Output Stage");
            break;
        }
        case 0x4:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Supply Voltage too Low"));
            //ROS_WARN("Supply Voltage too Low");
            break;
        }
        case 0x2:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Supply Voltage too High"));
            //ROS_WARN("Supply Voltage too High");
            break;
        }
        case 0x1:
        {
            ui->status_TE->setTextColor(QColor("red"));
            ui->status_TE->append("FATAL ERROR !! please check the status tab");
            ui->status_error_TE->append(QString("[") +QString::number(numberOfMessage) + QString("]") + QString(" Over current in Output Stage"));
            //ROS_WARN("Over current in Output Stage");
            break;
        }
        case 0x0:
        {
            break;
        }
        default:
        {
            //ROS_ERROR("Not specified 1  (%x)",num1);
            break;
        }
    }

    if (num4 == 0 && num3 == 0 && num2 == 0 && num1 == 0){
        //ui->status_error_TE->setTextColor(QColor("green"));
        //ui->status_error_TE->append(" No errors");
        //ROS_INFO("No errors");
    }


}

double MainWindow::dmod(double x, long long mod){
    return static_cast<long long>(x) % mod + x - static_cast<long long>(x);
}

int MainWindow::normalizeToRange2PI(int inputNumber){

    int modifiedAngleInt = inputNumber, k=0;

    if (modifiedAngleInt < 0){
        //ROS_INFO("Current angle less than 0 (%d)",modifiedAngleInt);
        k = -(modifiedAngleInt/3600);
        modifiedAngleInt = (k+1)*3600 + modifiedAngleInt;
        //ROS_INFO("Current angle modified to (%d)",modifiedAngleInt);
    }

    if (modifiedAngleInt >= 3600){
        //ROS_INFO("Angle is over 3600 (%d)",modifiedAngleInt);
        modifiedAngleInt = modifiedAngleInt % 3600;
        //ROS_INFO("Angle is over 3600, modif angle is %d",modifiedAngleInt);
    }else{
        //ROS_INFO("Angle OK %d",modifiedAngleInt);
    }

    return modifiedAngleInt;

}

void MainWindow::displayCurrentValues(){

    static uint8_t last_status_hexa_number1 = 255, last_status_hexa_number2 = 255, last_status_hexa_number3 = 255, last_status_hexa_number4 = 255;
    static uint8_t last_error_hexa_number1 = 255, last_error_hexa_number2 = 255, last_error_hexa_number3 = 255, last_error_hexa_number4 = 255;
    static int last_power_stage_temperature = 255, last_microprocessor_temperature = 255, last_chopper_temperature = 255, last_filtered_motor_current = 255;

    //Position
    ui->currentPositionDeg_LCD->display(currentAngleInt/10.0);
    ui->currentPositionRad_LCD->display(currentAngleInt/10.0*DEG_TO_RAD);

    //velocity
    ui->currentVelocityDeg_LCD->display(currentVelocityDeg);
    ui->currentVelocityRad_LCD->display(currentVelocityDeg*DEGREES_per_SECOND_TO_ROTATIONperMINUTE);

    //Status
    if ((status_hexa_number1 != last_status_hexa_number1) || (status_hexa_number2 != last_status_hexa_number2) || (status_hexa_number3 != last_status_hexa_number3) || (status_hexa_number4 != last_status_hexa_number4)){
        //ROS_INFO("Status change");
        displayCurrentWorkingStatus(status_hexa_number1, status_hexa_number2, status_hexa_number3, status_hexa_number4,statusNumberOfMessage);
        statusNumberOfMessage++;
        last_status_hexa_number1 = status_hexa_number1;
        last_status_hexa_number2 = status_hexa_number2;
        last_status_hexa_number3 = status_hexa_number3;
        last_status_hexa_number4 = status_hexa_number4;
    }
    //Error
    if ((error_hexa_number1 != last_error_hexa_number1) || (error_hexa_number2 != last_error_hexa_number2) || (error_hexa_number3 != last_error_hexa_number3) || (error_hexa_number4 != last_error_hexa_number4)){
        //ROS_INFO("Error change");
        displayCurrentWorkingError(error_hexa_number1, error_hexa_number2, error_hexa_number3, error_hexa_number4,errorNumberOfMessage);
        errorNumberOfMessage++;
        last_error_hexa_number1 = error_hexa_number1;
        last_error_hexa_number2 = error_hexa_number2;
        last_error_hexa_number3 = error_hexa_number3;
        last_error_hexa_number4 = error_hexa_number4;
    }

    //Temperature and Current
    if ((power_stage_temperature != last_power_stage_temperature) || (microprocessor_temperature != last_microprocessor_temperature) || (chopper_temperature != last_chopper_temperature) || (filtered_motor_current != last_filtered_motor_current)){
        //ROS_INFO("Temperature or Current change");
        ui->status_powerStageTemp_LCD->display(power_stage_temperature);
        ui->status_MicroprocesorTemp_LCD->display(microprocessor_temperature);
        ui->status_ChopperTemp_LCD->display(chopper_temperature);
        ui->filteredMotorCurrent_LCD->display(filtered_motor_current);
        last_power_stage_temperature = power_stage_temperature;
        last_microprocessor_temperature = microprocessor_temperature;
        last_chopper_temperature = chopper_temperature;
        last_filtered_motor_current = filtered_motor_current;
    }



}
/*****************************************************/



/******************* Callbacks ***********************/
void MainWindow::CurrentAngleCallback(const std_msgs::Int32 currentAngle){

    /**    Current angle is in increments. So [Deg] = [Increment] * 0.1  **/

    //ROS_INFO("Current angle callback %.1f",(double)(currentAngle.data)/10.0);
    currentAngleDeg = (double)(currentAngle.data)/10.0; //Globalna premenna v ktorej je ulozena aktualna pozicia
    currentAngleInt = normalizeToRange2PI(currentAngle.data);
    //rotateImg((double)(currentAngleInt)/10.0);
    //ui->currentPositionDeg_LCD->display(currentAngleInt/10.0);
    //ui->currentPositionRad_LCD->display(currentAngleInt/10.0*DEG_TO_RAD);

}

void MainWindow::CurrentVelocityCallback(const std_msgs::Int32 currentVelocity){

    /**  Current velocity is in 1/min  **/

    currentVelocityDeg = currentVelocity.data*ROTATIONperMINUTE_TO_DEGREES_per_SECOND;  //Globalna premenna v ktorej je ulozena aktualna rychlost
    //ROS_INFO("Current velocity callback %.1f",currentVelocityDeg);
    //ui->currentVelocityDeg_LCD->display(currentVelocityDeg);
    //ui->currentVelocityRad_LCD->display(currentVelocity.data);

}

void MainWindow::CurrentWorkingStateCallback(const std_msgs::Int32 currentWorkingState){

    //ROS_INFO("Current working state in hexa =%x",currentWorkingState.data);
    status_hexa_number1 = 0;
    status_hexa_number2 = 0;
    status_hexa_number3 = 0;
    status_hexa_number4 = 0;
    //Decompose hexa numbers
    status_hexa_number1 = (currentWorkingState.data & 0xf);
    status_hexa_number2 = (currentWorkingState.data & 0xf0)>>4;
    status_hexa_number3 = (currentWorkingState.data & 0xf00)>>8;
    status_hexa_number4 = (currentWorkingState.data & 0xf000)>>12;

}

void MainWindow::CurrentWorkingErrorCallback(const std_msgs::Int32 currentWorkingError){

    //ROS_INFO("Current working error callback %x",currentWorkingError.data);
    error_hexa_number1 = 0;
    error_hexa_number2 = 0;
    error_hexa_number3 = 0;
    error_hexa_number4 = 0;
    error_hexa_number1 = (currentWorkingError.data & 0xf);
    error_hexa_number2 = (currentWorkingError.data & 0xf0)>>4;
    error_hexa_number3 = (currentWorkingError.data & 0xf00)>>8;
    error_hexa_number4 = (currentWorkingError.data & 0xf000)>>12;

}

void MainWindow::CurrentStatusCallback(const scara_msgs::status_rt status){

    //ROS_INFO("Status msg: temp1 = %x(%d), temp2=%x(%d), temp3=%x(%d), current=%x(%d)",status.power_stage_temperature,status.power_stage_temperature,
    //status.microprocessor_temperature,status.microprocessor_temperature,status.chopper_temperature,status.chopper_temperature,
    //status.filtered_motor_current, status.filtered_motor_current);

    power_stage_temperature = status.power_stage_temperature;
    microprocessor_temperature = status.microprocessor_temperature;
    chopper_temperature = status.chopper_temperature;
    filtered_motor_current = status.filtered_motor_current;

}   /** Bude treba preratat hodnotu  statusu na realne jednotky stupne a mA **/
/*****************************************************/

