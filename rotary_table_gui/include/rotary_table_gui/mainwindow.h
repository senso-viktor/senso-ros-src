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
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/String.h"
#include "scara_msgs/pose_velocity_direction.h"
#include "scara_msgs/status_rt.h"
#include <stdlib.h>

const double RAD_TO_DEG = 57.2957795130;
const double DEG_TO_RAD = 0.0174532925;
const double ROTATIONperMINUTE_TO_DEGREES_per_SECOND = ((2*M_PI) / 60) * RAD_TO_DEG;
const double DEGREES_per_SECOND_TO_ROTATIONperMINUTE = (60/(2*M_PI)) * DEG_TO_RAD;

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

    //Callbacks
    void CurrentAngleCallback(const std_msgs::Int32 currentAngle);

    void CurrentVelocityCallback(const std_msgs::Int32 currentVelocity);

    void CurrentWorkingStateCallback(const std_msgs::Int32 currentWorkingState);

    void CurrentWorkingErrorCallback(const std_msgs::Int32 currentWorkingError);

    void CurrentStatusCallback(const scara_msgs::status_rt status);  //Zmenit typ spravy na svoj preddefinovany

private slots:
    void on_config_OFF_PB_clicked();

    void on_config_READY_PB_clicked();

    void on_config_ON_PB_clicked();

    void on_config_ERROR_PB_clicked();

    void on_relativeControl_slider_SLIDER_actionTriggered(int action);

    void on_relativeControl_slider_PB_clicked();

    void on_relativeControl_input_PB_clicked();

    void on_absoluteControl_slider_SLIDER_actionTriggered(int action);

    void on_absoluteControl_slider_PB_clicked();

    void on_absoluteControl_input_PB_clicked();

    void on_MaxVelocity_input_PB_clicked();

    void on_direction_LEFT_PB_clicked();

    void on_direction_RIGHT_PB_clicked();

    void on_stop_PB_clicked();

    void on_centralStop_PB_clicked();

    void on_direction_LEFT_CB_toggled(bool checked);

    void on_direction_RIGHT_CB_toggled(bool checked);

    void on_smooth_plusHalf_PB_clicked();

    void on_smooth_plusOne_PB_clicked();

    void on_smooth_minusHalf_PB_clicked();

    void on_smooth_minusOne_PB_clicked();

    void on_pushButton_clicked();

    //Custom functions
    void display_exit_of_program();

    void sendWorkingMode(const int modeSelect);

    int hex2dec(char hex_value[]);

    void rotateImg(double angle);

    void displayCurrentWorkingStatus(int num1, int num2, int num3, int num4, int numberOfMessage);

    void displayCurrentWorkingError(int num1, int num2, int num3, int num4, int numberOfMessage);

    double dmod(double x, long long mod);

    int normalizeToRange2PI(int inputNumber);

    void displayCurrentValues();

private:
    Ui::MainWindow *ui;

    ros::AsyncSpinner *aspinner;

    bool directionOfRotation = true;                //directionOfRotation=true ->right     directionOfRotation=false->left
    uint8_t status_hexa_number1 = 0, status_hexa_number2 = 0, status_hexa_number3 = 0, status_hexa_number4 = 0;
    uint8_t error_hexa_number1 = 0, error_hexa_number2 = 0, error_hexa_number3 = 0, error_hexa_number4 = 0;
    int centralStopCounter = 0, currentAngleInt = 0, desiredAngleInt = 0,  power_stage_temperature = 0, microprocessor_temperature = 0, chopper_temperature = 0, filtered_motor_current = 0;
    int statusNumberOfMessage = 1, errorNumberOfMessage = 1, currentAngleHelp = 0;
    double currentAngleDeg = 0.0, currentVelocityDeg = 0.0;
    char hexString[16];

    std_msgs::Bool bool_msg;
    std_msgs::Int32 int32_msg;
    std_msgs::UInt8MultiArray uInt8MultiArray_msg;
    std_msgs::String string_msg;
    scara_msgs::pose_velocity_direction pose_velocity_direction_msg;
    scara_msgs::status_rt status_msg;

    ros::Publisher rotate_DEC_pub, rotate_HEX_pub,  temperatureAndCurrent_pub, workingState_pub, exitProgram_pub;
    ros::Subscriber currentAngleDeg_sub, currentVelocityPerMinute_sub, currentState_sub, currentError_sub, status_sub, useless_sub;
};

#endif // MAINWINDOW_H
