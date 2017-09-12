#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Ui::MainWindow *ui;
    void hovno();

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


    void on_status_joint1vel_3_overflow();

    void on_positionControl_Gripper_Checkbox_4_toggled(bool checked);

    void on_teachMode_teachButton_clicked();

    void on_teachMode_stopTeachButton_clicked();

    void on_teachMode_tabWidget_tabBarClicked(int index);

    void on_teachModeRun_start_pushbutton_clicked();

    void on_teachModeRun_stop_pushbutton_clicked();

    void on_teachMode_teachButton_4_clicked();

    void on_teachMode_stopTeachButton_4_clicked();

    void on_teachModeRun_start_pushbutton_4_clicked();

    void on_teachModeRun_stop_pushbutton_4_clicked();

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

    void on_checkBox_toggled(bool checked);

    void on_moveit_gripper_checkBox_toggled(bool checked);

    void on_colisionObject_CustomObj_posChangeenterpushButton_2_clicked();

    void on_colisionObject_CustomObj_SizeenterpushButton_clicked();

    void on_colisionObject_RealObj_SizeenterpushButton_2_clicked();

    void on_setParameters_Precision_PushButton_4_clicked();

    void on_teachModeRun_collision_checkbox_toggled(bool checked);

    void on_teachModeRun_collisiongui_checkbox_clicked();

    void on_positionControl2_collision_checkbox_toggled(bool checked);

    void on_teachModeRun_collisiongui_checkbox_toggled(bool checked);

    void on_positionControlCustom_collision_checkbox_toggled(bool checked);

    void on_jointControl_collision_checkbox_toggled(bool checked);

private:
    //Ui::MainWindow *ui;
    bool jointControl_gripperState = false;
};

#endif // MAINWINDOW_H
