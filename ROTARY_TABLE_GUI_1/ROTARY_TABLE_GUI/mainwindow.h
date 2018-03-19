#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>


const double RAD_TO_DEG = 57.2957795130;
const double DEG_TO_RAD = 0.0174532925;

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

    //void displayCurrentValues();

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    int timerId;

    bool directionOfRotation = true;                //directionOfRotation=true ->right     directionOfRotation=false->left
    double currentAngleDeg = 0.0, currentAngleRad = 0.0;
protected:
    void displayCurrentValues(QTimerEvent *event);
};

#endif // MAINWINDOW_H
