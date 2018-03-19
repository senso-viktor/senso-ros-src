#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


   // timer = new QTimer();
    //connect(timer, SIGNAL(timeout()), this, SLOT(displayCurrentValues()));
    //timer->start(1000);


}

MainWindow::~MainWindow()
{
    delete ui;
}

/****************************************/

void MainWindow::on_config_OFF_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state OFF - Check working state");

}

void MainWindow::on_config_READY_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state READY - Check working state");

}

void MainWindow::on_config_ON_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state ON - Check working state");
    QPixmap pixmap("/home/viktor/catkin_ws/src/rotary_table_gui/pictures/Sensodrive-LogoE.gif");
    QMatrix rm;
    rm.rotate(-90);
    pixmap = pixmap.transformed(rm);
    ui->label_2->setPixmap(pixmap);

}

void MainWindow::on_config_ERROR_PB_clicked(){

    ui->status_TE->setTextColor(QColor("orange"));
    ui->status_TE->append("trying to get to state ERROR - Check working state");
    QPixmap pixmap("/home/viktor/catkin_ws/src/rotary_table_gui/pictures/Sensodrive-LogoE.gif");
    QMatrix rm;
    rm.rotate(90);
    pixmap = pixmap.transformed(rm);
    ui->label_2->setPixmap(pixmap);

}

/*********************************************/


void MainWindow::on_relativeControl_slider_SLIDER_actionTriggered(int action){

    //GUI
    ui->relativeControl_slider_LE->setText(QString::number(ui->relativeControl_slider_SLIDER->value()) + " deg");

}

void MainWindow::on_relativeControl_slider_PB_clicked(){

    //GUI
   if (directionOfRotation){
        ui->relativeControl_slider_LE->setText(QString::number(ui->relativeControl_slider_SLIDER->value()) + " deg");
        ui->desiredPositionDeg_LCD->display(ui->relativeControl_slider_SLIDER->value() + currentAngleDeg);
        ui->desiredPositionRad_LCD->display((ui->relativeControl_slider_SLIDER->value() + currentAngleDeg) * DEG_TO_RAD);
        currentAngleDeg+=ui->relativeControl_slider_SLIDER->value();
    }else{
        ui->relativeControl_slider_LE->setText(QString::number(-ui->relativeControl_slider_SLIDER->value()) + " deg");
        ui->desiredPositionDeg_LCD->display(-ui->relativeControl_slider_SLIDER->value() + currentAngleDeg);
        ui->desiredPositionRad_LCD->display((-ui->relativeControl_slider_SLIDER->value() + currentAngleDeg) * DEG_TO_RAD);
        currentAngleDeg-=ui->relativeControl_slider_SLIDER->value();
    }
   ui->status_TE->append("Moving[relative] to " + QString::number(ui->relativeControl_slider_SLIDER->value()) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));


}

void MainWindow::on_relativeControl_input_PB_clicked(){

    //GUI
   if (directionOfRotation){
        ui->relativeControl_slider_LE->setText(QString::number(ui->relativeControl_input_LE->text().toDouble()) + " deg");
        ui->desiredPositionDeg_LCD->display(ui->relativeControl_input_LE->text().toDouble() + currentAngleDeg);
        ui->desiredPositionRad_LCD->display((ui->absoluteControl_input_LE->text().toDouble() + currentAngleDeg) * DEG_TO_RAD);
        currentAngleDeg+=ui->relativeControl_input_LE->text().toDouble();
    }else{
        ui->relativeControl_slider_LE->setText(QString::number(-ui->relativeControl_input_LE->text().toDouble()) + " deg");
        ui->desiredPositionDeg_LCD->display(-ui->relativeControl_input_LE->text().toDouble() + currentAngleDeg);
        ui->desiredPositionRad_LCD->display((-ui->absoluteControl_input_LE->text().toDouble() + currentAngleDeg) * DEG_TO_RAD);
        currentAngleDeg-=ui->relativeControl_input_LE->text().toDouble();
    }
    ui->status_TE->append("Moving[relative] to " + QString::number(ui->relativeControl_input_LE->text().toDouble()) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));

}

void MainWindow::on_absoluteControl_slider_SLIDER_actionTriggered(int action){

    //GUI
    ui->absoluteControl_slider_LE->setText(QString::number(ui->absoluteControl_slider_SLIDER->value()) + " deg");

}

void MainWindow::on_absoluteControl_slider_PB_clicked(){

    //GUI
    ui->absoluteControl_slider_LE->setText(QString::number(ui->absoluteControl_slider_SLIDER->value()) + " deg");
    ui->desiredPositionDeg_LCD->display(ui->absoluteControl_slider_SLIDER->value());
    ui->desiredPositionRad_LCD->display(ui->absoluteControl_slider_SLIDER->value()*DEG_TO_RAD);
    ui->status_TE->append("Moving[absolute] to " + QString::number(ui->absoluteControl_slider_SLIDER->value()) + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));



}

void MainWindow::on_absoluteControl_input_PB_clicked(){

    //GUI
    ui->absoluteControl_slider_LE->setText(QString::number(ui->absoluteControl_input_LE->text().toDouble()) + " deg");
    ui->desiredPositionDeg_LCD->display(ui->absoluteControl_input_LE->text().toDouble());
    ui->desiredPositionRad_LCD->display(ui->absoluteControl_input_LE->text().toDouble()*DEG_TO_RAD);
    ui->status_TE->append("Moving[absolute] to " + ui->absoluteControl_input_LE->text() + " deg in direction " + QString(directionOfRotation ? "RIGHT":"LEFT"));

}

void MainWindow::on_MaxVelocity_input_PB_clicked(){

    //GUI
    ui->maxVelocityDeg_LCD->display(ui->MaxVelocity_input_LE->text().toDouble());
    ui->maxVelocityRad_LCD->display(ui->MaxVelocity_input_LE->text().toDouble()*DEG_TO_RAD);
    ui->status_TE->append("Max velocity set to " + ui->MaxVelocity_input_LE->text() + " deg/s");


}

void MainWindow::on_direction_LEFT_PB_clicked(){

    //GUI
    ui->direction_RIGHT_CB->setChecked(false);
    ui->direction_LEFT_CB->setChecked(true);
    ui->direction_LE->setText("Left");

    //ROS
    directionOfRotation = false;

}

void MainWindow::on_direction_RIGHT_PB_clicked(){

    //GUI
    ui->direction_LEFT_CB->setChecked(false);
    ui->direction_RIGHT_CB->setChecked(true);
    ui->direction_LE->setText("Right");

    //ROS
    directionOfRotation = true;
}

void MainWindow::on_direction_LEFT_CB_toggled(bool checked){

    if(checked){
        ui->direction_LE->setText("Left");
        ui->direction_RIGHT_CB->setChecked(false);
        directionOfRotation = false;
    }

}

void MainWindow::on_direction_RIGHT_CB_toggled(bool checked)
{
    if(checked){
        ui->direction_LE->setText("Right");
        ui->direction_LEFT_CB->setChecked(false);
        directionOfRotation = true;
    }

}

/*********************************************/



void MainWindow::on_stop_PB_clicked(){

    //GUI
    ui->status_TE->setTextColor(QColor("red"));
    ui->status_TE->append("Movement STOPPED!");
    ui->status_TE->setTextColor(QColor("black"));

    //ROS

}

void MainWindow::on_centralStop_PB_clicked(){

    //GUI
    ui->status_TE->setTextColor(QColor("red"));
    ui->status_TE->append("***************************************");
    ui->status_TE->append("CENTRAL STOP -> EXITING PROGRAM!");
    ui->status_TE->append("***************************************");


    //kill roslaunch


    //Kill GUI
    //sleep(3);
    //QApplication::quit();

}



void MainWindow::on_smooth_plusHalf_PB_clicked()
{

}

void MainWindow::on_smooth_plusOne_PB_clicked()
{

}

void MainWindow::on_smooth_minusHalf_PB_clicked()
{

}

void MainWindow::on_smooth_minusOne_PB_clicked()
{

}

void MainWindow::displayCurrentValues(QTimerEvent *event){

   // ROS_WARN("Timer start");
    qDebug() << "timer function";
    ui->status_TE->append("timer!");

}




void MainWindow::on_pushButton_clicked()
{

}
