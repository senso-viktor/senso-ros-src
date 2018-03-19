#include "ui_mainwindow.h"
#include "QWidget"
#include "rotary_table_gui/mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[]) {

    QApplication a(argc, argv);
    MainWindow *qMain = new MainWindow();
    qMain->show();

    return a.exec();
}
