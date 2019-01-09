#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    MainWindow w;
    w.resize(1305, 840);
    w.setMinimumSize(1305, 840);
    w.move(20, 20);
    w.show();

    return a.exec();
}
