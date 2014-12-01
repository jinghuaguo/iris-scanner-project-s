#include "mainwindow.h"
#include <QApplication>
#include "typedef.h"

char *initFilePath = "";

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    if (argc == 2)
        initFilePath = argv[1];
    MainWindow *w(new MainWindow());
    w->show();
    return a.exec();
}
