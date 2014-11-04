#include "mainwindow.h"
#include <QApplication>
#include "stdafx.h"

char *file_path = "";
MainWindow *w_ptr = 0;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    file_path = "";
    if (argc == 2)
        file_path = argv[1];
    MainWindow w;
    w_ptr = &w;
    w.show();
    return a.exec();
}
