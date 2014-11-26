#include "mainwindow.h"
#include <QApplication>
#include "typedef.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    char *file_path = "";
    if (argc == 2)
        file_path = argv[1];
    MainWindow w;
    w.initFilePath = file_path;
    w.show();
    return a.exec();
}
