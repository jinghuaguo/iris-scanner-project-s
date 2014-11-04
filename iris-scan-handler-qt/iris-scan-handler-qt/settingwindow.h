#ifndef SETTINGWINDOW_H
#define SETTINGWINDOW_H

#include <QDialog>
#include "QAbstractButton"

#include "mainwindow.h"

#ifndef Q_MOC_RUN

#include "icp.h"
#include "grayscaleregistration.h"
#include "surface.h"
#include "registration.h"

#endif

namespace Ui {
class SettingWindow;
}

class SettingWindow : public QDialog
{
    Q_OBJECT

public:
    explicit SettingWindow(QWidget *parent = 0);
    ~SettingWindow();
    void resetToOriginal();
    void restoreToDefaults();
    void applySettings();

private slots:
    void on_buttonBox_clicked(QAbstractButton *button);

private:
    Ui::SettingWindow *ui;
    ICP *icp;
    GrayScaleRegistration *gsr;
    Surface *surface;
    Registration *reg;
};

#endif // SETTINGWINDOW_H
