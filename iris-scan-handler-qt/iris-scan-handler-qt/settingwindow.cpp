#include "settingwindow.h"
#include "ui_settingwindow.h"
#include "QDialogButtonBox"
#include "QMessageBox"

#include "mainwindow.h"

SettingWindow::SettingWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingWindow)
{
    ui->setupUi(this);

    //load parameters.
    icp = mainWindowPtr->icp;
    gsr = mainWindowPtr->gsr;
    surface = mainWindowPtr->surface;
    reg = mainWindowPtr->reg;

    resetToOriginal();
}

SettingWindow::~SettingWindow()
{
    delete ui;
}

void SettingWindow::resetToOriginal()
{
    this->ui->gsrMaxRemoveDistanceThresholdValue->setValue(gsr->_GSRMaxRemoveDistanceThreshold);
    this->ui->gsrSurfThresholdValue->setValue(gsr->_GSRSurfThreshold);
    this->ui->icpDownSampleLeafSizeValue->setValue(icp->_ICPDownSampleLeafSize);
    this->ui->icpIterationTimesPerPeriodValue->setValue(icp->_ICPIterationTimesPer);
    this->ui->icpIterationTimesValue->setValue(icp->_ICPIterationTimes);
    this->ui->icpMaxCorrespondenceDistanceValue->setValue(icp->_ICPMaxCorrespondenceDistance);
    this->ui->icpTransformEpilsonValue->setValue(icp->_ICPTransformEpsilon);
    this->ui->icpKSearchValue->setValue(icp->_GetNormalKSearch);
    this->ui->gp3MuValue->setValue(surface->_gp3Mu);
    this->ui->gp3NearestNeighborsValue->setValue(surface->_gp3NearestNeighbors);
    this->ui->gp3RadiusValue->setValue(surface->_gp3Radius);
    this->ui->mlsRadiusValue->setValue(surface->_mlsRadius);
    this->ui->estimationMethodCombo->setCurrentIndex(reg->estimationMethod);
}

void SettingWindow::applySettings()
{
    gsr->_GSRMaxRemoveDistanceThreshold = this->ui->gsrMaxRemoveDistanceThresholdValue->value();
    gsr->_GSRSurfThreshold = this->ui->gsrSurfThresholdValue->value();
    icp->_ICPDownSampleLeafSize = this->ui->icpDownSampleLeafSizeValue->value();
    icp->_ICPIterationTimesPer = this->ui->icpIterationTimesPerPeriodValue->value();
    icp->_ICPIterationTimes = this->ui->icpIterationTimesValue->value();
    icp->_ICPMaxCorrespondenceDistance = this->ui->icpMaxCorrespondenceDistanceValue->value();
    icp->_ICPTransformEpsilon = this->ui->icpTransformEpilsonValue->value();
    icp->_GetNormalKSearch = this->ui->icpKSearchValue->value();
    surface->_gp3Mu = this->ui->gp3MuValue->value();
    surface->_gp3NearestNeighbors = this->ui->gp3NearestNeighborsValue->value();
    surface->_gp3Radius = this->ui->gp3RadiusValue->value();
    surface->_mlsRadius = this->ui->mlsRadiusValue->value();
    reg->estimationMethod = this->ui->estimationMethodCombo->currentIndex();
}

void SettingWindow::restoreToDefaults()
{
    this->ui->gsrMaxRemoveDistanceThresholdValue->setValue(4);
    this->ui->gsrSurfThresholdValue->setValue(750);
    this->ui->icpDownSampleLeafSizeValue->setValue(0.05);
    this->ui->icpIterationTimesPerPeriodValue->setValue(20);
    this->ui->icpIterationTimesValue->setValue(50);
    this->ui->icpMaxCorrespondenceDistanceValue->setValue(0.5);
    this->ui->icpTransformEpilsonValue->setValue(0.1);
    this->ui->icpKSearchValue->setValue(15);
    this->ui->gp3MuValue->setValue(4);
    this->ui->gp3NearestNeighborsValue->setValue(100);
    this->ui->gp3RadiusValue->setValue(0.15);
    this->ui->mlsRadiusValue->setValue(0.15);
    this->ui->estimationMethodCombo->setCurrentIndex(0);
}

void SettingWindow::on_buttonBox_clicked(QAbstractButton *button)
{
    QMessageBox::StandardButton reply;

    QDialogButtonBox::StandardButton standardButton = ui->buttonBox->standardButton(button);
    switch(standardButton)
    {
    case QDialogButtonBox::Ok:
        applySettings();
        break;
    case QDialogButtonBox::Apply:
        applySettings();
        break;
    case QDialogButtonBox::Cancel:

        break;
    case QDialogButtonBox::Reset:
        reply = QMessageBox::question(this, tr("Reset all settings"), tr("Are you sure to reset all settings to original?"), QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
            resetToOriginal();
        break;
    case QDialogButtonBox::RestoreDefaults:
        reply = QMessageBox::question(this, tr("Restore all settings"), tr("Are you sure to restore all settings to defaults?"), QMessageBox::Yes | QMessageBox::No);
        if (reply == QMessageBox::Yes)
            restoreToDefaults();
        break;
    default:

        break;
    }
}
