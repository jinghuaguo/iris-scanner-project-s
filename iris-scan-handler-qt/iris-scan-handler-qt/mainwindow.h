#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "stdafx.h"
#include "viewhandler.h"
#include "project.h"
#include "registration.h"

#ifndef Q_MOC_RUN

#include "utils.h"
#include "icp.h"
#include "grayscaleregistration.h"
#include "surface.h"

#endif

#include <QMainWindow>
#include <QLabel>
#include <QListWidgetItem>

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void refreshSelected(QList<QListWidgetItem*> selectedItems, QListWidget *object);
    ViewHandler *vh;
    pcl::visualization::PCLVisualizer *viewer;
    Project *cPrj;
    void addExternalClouds(QStringList paths);
    void refreshLists(bool updateViewer);
    void addSphereforKeyPoint(int i, int j);

    //Workers
    ICP *icp;
    GrayScaleRegistration *gsr;
    Surface *surface;
    Registration *reg;

public slots:
    void refresh();
    void refreshViewerContents();
    void handlePickedPoint(int index);
    void handlePickedPoint(float x, float y, float z);
    void updateStatus(QString status = "Ready");

private:
    Ui::MainWindow *ui;
    QLabel *lblStatus;
    void closeEvent(QCloseEvent *event);
    bool stayOpen();

private slots:
    void showAboutApp();
    void handleCoordinateSystem();
    void resetCameraPara();
    void setViewerBackgroundColor();
    void newProject();
    void manageNewProject();
    void manageOpenProject();
    void openProject(QString path);
    void addSelToProc();
    void on_lstPointClouds_itemSelectionChanged();
    void importFile();
    void importFolder();
    void clearAllProcessed();
    void on_lstProcessList_itemDoubleClicked(QListWidgetItem *item);
    void on_lstProcessList_itemSelectionChanged();
    void removeCurrentSelectedClouds();
    void doGrayScaleReg();
    void doDownSample();
    void doICPReg();
    void doKeyPointReg();
    void doCombine();
    void doMLS();
    void doGP3();
    void createCopy();
    void doApplyTransformation();
    void saveNotSavedPointCloud();
    void saveNotSavedPointCloud(int index);
    void saveProject();
    void saveProjectAs();
    void on_chkShowFusedProcList_clicked();
    void on_btnProcToTop_clicked();
    void on_lstPointClouds_itemDoubleClicked(QListWidgetItem *item);
    void on_lstCorrespondences_itemDoubleClicked(QListWidgetItem *item);
    void openGrabber();
    void removeRecentKeyPoint();
    void removeAllKeyPoints();
    void removeAttachMesh();
    void showSettings();
    void onReturnRead();
};

#endif // MAINWINDOW_H
