#include "typedef.h"
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "settingwindow.h"

#include "QMessageBox"
#include "QInputDialog"
#include "QColorDialog"
#include "QStringBuilder"
#include "QTextStream"
#include "QFileDialog"
#include "QFile"
#include "QDir"
#include "QFileInfo"
#include "QFileInfoList"
#include "QList"
#include "QCloseEvent"
#include "QProcess"

MainWindow *mainWindowPtr = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    icp(new ICP()),
    gsr(new GrayScaleRegistration()),
    surface(new Surface()),
    reg(new Registration())
{
    mainWindowPtr = this;

    ui->setupUi(this);
    //this->setWindowState(Qt::WindowState::WindowMaximized);
    this->cPrj = 0;

    //lblStatus
    lblStatus = new QLabel(this);
    lblStatus->setText("Ready.");
    ui->statusBar->addWidget(lblStatus, 1);

    //connects
    connect(ui->actionAbout_Qt, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
    connect(ui->action_Exit, SIGNAL(triggered()), this, SLOT(close()));
    connect(ui->actionAbout_IRIS_Scan_Handler, SIGNAL(triggered()), this, SLOT(showAboutApp()));
    connect(ui->action_CoordinateSystem, SIGNAL(triggered()), this, SLOT(handleCoordinateSystem()));
    connect(ui->action_Reset_Camera_Parameter, SIGNAL(triggered()), this, SLOT(resetCameraPara()));
    connect(ui->actionSet_Background_Color, SIGNAL(triggered()), this, SLOT(setViewerBackgroundColor()));
    connect(ui->action_File, SIGNAL(triggered()), this, SLOT(importFile()));
    connect(ui->action_Refresh, SIGNAL(triggered()), this, SLOT(refresh()));
    connect(ui->action_Folder, SIGNAL(triggered()), this, SLOT(importFolder()));
    connect(ui->action_Remove_Current_Clouds, SIGNAL(triggered()), this, SLOT(removeCurrentSelectedClouds()));
    connect(ui->action_Clear_Process_List, SIGNAL(triggered()), this, SLOT(clearAllProcessed()));
    connect(ui->actionRegister_by_Gray_Scale_Features, SIGNAL(triggered()), this, SLOT(doGrayScaleReg()));
    connect(ui->action_Downsample, SIGNAL(triggered()), this, SLOT(doDownSample()));
    connect(ui->actionApply_Transformation_with, SIGNAL(triggered()), this, SLOT(doApplyTransformation()));
    connect(ui->actionRegister_by_ICP, SIGNAL(triggered()), this, SLOT(doICPReg()));
    connect(ui->action_Add_Selected_Cloud_s_to_Process_List, SIGNAL(triggered()), this, SLOT(addSelToProc()));
    connect(ui->actionSave_Current_Project, SIGNAL(triggered()), this, SLOT(saveProject()));
    connect(ui->actionSave_Current_Project_As, SIGNAL(triggered()), this, SLOT(saveProjectAs()));
    connect(ui->actionOpen_Project, SIGNAL(triggered()), this, SLOT(manageOpenProject()));
    connect(ui->actionNew, SIGNAL(triggered()), this, SLOT(manageNewProject()));
    connect(ui->actionRemove_the_Recent_Key_Point, SIGNAL(triggered()), this, SLOT(removeRecentKeyPoint()));
    connect(ui->actionRemove_All_K_ey_Point, SIGNAL(triggered()), this, SLOT(removeAllKeyPoints()));
    connect(ui->actionRegister_by_Manual_Keypoint, SIGNAL(triggered()), this, SLOT(doKeyPointReg()));
    connect(ui->action_Export_Selected_Point_Cloud_s, SIGNAL(triggered()), this, SLOT(saveNotSavedPointCloud()));
    connect(ui->actionC_reate_Copy, SIGNAL(triggered()), this, SLOT(createCopy()));
    connect(ui->action_Combine, SIGNAL(triggered()), this, SLOT(doCombine()));
    connect(ui->actionSe_ttings, SIGNAL(triggered()), this, SLOT(showSettings()));
    connect(ui->actionMoving_Least_Squares, SIGNAL(triggered()), this, SLOT(doMLS()));
    connect(ui->actionGreedy_Projection_Triangulation, SIGNAL(triggered()), this, SLOT(doGP3()));
    connect(ui->actionRemove_the_Mesh_s_in_Selected_Cloud_s, SIGNAL(triggered()), this, SLOT(removeAttachMesh()));
    connect(ui->action_Start_capturing, SIGNAL(triggered()), this, SLOT(openGrabber()));

    //Viewer
    ViewHandler *vh(new ViewHandler(this));
    this->setCentralWidget(vh);
    this->viewer = &(vh->viewer);
    this->vh = vh;
    this->ui->tabDataList->setCurrentIndex(0);

    //Layout
    ui->dockInfoWidget->setContentsMargins(1, 1, 1, 1);
    ui->dockInfoWidget->layout()->setContentsMargins(1, 1, 1, 1);
    ui->tabData1->setContentsMargins(0, 0, 0, 0);
    ui->tabData1->layout()->setContentsMargins(0, 0, 0, 0);
    ui->tabData2->setContentsMargins(0, 0, 0, 0);
    ui->tabData2->layout()->setContentsMargins(0, 0, 0, 0);
    ui->dockInfo->hide();

    //Project
    std::cout << initFilePath << std::endl;

    if (initFilePath[0] == 0)
        this->newProject();
    else
        this->openProject(QString::fromAscii(initFilePath));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showAboutApp()
{
    QMessageBox::information(this, "About IRIS Scan Handler", "IRIS Scan Handler\nOne component of IRIS Scanner Project\n\n(c) 2014 IRIS Study & Research Group, Wuhan Univ. \nAuthor : Yue Guan (yue.guan@outlook.com)\n\nFunctions on capturing, filtering, registrating, surfacing, etc. are based on PCL (Point Cloud Library, http://www.pointcloud.org). \n\nSome functions on registrating are based on OpenCV (http://www.opencv.org). \n\nThis program is under GNU GPL lisence. (http://www.gnu.org/copyleft/gpl.html)", QMessageBox::Ok);
}

void MainWindow::handleCoordinateSystem()
{
    if (vh->showCoordinateSystem == false)
    {
        viewer->addCoordinateSystem();
    }
    else
    {
        viewer->removeCoordinateSystem();
    }
    vh->showCoordinateSystem = !(vh->showCoordinateSystem);
    vh->update();
    updateStatus("Coordinate System Adjusted Successfully.");
}

void MainWindow::resetCameraPara()
{
    viewer->setCameraPosition(-1, 1, -10, 0, -1, 0);
    vh->update();
    updateStatus("Reset Camera Parameters Successfully.");

}

void MainWindow::setViewerBackgroundColor()
{
    QColor color = QColorDialog::getColor(Qt::black, this);
    if (color.isValid())
    {
        viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    }
    updateStatus("Set Viewer Background Successfully.");

}

void MainWindow::newProject()
{
    Project* _newProject(new Project("New Project"));
    this->cPrj = _newProject;
    this->setWindowTitle("IRIS Scan Handler - " + this->cPrj->name);
    refresh();
    updateStatus("New Project Created Successfully.");
}

void MainWindow::manageNewProject()
{
    if (cPrj == NULL || cPrj->isSaved || !stayOpen())
    {
        this->newProject();
    }
}

void MainWindow::manageOpenProject()
{
    if (cPrj == NULL || cPrj->isSaved || !stayOpen())
    {
        QString path = QFileDialog::getOpenFileName(this, "Open Project File", "", tr("Project (*.isproj)"));
        if (path != "")
            this->openProject(path);
    }
}

void MainWindow::openProject(QString path)
{
    updateStatus("Opening " + path + " ...");

    Project *newPrj(new Project(path));
    QFile file(path);

    if (!file.open(QIODevice::ReadWrite))
    {
        updateStatus("Cannot open file : " + file.errorString());
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
        if (newPrj->readMark(in.readLine()) != 0)
        {
            updateStatus("An error occoured while loading the project.");
            refresh();
            return;
        }
    }

    delete this->cPrj;
    cPrj = newPrj;

    this->setWindowTitle("IRIS Scan Handler - " + cPrj->name);
    updateStatus(path + " Open Successfully.");
    cPrj->isSaved = true;
    refresh();
}

void MainWindow::addSelToProc()
{
    int sum = 0;
    for (int i = 0; i < cPrj->selClouds.size(); i++)
    {
        if (cPrj->procClouds.contains(cPrj->selClouds[i]))
        {
            updateStatus("Cloud #" + QString::number(cPrj->selClouds[i] + 1) + " is already existed in process list.");
            continue;
        }
        cPrj->procClouds.push_back(cPrj->selClouds[i]);
        sum++;
    }
    refresh();
    cPrj->isSaved = false;
    updateStatus(QString::number(sum) + " cloud(s) have been added to process list.");
}

void MainWindow::on_lstPointClouds_itemSelectionChanged()
{
    refreshSelected(ui->lstPointClouds->selectedItems(), ui->lstPointClouds);
    if (!vh->showFusedProcessList)
        refreshViewerContents();
    updateStatus(QString::number(cPrj->selClouds.size()) + " cloud(s) have been selected.");
}

void MainWindow::refreshSelected(QList<QListWidgetItem *> selectedItems, QListWidget *object)
{
    cPrj->selClouds.clear();
    if (selectedItems.size() == 0) return;
    int i = 0, j = 0;
    for (i = 0; i < object->count(); i++)
        for (j = 0; j < selectedItems.size(); j++)
            if (selectedItems[j] == object->item(i))
            {
                cPrj->selClouds.push_back(i);
            }
}

void MainWindow::addExternalClouds(QStringList paths)
{
    vh->showFusedProcessList = false;
    for (auto i = paths.begin(); i != paths.end(); i++)
    {
        CloudPtr p(new Cloud());
        QString name = *i;
        this->updateStatus("Reading point cloud " + name + " ...");
        Utils::readSinglePointCloud(name.toStdString(), p);
        cPrj->addCloud(p, name);
        cPrj->cIsSaved[cPrj->getCloudSize() - 1] = true;
    }
    cPrj->isSaved = false;
    refresh();
    updateStatus(QString::number(paths.size()) + " cloud(s) have been imported.");
}

void MainWindow::importFile()
{
    QStringList paths = QFileDialog::getOpenFileNames(this, tr("Import Point Cloud"), "", tr("PCL Format (*.pcd);;Stanford PLY Format (*.ply)"));
    addExternalClouds(paths);
}

void MainWindow::importFolder()
{
    QString path = QFileDialog::getExistingDirectory(this, tr("Import Point Cloud"), "");
    QDir dir(path);
    if (!dir.exists()) return;
    dir.setFilter(QDir::Files | QDir::NoSymLinks);
    QFileInfoList list = dir.entryInfoList();
    int file_count = list.count();
    if (file_count <= 0) return;

    QStringList string_list;
    for (int i = 0; i < file_count; i++)
    {
        QFileInfo file_info = list.at(i);
        QString suffix = file_info.suffix();
        if (QString::compare(suffix, QString("pcd"), Qt::CaseInsensitive) == 0 || QString::compare(suffix, QString("ply"), Qt::CaseInsensitive) == 0)
        {
            QString absolute_file_path = file_info.absoluteFilePath();
            string_list.append(absolute_file_path);
        }
    }
    addExternalClouds(string_list);
}

void MainWindow::refreshLists(bool updateViewer = true)
{
    QString strBuf;
    ui->lstPointClouds->clear();
    for (int i = 0; i < cPrj->getCloudSize(); i++)
    {
        strBuf = "Cloud #";
        strBuf += QString::number(i + 1);
        strBuf += cPrj->cIsSaved[i] ? " " : "*";
        strBuf += " " + cPrj->cPath[i];
        strBuf += (cPrj->cAttachMesh[i] == -1) ? " " : " [Mesh]";
        ui->lstPointClouds->addItem(strBuf);
    }

    ui->lstCorrespondences->clear();
    for (int i = 0; i < cPrj->getCorrespondenceSize(); i++)
    {
        strBuf = "Corr. #";
        strBuf += QString::number(i + 1);
        strBuf += " Cloud #" + QString::number(cPrj->rSource[i] + 1);
        strBuf += " -> Cloud #" + QString::number(cPrj->rTarget[i] + 1);
        strBuf += cPrj->rIsTopo[i] ? " [TOPO]" : "";
        ui->lstCorrespondences->addItem(strBuf);
    }

    ui->lstProcessList->clear();
    for (int i = 0; i < cPrj->procClouds.size(); i++)
    {
        strBuf = "No.";
        strBuf += QString::number(i + 1);
        strBuf += " Cloud #";
        strBuf += QString::number(cPrj->procClouds[i] + 1);
        strBuf += " " + cPrj->cPath[cPrj->procClouds[i]];
        ui->lstProcessList->addItem(strBuf);
    }
    if (updateViewer)
        refreshViewerContents();
}

void MainWindow::refresh()
{
    refreshLists();
    this->updateStatus();
}

void MainWindow::addSphereforKeyPoint(int i, int j)
{
    double r, g, b;
    Utils::getColorFromColorChart(j, r, g, b);
    if (cPrj->getCloudSize() > i)
        viewer->addSphere(cPrj->cRaw[i]->points[cPrj->cKeyPoints[i][j]],
                0.03, r, g, b, QString::number(i * cPrj->getCloudSize() + j).toStdString());
}

void MainWindow::refreshViewerContents()
{
    try
    {
        if (viewer->wasStopped()) return;
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        if (vh->showFusedProcessList)
        {
            if (cPrj->procClouds.size() >= 1)
            {
                updateStatus("Rebuild the graph based on Cloud #" + cPrj->cPath[cPrj->procClouds[0]] + " ...");
                cPrj->rebuildTopoCorrespondences(cPrj->procClouds[0]);
                for (int i = 0; i < cPrj->procClouds.size(); i++)
                    if (i < cPrj->procClouds.size())
                    {
                        CloudPtr transCloud(new Cloud());
                        updateStatus("Transforming and Rendering Point Cloud " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
                        Utils::transformPointCloud(cPrj->cRaw[cPrj->procClouds[i]], transCloud, cPrj->getTransformMatrix(cPrj->procClouds[0], cPrj->procClouds[i]));
                        updateStatus("Rendering Point Cloud " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
                        if (cPrj->cAttachMesh[cPrj->procClouds[i]] == -1 || cPrj->cAttachMesh[cPrj->procClouds[i]] >= cPrj->cAttachMesh.size())
                            viewer->addPointCloud(transCloud, (cPrj->cPath[cPrj->procClouds[i]] + " " + QString::number(i)).toStdString());
                        else
                        {
                            pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2());
                            pcl::toPCLPointCloud2(*transCloud, *cloud);
                            cPrj->sMesh[cPrj->cAttachMesh[cPrj->procClouds[i]]]->cloud = *cloud;
                            viewer->addPolygonMesh(*cPrj->sMesh[cPrj->cAttachMesh[cPrj->procClouds[i]]], cPrj->cPath[cPrj->procClouds[i]].toStdString());
                        }
                    }
            }

            this->refreshLists(false);
        }
        else
        {
            for (int i = 0; i < cPrj->selClouds.size(); i++)
            {
                updateStatus("Rendering Point Cloud " + cPrj->cPath[cPrj->selClouds[i]] + " ...");
                if (i < cPrj->selClouds.size())
                {
                    if (cPrj->cAttachMesh[cPrj->selClouds[i]] == -1 || cPrj->cAttachMesh[cPrj->selClouds[i]] >= cPrj->cAttachMesh.size())
                        viewer->addPointCloud(cPrj->cRaw[cPrj->selClouds[i]], cPrj->cPath[cPrj->selClouds[i]].toStdString());
                    else
                    {
                        if (cPrj->cAttachMesh[cPrj->selClouds[i]] != -1)
                        {
                            pcl::PCLPointCloud2Ptr cloud(new pcl::PCLPointCloud2());
                            pcl::toPCLPointCloud2(*cPrj->cRaw[cPrj->selClouds[i]], *cloud);
                            cPrj->sMesh[cPrj->cAttachMesh[cPrj->selClouds[i]]]->cloud = *cloud;
                            viewer->addPolygonMesh(*cPrj->sMesh[cPrj->cAttachMesh[cPrj->selClouds[i]]], cPrj->cPath[cPrj->selClouds[i]].toStdString());
                        }
                    }
                    for (int j = 0; j < cPrj->cKeyPoints[cPrj->selClouds[i]].size(); j++)
                        addSphereforKeyPoint(cPrj->selClouds[i], j);
                }
            }
        }
        vh->update();
        updateStatus();
    }
    catch (std::exception e)
    {
        updateStatus("An exception catched : " + QString::fromAscii(e.what()));
    }
}

void MainWindow::handlePickedPoint(int index)
{
    if (cPrj->selClouds.size() == 1)
    {
        if (!cPrj->cKeyPoints[cPrj->selClouds[0]].contains(index))
        {
            cPrj->cKeyPoints[cPrj->selClouds[0]].push_back(index);
            this->addSphereforKeyPoint(cPrj->selClouds[0], cPrj->cKeyPoints[cPrj->selClouds[0]].size() - 1);
            cPrj->isSaved = false;
        }
        else
        {
            this->updateStatus("The selected point is already contained in the key point list.");
        }
    }
}

void MainWindow::handlePickedPoint(float x, float y, float z)
{
    if (cPrj->selClouds.size() == 1)
    {
        for (int i = 0; i < cPrj->cRaw[cPrj->selClouds[0]]->points.size(); i++)
        {
            if (cPrj->cRaw[cPrj->selClouds[0]]->points[i].x == x &&
                    cPrj->cRaw[cPrj->selClouds[0]]->points[i].y == y &&
                    cPrj->cRaw[cPrj->selClouds[0]]->points[i].z == z)
            {
                this->handlePickedPoint(i);
                return;
            }
        }
    }
}

void MainWindow::clearAllProcessed()
{
    cPrj->procClouds.clear();
    cPrj->isSaved = false;
    refreshLists();
    refreshViewerContents();
}

void MainWindow::updateStatus(QString status)
{
    lblStatus->setText(status);
    if (status != "Ready")
        ui->txtInfo->append(status + "\n");
    qApp->processEvents();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (cPrj == NULL || cPrj->isSaved || !stayOpen())
    {
        std::cout << "Closing..." << std::endl;
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->close();
        vh->close();
        qApp->exit(0);
    }
    event->ignore();
}

bool MainWindow::stayOpen()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, tr("Save Changes"), tr("The current project ") + cPrj->name + tr(" has not been saved.\nDo you want to save it right now?"), QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    if (reply == QMessageBox::No) return false;
    if (reply == QMessageBox::Cancel) return true;
    if (reply == QMessageBox::Yes)
    {
        this->saveProject();
        return false;
    }
    return false;
}

void MainWindow::on_lstPointClouds_itemDoubleClicked(QListWidgetItem *item)
{
    int index;
    for (index = 0; index < ui->lstPointClouds->count(); index++)
    {
        if (ui->lstPointClouds->item(index) == item)
            break;
    }
    if (cPrj->procClouds.contains(index))
    {
        updateStatus("Cloud #" + QString::number(index + 1) + " is already existed in process list.");
        return;
    }
    cPrj->procClouds.push_back(index);
    cPrj->isSaved = false;
    refreshLists();
}

void MainWindow::on_lstProcessList_itemDoubleClicked(QListWidgetItem *item)
{
    int index;
    for (index = 0; index < ui->lstProcessList->count(); index++)
    {
        if (ui->lstProcessList->item(index) == item)
            break;
    }
    if (index < cPrj->procClouds.size())
        cPrj->procClouds.erase(cPrj->procClouds.begin() + index);
    cPrj->isSaved = false;
    refreshLists();
    updateStatus("The selected clouds and their correspondences have been removed.");
}

void MainWindow::on_lstProcessList_itemSelectionChanged()
{
    refreshSelected(ui->lstProcessList->selectedItems(), ui->lstProcessList);
    for (auto i = cPrj->selClouds.begin(); i != cPrj->selClouds.end(); i++)
        *i = cPrj->procClouds[*i];
    if (!vh->showFusedProcessList)
        refreshViewerContents();
    updateStatus(QString::number(cPrj->selClouds.size()) + " cloud(s) have been selected.");
}

void MainWindow::removeCurrentSelectedClouds()
{
    for (int i = 0; i < cPrj->selClouds.count(); i++)
    {
        for (int j = 0; j < cPrj->procClouds.count(); j++)
        {
            if (cPrj->procClouds[j] == cPrj->selClouds[i])
                cPrj->procClouds.erase(cPrj->procClouds.begin() + j);
            if (cPrj->procClouds[j] >= cPrj->selClouds[i])
                cPrj->procClouds[j]--;
        }
        cPrj->removeCloud(cPrj->selClouds[i] - i);
    }
    refresh();
    updateStatus("The selected clouds and their correspondences have been removed.");
}

void MainWindow::doGrayScaleReg()
{
    CloudPtr cloud_t, cloud_s;
    for (int i = 0; i < cPrj->procClouds.size() - 1; i++)
    {
        updateStatus("Registrating Cloud #" + QString::number(cPrj->procClouds[i] + 1) + " with Cloud #" + QString::number(cPrj->procClouds[i + 1] + 1) + " by Gray Scale Registration...");
        if (cPrj->addCorrespondence(cPrj->procClouds[i], cPrj->procClouds[i + 1]) < 0)
        {
            updateStatus("An existed correspondence banned the registration.");
            continue;
        }
        cloud_t = cPrj->cRaw[cPrj->procClouds[i]];
        cloud_s = cPrj->cRaw[cPrj->procClouds[i + 1]];
        try
        {
            gsr->getGrayScaleRegMatrix(cloud_t, cloud_s, cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1]);
            if (cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1] == Eigen::Matrix4f::Identity())
            {
                cPrj->removeCorrespondence(cPrj->getCorrespondenceSize() - 1);
                updateStatus("The registration didn't return an ideal value. The result is aborted.");
                continue;
            }
        }
        catch (std::exception e)
        {
            updateStatus("An exception occoured : " + QString::fromAscii(e.what()) + " while registrating Cloud #" + QString::number(cPrj->procClouds[i] + 1) + " with Cloud #" + QString::number(cPrj->procClouds[i + 1] + 1) + ".");
            continue;
        }
    }
    refresh();
    updateStatus("Well done!");
}

void MainWindow::doDownSample()
{
    bool ok = false;
    float leafSize = (float)QInputDialog::getDouble(this, "Downsample", "Please input leaf size of Downsample.\n\nTips:\nFor original point cloud, the unit used is meter, and the distance between two neighbors is usually 0.003m.", 0.01, 0, 2173483647, 4, &ok);
    if (ok)
    {
        for (int i = 0; i < cPrj->procClouds.size(); i++)
        {
            updateStatus("Downsampling " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
            Utils::downSamplePointCloud(cPrj->cRaw[cPrj->procClouds[i]], cPrj->cRaw[cPrj->procClouds[i]], leafSize);
            if (cPrj->cIsSaved[cPrj->procClouds[i]])
            {
                updateStatus("Saving " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
                Utils::savePointCloud(cPrj->cRaw[cPrj->procClouds[i]], cPrj->cPath[cPrj->procClouds[i]].toStdString());
            }
        }
        updateStatus("Well done!");
        refreshViewerContents();
    }
    else
    {
        updateStatus("An error occoured while reading the leaf size. Downsample failed.");
    }
}

void MainWindow::doICPReg()
{
    CloudPtr cloud_t, cloud_s;
    for (int i = 0; i < cPrj->procClouds.size() - 1; i++)
    {
        updateStatus("Registrating Cloud #" + QString::number(cPrj->procClouds[i] + 1) + " with Cloud #" + QString::number(cPrj->procClouds[i + 1] + 1) + " by ICP...");
        if (cPrj->addCorrespondence(cPrj->procClouds[i], cPrj->procClouds[i + 1]) < 0)
        {
            updateStatus("An existed correspondence banned the registration.");
            continue;
        }
        cloud_t = cPrj->cRaw[cPrj->procClouds[i]];
        cloud_s = cPrj->cRaw[cPrj->procClouds[i + 1]];
        try
        {
            icp->align(cloud_t, cloud_s, cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1]);
            if (cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1] == Eigen::Matrix4f::Identity())
            {
                cPrj->removeCorrespondence(cPrj->getCorrespondenceSize() - 1);
                updateStatus("The registration didn't return an ideal value. The result is aborted.");
                continue;
            }
        }
        catch (std::exception e)
        {
            updateStatus("An exception occoured : " + QString::fromAscii(e.what()) + " while registrating Cloud #" + QString::number(cPrj->procClouds[i] + 1) + " with Cloud #" + QString::number(cPrj->procClouds[i + 1] + 1) + ".");
            continue;
        }
    }
    refresh();
    updateStatus("Well done!");
}

void MainWindow::doKeyPointReg()
{
    CloudPtr cloud_t, cloud_s;
    for (int i = 0; i < cPrj->procClouds.size() - 1; i++)
    {
        updateStatus("Registrating Cloud #" + QString::number(cPrj->procClouds[i] + 1) + " with Cloud #" + QString::number(cPrj->procClouds[i + 1] + 1) + " by Manual Keypoints...");
        if (cPrj->addCorrespondence(cPrj->procClouds[i], cPrj->procClouds[i + 1]) < 0)
        {
            updateStatus("An existed correspondence banned the registration.");
            continue;
        }
        cloud_t = cPrj->cRaw[cPrj->procClouds[i]];
        cloud_s = cPrj->cRaw[cPrj->procClouds[i + 1]];
        try
        {
            if (reg->keyPointRegistration(cloud_t, cloud_s,
                                          cPrj->cKeyPoints[cPrj->procClouds[i]].toVector().toStdVector(),
                                          cPrj->cKeyPoints[cPrj->procClouds[i + 1]].toVector().toStdVector(),
                                          cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1]) == -1 ||
                    cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1] == Eigen::Matrix4f::Identity())
            {
                cPrj->removeCorrespondence(cPrj->getCorrespondenceSize() - 1);
                updateStatus("The registration didn't return an ideal value. The result is aborted.");
                continue;
            }
        }
        catch (std::exception e)
        {
            updateStatus("An exception occoured : " + QString::fromAscii(e.what()) + " while registrating Cloud #" + QString::number(cPrj->procClouds[i] + 1) + " with Cloud #" + QString::number(cPrj->procClouds[i + 1] + 1) + ".");
            continue;
        }
    }
    refresh();
    updateStatus("Well done!");
}

void MainWindow::createCopy()
{
    QString name;
    for (int i = 0; i < cPrj->procClouds.size(); i++)
    {
        CloudPtr p(new Cloud());
        pcl::copyPointCloud(*cPrj->cRaw[cPrj->procClouds[i]], *p);
        name = cPrj->cPath[cPrj->procClouds[i]] + " [Copy]";
        cPrj->addCloud(p, name);
    }
    refresh();
    updateStatus("Well done!");
}

void MainWindow::doCombine()
{
    CloudPtr combineCloud(new Cloud());
    if (cPrj->procClouds.size() >= 1)
    {
        updateStatus("Rebuild the graph based on Cloud #" + cPrj->cPath[cPrj->procClouds[0]] + " ...");
        cPrj->rebuildTopoCorrespondences(cPrj->procClouds[0]);
        pcl::copyPointCloud(*cPrj->cRaw[cPrj->procClouds[0]], *combineCloud);
        for (int i = 1; i < cPrj->procClouds.size(); i++)
        {
            if (i < cPrj->procClouds.size())
            {
                CloudPtr transCloud(new Cloud());
                updateStatus("Transforming Point Cloud " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
                Utils::transformPointCloud(cPrj->cRaw[cPrj->procClouds[i]], transCloud, cPrj->getTransformMatrix(cPrj->procClouds[0], cPrj->procClouds[i]));
                *combineCloud += *transCloud;
            }
        }
        QString name = "[Combined cloud]";
        cPrj->addCloud(combineCloud, name);
        refresh();
    }
    updateStatus("Well done!");
}

void MainWindow::doMLS()
{
    for (int i = 0; i < cPrj->procClouds.size(); i++)
    {
        updateStatus("Moving least squares for " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
        NormalCloudPtr normalCloud(new NormalCloud());
        surface->mlsSmooth(cPrj->cRaw[cPrj->procClouds[i]], normalCloud);
        Utils::combineField(cPrj->cRaw[cPrj->procClouds[i]], normalCloud);
        Utils::subtractField(normalCloud, cPrj->cRaw[cPrj->procClouds[i]]);
        if (cPrj->cIsSaved[cPrj->procClouds[i]])
        {
            updateStatus("Saving " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
            Utils::savePointCloud(cPrj->cRaw[cPrj->procClouds[i]], cPrj->cPath[cPrj->procClouds[i]].toStdString());
        }
    }
    refreshViewerContents();
    updateStatus("Well done!");
}

void MainWindow::doGP3()
{
    for (int i = 0; i < cPrj->procClouds.size(); i++)
    {
        if (cPrj->cAttachMesh[cPrj->procClouds[i]] != -1)
        {
            updateStatus("An existed mesh banned the reconstruction. Please remove the mesh first. ");
            return;
        }
        updateStatus("Getting normal of " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
        NormalCloudPtr normalCloud(new NormalCloud());
        icp->getNormal(cPrj->cRaw[cPrj->procClouds[i]], normalCloud);
        cPrj->addMesh(cPrj->procClouds[i]);

        updateStatus("Reconstructing " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
        surface->GP3(normalCloud, cPrj->sMesh[cPrj->getMeshSize() - 1]);
    }
    refresh();
    updateStatus("Well done!");
}

void MainWindow::doApplyTransformation()
{
    for (int i = 1; i < cPrj->procClouds.size(); i++)
    {
        updateStatus("Transforming Point Cloud " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
        Utils::transformPointCloud(cPrj->cRaw[cPrj->procClouds[i]], cPrj->cRaw[cPrj->procClouds[i]], cPrj->getTransformMatrix(cPrj->procClouds[0], cPrj->procClouds[i]));
        for (int j = 0; j < cPrj->getCorrespondenceSize(); j++)
        {
            if (cPrj->rSource[j] == cPrj->procClouds[i] || cPrj->rTarget[j] == cPrj->procClouds[i])
                cPrj->removeCorrespondence(j);
        }
        cPrj->addCorrespondence(cPrj->procClouds[0], cPrj->procClouds[i]); // Default identity matrix.
        if (cPrj->cIsSaved[cPrj->procClouds[i]])
        {
            updateStatus("Saving " + cPrj->cPath[cPrj->procClouds[i]] + " ...");
            Utils::savePointCloud(cPrj->cRaw[cPrj->procClouds[i]], cPrj->cPath[cPrj->procClouds[i]].toStdString());
        }
    }
    refresh();
    updateStatus("Well done!");
}

void MainWindow::saveNotSavedPointCloud()
{
    for (int i = 0; i < cPrj->selClouds.size(); i++)
    {
        saveNotSavedPointCloud(cPrj->selClouds[i]);
    }
    updateStatus("Well done!");
    refresh();
}

void MainWindow::saveNotSavedPointCloud(int index)
{
    QString path = QFileDialog::getSaveFileName(this, "Save Cloud #" + QString::number(index + 1), "", tr("PCL Format (*.pcd);;Stanford PLY Format (*.ply)"));
    updateStatus("Saving point cloud " + path + " ...");
    cPrj->savePointCloudInProject(index, path);
}

void MainWindow::saveProject()
{
    if (cPrj->isSaved)
    {
        updateStatus("Current project is already saved, or it's no need to save.");
        return;
    }

    QString path;
    if (cPrj->name == "New Project")
        path = QFileDialog::getSaveFileName(this, "Save Project File", "", tr("Project (*.isproj)"));
    else
        path = cPrj->name;

    int saveClouds = 0;
    for (int i = 0; i < cPrj->getCloudSize(); i++)
    {
        if (!cPrj->cIsSaved[i])
        {
            if (saveClouds == 0)
            {
                QMessageBox::StandardButton reply;
                reply = QMessageBox::question(this, tr("Save Clouds Contained in Project File"), tr("The cloud in your current project ") +
                                              cPrj->cPath[i] + tr(" has not been saved.\nDo you want to save the cloud? If not, the cloud would be abandoned after quiting the program. \n\n") +
                                              tr("Yes - Save the cloud.\nNo - Don't save the cloud at own risk.\nCancel - Stop the save period."),
                                              QMessageBox::Yes | QMessageBox::YesToAll | QMessageBox::No | QMessageBox::NoToAll | QMessageBox::Cancel);
                if (reply == QMessageBox::No)
                {
                    continue;
                }
                if (reply == QMessageBox::Cancel)
                {
                    return;
                }
                if (reply == QMessageBox::Yes)
                {
                    this->saveNotSavedPointCloud(i);
                }
                if (reply == QMessageBox::YesToAll)
                {
                    this->saveNotSavedPointCloud(i);
                    saveClouds = 1;
                }
                if (reply == QMessageBox::NoToAll)
                {
                    saveClouds = -1;
                    break;
                }
            }
            else if (saveClouds == 1)
            {
                this->saveNotSavedPointCloud(i);
            }
        }
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly))
    {
        updateStatus("Cannot save file : " + file.errorString());
        return;
    }

    QString mark;
    cPrj->generateMark(mark);

    QTextStream ts(&file);
    ts << mark;
    cPrj->isSaved = true;
    cPrj->name = path;
    updateStatus(path + " saved successfully.");
    this->setWindowTitle("IRIS Scan Handler - " + cPrj->name);
    refresh();
}

void MainWindow::saveProjectAs()
{
    cPrj->isSaved = false;
    cPrj->name = "New Project";
    this->saveProject();
}

void MainWindow::on_chkShowFusedProcList_clicked()
{
    vh->showFusedProcessList = this->ui->chkShowFusedProcList->isChecked();
    refresh();
}

void MainWindow::on_btnProcToTop_clicked()
{
    if (cPrj->selClouds.size() == 1)
        for (int i = 0; i < cPrj->procClouds.size(); i++)
            if (cPrj->selClouds[0] == cPrj->procClouds[i])
            {
                for (int j = i - 1; j >= 0; j--)
                    cPrj->procClouds[j + 1] = cPrj->procClouds[j];
                cPrj->procClouds[0] = cPrj->selClouds[0];
                refreshLists(false);
                return;
            }
    updateStatus("The select cloud has been set as base.");
}

void MainWindow::on_lstCorrespondences_itemDoubleClicked(QListWidgetItem *item)
{
    int index;
    for (index = 0; index < ui->lstCorrespondences->count(); index++)
    {
        if (ui->lstCorrespondences->item(index) == item)
            break;
    }
    cPrj->removeCorrespondence(index);
    refreshLists();
    updateStatus("The correspondence has been removed.");
}

void MainWindow::openGrabber()
{
    updateStatus("Initalizing Grabber...");
    QFile grabberBin("iris-scan-grabber.exe");
    if (!grabberBin.exists())
    {
        QMessageBox::information(this, "Error", "The grabber binary file cannot be found.\nPlease reinstall the product.", QMessageBox::Ok);
        updateStatus("Aborted.");
        return;
    }
    QProcess *proc = new QProcess(this);
    connect(proc, SIGNAL(readyRead()), this, SLOT(onReturnRead()));
    QStringList list;
    QString path = QFileDialog::getExistingDirectory(this, tr("Select the Path to Save the Captured Clouds"), "");
    if (path == "")
    {
        updateStatus("Aborted.");
        return;
    }
    list.append(path);
    try
    {
        proc->start(grabberBin.fileName(), list);
    }
    catch (std::exception e)
    {
        updateStatus("An exception catched : " + QString::fromAscii(e.what()));
        return;
    }
    updateStatus();
}

void MainWindow::removeRecentKeyPoint()
{
    if (cPrj->selClouds.size() == 1 && cPrj->cKeyPoints[cPrj->selClouds[0]].size() >= 1)
    {
        cPrj->cKeyPoints[cPrj->selClouds[0]].erase(cPrj->cKeyPoints[cPrj->selClouds[0]].end() - 1);
        cPrj->isSaved = false;
        refreshViewerContents();
        updateStatus("The recent created key point has been removed.");
    }
    else
        updateStatus("Only one cloud should be selected for the removal.");
}

void MainWindow::removeAllKeyPoints()
{
    if (cPrj->selClouds.size() == 1 && cPrj->cKeyPoints[cPrj->selClouds[0]].size() >= 1)
    {
        cPrj->cKeyPoints[cPrj->selClouds[0]].clear();
        cPrj->isSaved = false;
        refreshViewerContents();
        updateStatus("All key points have been removed.");
    }
    else
        updateStatus("Only one cloud should be selected for the removal.");
}

void MainWindow::removeAttachMesh()
{
    for (int i = 0; i < cPrj->selClouds.size(); i++)
    {
        cPrj->removeMesh(cPrj->selClouds[i]);
    }
    refresh();
    cPrj->isSaved = false;
    updateStatus("The attached mesh has been removed.");
}

void MainWindow::showSettings()
{
    SettingWindow *sw(new SettingWindow(this));
    sw->exec();
}

void MainWindow::onReturnRead()
{
    QProcess *proc = (QProcess *)(sender());
    QString read = proc->readAll();

    if (read[0] == '>')
    {
        QStringList cache = read.split('|');
        QStringList paths;
        for (int i = 1; i < cache.size() - 1; i++)
        {
            if (cache[i] != "")
                paths.push_back(cache[i]);
        }
        addExternalClouds(paths);
    }
    else if (read.mid(0, 6) == "No dev")
    {
        QMessageBox::information(this, "Error", "No device found.\nPlease check the connection of the sensor, or consult the device support.", QMessageBox::Ok);
        updateStatus("No device found. ");
    }
    else
    {
        ui->txtInfo->append(read);
    }

}
