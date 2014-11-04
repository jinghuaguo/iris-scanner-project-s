#include "stdafx.h"
#include "viewhandler.h"
#include "mainwindow.h"

extern MainWindow *w_ptr;

ViewHandler::ViewHandler()
{
}

ViewHandler::ViewHandler(QWidget *parent) : QVTKWidget(parent)
{
    this->initParameter();
    vtkRenderWindow *renderWindow;
    renderWindow = viewer.getRenderWindow();
    this->SetRenderWindow(renderWindow);
}

void ViewHandler::pointPickingEvent(const pcl::visualization::PointPickingEvent &event, void *)
{
    if (event.getPointIndex() != -1)
    {
        float x, y, z;
        event.getPoint(x, y, z);
        w_ptr->handlePickedPoint(x, y, z);
    }
}

void ViewHandler::initParameter()
{
    viewer.setShowFPS(false);
    viewer.setCameraPosition(-1, 1, -10, 0, -1, 0);
    viewer.registerPointPickingCallback(&ViewHandler::pointPickingEvent, *this);
    this->showCoordinateSystem = false;
    this->showFusedProcessList = false;
}
