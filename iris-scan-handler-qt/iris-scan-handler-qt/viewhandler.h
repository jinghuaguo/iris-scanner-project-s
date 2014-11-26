#ifndef VIEWHANDLER_H
#define VIEWHANDLER_H

#include "typedef.h"
#include "mainwindow.h"
#include "QVTKWidget.h"
#include "QWidget"


#ifndef Q_MOC_RUN

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/mouse_event.h>

#endif

class ViewHandler : public QVTKWidget
{
    Q_OBJECT

public:
    ViewHandler();
    ViewHandler(QWidget *parent);
    void initParameter();
    pcl::visualization::PCLVisualizer viewer;
    bool showFusedProcessList;
    bool showCoordinateSystem;

    void pointPickingEvent(const pcl::visualization::PointPickingEvent &event, void *);
};

#endif // VIEWHANDLER_H
