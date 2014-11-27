#ifndef VIEWHANDLER_H
#define VIEWHANDLER_H

#include "typedef.h"
#include "QVTKWidget.h"
#include "QWidget"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/mouse_event.h>

class ViewHandler : public QVTKWidget
{
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
