#ifndef SURFACE_H
#define SURFACE_H

#include "stdafx.h"
#include "utils.h"
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>

class Surface
{
public:
    Surface();
    int GP3(const NormalCloudPtr &cloud_in, const pcl::PolygonMeshPtr &mesh_out);
    int mlsSmooth(const OriCloudPtr &cloud_in, const NormalCloudPtr &cloud_out);

    double _mlsRadius;
    double _gp3Radius;
    double _gp3Mu;
    int _gp3NearestNeighbors;
};

#endif // SURFACE_H
