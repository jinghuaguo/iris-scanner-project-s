#include "stdafx.h"
#include "surface.h"

Surface::Surface() :
    _mlsRadius(0.1),
    _gp3Radius(0.15),
    _gp3Mu(4),
    _gp3NearestNeighbors(100)
{
}

int Surface::GP3(const NormalCloudPtr &cloud_in, const pcl::PolygonMeshPtr &mesh_out)
{
    pcl::GreedyProjectionTriangulation<NormalPoint> gp3;
    gp3.setSearchRadius(_gp3Radius);
    gp3.setMu(_gp3Mu);
    gp3.setMaximumNearestNeighbors(_gp3NearestNeighbors);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(cloud_in);
    pcl::search::KdTree<NormalPoint>::Ptr tree(new pcl::search::KdTree<NormalPoint>);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*mesh_out);
    return 0;
}

int Surface::mlsSmooth(const OriCloudPtr &cloud_in, const NormalCloudPtr &cloud_out)
{
    pcl::search::KdTree<OriPoint>::Ptr tree(new pcl::search::KdTree<OriPoint>);
    pcl::MovingLeastSquares<OriPoint, NormalPoint> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_in);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(_mlsRadius);
    mls.process(*cloud_out);
    return 0;
}
