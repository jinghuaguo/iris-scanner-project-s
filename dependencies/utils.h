#ifndef UTILS_H
#define UTILS_H

#include "typedef.h"

#include <cmath>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>

class Utils
{
public:
    static double getDistance(const Point &p1, const Point &p2);
    static int readPointClouds(const int &count, char** &paths, std::vector<CloudPtr> &cloud_list);
    static int readSinglePointCloud(const std::string &path, const CloudPtr cloud);
    static int readSinglePointCloud(const std::string &path, const pcl::PCLPointCloud2Ptr cloud);
    static int savePointCloud(const CloudPtr &cloud, const std::string &path = "");
    static int savePointCloud(const pcl::PolygonMeshPtr polygonMesh, const std::string &path = "");
    static int saveTXTFile(std::string path, Cloud &cloud);
    static int combinePointCloud(const CloudPtr &cloud1, const CloudPtr &cloud2, const CloudPtr &combinedCloud, float leaf_size = 0.005f);
    static int combinePointCloud(const std::vector<CloudPtr> clouds, const CloudPtr &combinedCloud, float leaf_size = 0.005f);
    static int downSamplePointCloud(const CloudPtr &cloud_in, const CloudPtr &cloud_out, float leaf_size = 0.005f);
    static int transformPointCloud(const CloudPtr &cloud_in, const CloudPtr &cloud_out, const Eigen::Matrix4f &transformMatrix);
    static int combineField(const CloudPtr &cloud_in, const NormalCloudPtr &cloud_out);
    static int subtractField(const NormalCloudPtr &cloud_in, const CloudPtr &cloud_out);
    static void getColorFromColorChart(const int index, double &r, double &g, double &b);
    static double getColorChartElement(int index);
};

#endif // UTILS_H
