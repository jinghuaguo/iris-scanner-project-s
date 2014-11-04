#ifndef UTILS_H
#define UTILS_H

#include "stdafx.h"

class Utils
{
public:
    static double getDistance(const OriPoint &p1, const OriPoint &p2);
    static int readPointClouds(const int &argc, char** &argv, std::vector<OriCloudPtr> &cloud_list);
    static int readSinglePointCloud(const std::string &path, const OriCloudPtr cloud);
    static int readSinglePointCloud(const std::string &path, const pcl::PCLPointCloud2Ptr cloud);
    static void savePointCloud(const OriCloudPtr &cloud, const std::string &path = "D:\\saved_cloud.pcd", const std::string type = "binary");
    static void savePointCloud(const pcl::PolygonMeshPtr polygonMesh, const std::string &path = "D:\\saved_cloud.pcd");
    static int saveTXTFile(std::string path, OriCloud &cloud);
    static int combinePointCloud(const OriCloudPtr &cloud1, const OriCloudPtr &cloud2, const OriCloudPtr &combinedCloud, float leaf_size = 0.005f);
    static int combinePointCloud(const std::vector<OriCloudPtr> clouds, const OriCloudPtr &combinedCloud, float leaf_size = 0.005f);
    static int downSamplePointCloud(const OriCloudPtr &cloud_in, const OriCloudPtr &cloud_out, float leaf_size = 0.005f);
    static int transformPointCloud(const OriCloudPtr &cloud_in, const OriCloudPtr &cloud_out, const Eigen::Matrix4f &transformMatrix);
    static int combineField(const OriCloudPtr &cloud_in, const NormalCloudPtr &cloud_to_combine);
    static int subtractField(const NormalCloudPtr &cloud_in, const OriCloudPtr &cloud_out);
    static void getColorFromChart(const int index, double &r, double &g, double &b);
};

#endif // UTILS_H
