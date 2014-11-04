#ifndef UTILS_H
#define UTILS_H

#include "stdafx.h"

class Utils
{
public:
    static void savePointCloud(const OriCloudPtr &cloud, const std::string &path = "D:\\saved_cloud.pcd");
    static int transformPointCloud(const OriCloudPtr &cloud_in, const OriCloudPtr &cloud_out, const Eigen::Matrix4f &transformMatrix);
    static void getColorFromChart(const int index, double &r, double &g, double &b);
};

#endif // UTILS_H
