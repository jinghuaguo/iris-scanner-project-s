#include "stdafx.h"
#include "utils.h"

void Utils::savePointCloud(const OriCloudPtr &cloud, const std::string &path)
{
    if (pcl::io::savePCDFileBinaryCompressed(path, *cloud) == 0)
    {
        std::cout << "Saved successfully." << std::endl;
    }
    else
    {
        std::cout << "Failed in saving the point cloud." << std::endl;
    }
}

int Utils::transformPointCloud(const OriCloudPtr &cloud_in, const OriCloudPtr &cloud_out, const Eigen::Matrix4f &transformMatrix)
{
    pcl::transformPointCloud<OriPoint>((*cloud_in), (*cloud_out), transformMatrix);
    return 0;
}

const double colorChart[] = {0, 0, 255, 127, 255, 0, 100, 149, 237, 220, 20, 60, 255, 140, 0, 255, 20, 147, 30, 144, 255, 173, 255, 47, 255, 0, 255, 255, 255, 0};

void Utils::getColorFromChart(const int index, double &r, double &g, double &b)
{
    int x = index % 10 * 3;
    r = colorChart[x] / 255.0;
    g = colorChart[x + 1] / 255.0;
    b = colorChart[x + 2] / 255.0;
    return;
}
