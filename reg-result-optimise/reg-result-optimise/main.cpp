#include "utils.h"
#include "typedef.h"
#include "visualization.h"
#include "grayscaleregistration.h"

#include <pcl/kdtree/kdtree_flann.h>

#include "vector"
#include "iostream"
#include "ctime"

int main(int argc, char *argv[])
{
    clock_t tStart, tStop;
    argc = 2;
    argv = new char*[argc];
    argv[0] = "D:\\PCDs\\20140817\\0003.pcd";
    argv[1] = "D:\\PCDs\\20140817\\0001.pcd";
    std::vector <CloudPtr> clouds;
    int *returnVal = new int[1];
    tStart = clock();
    (*returnVal) = Utils::readPointClouds(argc, argv, clouds);
    if ((*returnVal) != 0)
        std::cerr << "An error occoured." << (*returnVal) << std::endl;
    tStop = clock();
    std::cout << clouds.size() << " cloud(s) have been loaded." << std::endl << "Duration : " << (double)(tStop - tStart) / CLK_TCK << std::endl;
    CloudPtr cache(new Cloud());
    Matrix4f matrix;
    GrayScaleRegistration gsr;
    gsr.getGrayScaleRegMatrix(clouds[0], clouds[1], matrix);
    Utils::transformPointCloud(clouds[1], cache, matrix);
    cache->swap(*clouds[1]);
    cache.reset();

    pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>());
    tree->setEpsilon(0.15f);
    std::vector<int> kIndices;
    std::vector<float> kDistances;

    CloudPtr combined(new Cloud());
    (*combined) = (*clouds[0]) + (*clouds[1]);
    tree->setInputCloud(combined);
    tStart = clock();
    for(int i = 0; i < combined->points.size() / 2; i++)
    {
        tree->nearestKSearch(combined->points[i], 1, kIndices, kDistances);
        if (kIndices.size() > 0 && kIndices[0] > combined->points.size() / 2)
        {
            std::cout << "Point No." << i << " has been found repeated with No." << kIndices[0] << ". Distance : " << kDistances[0] << std::endl;
            combined->points[i].r = combined->points[i].g = combined->points[i].b = 255;
            combined->points[kIndices[0]].r = combined->points[kIndices[0]].g = combined->points[kIndices[0]].b = 255;
        }
    }
    tStop = clock();
    std::cout << "Find neighbors completed." << std::endl << "Duration : " << (double)(tStop - tStart) / CLK_TCK << std::endl;

    clouds.clear();
    clouds.push_back(combined);
    Visualization::showMultiClouds(clouds, 1);
    delete returnVal;
}
