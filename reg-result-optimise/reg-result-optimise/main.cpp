#include "utils.h"
#include "typedef.h"
#include "visualization.h"
#include "grayscaleregistration.h"

#include "vector"
#include "iostream"
#include "ctime"
#include "cmath"

#include <pcl/kdtree/kdtree_flann.h>

void markPair(Point &a, Point &b)
{
    a.r = 255;
    a.g = 0;
    a.b = 0;
    a.a = 255;

    b.r = 0;
    b.g = 255;
    b.b = 0;
    b.a = 255;
}

double colorRank(const Point *a, const Point *b)
{
    double aLength = sqrt(pow((double)a->r, 2) + pow((double)a->g, 2) + pow((double)a->b, 2));
    double bLength = sqrt(pow((double)b->r, 2) + pow((double)b->g, 2) + pow((double)b->b, 2));
    double dotProject = a->r * b->r + a->g * b->g + a->b * b->b;
    if (aLength * bLength == 0)
        if (aLength == bLength)
            return 1;
        else
            return 0;
    else
        return dotProject / (aLength * bLength);
}

int main(int argc, char *argv[])
{
    clock_t tStart, tStop;
    argc = 2;
    argv = new char*[argc];
    argv[0] = "D:\\PCDs\\20140817\\0001.pcd";
    argv[1] = "D:\\PCDs\\20140817\\0003.pcd";
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
    std::vector<int> kIndices;
    std::vector<float> kDistances;

    CloudPtr combined(new Cloud());
    (*combined) = (*clouds[0]);
    (*combined) += (*clouds[1]);

    tree->setInputCloud(combined);
    int sum = 0; double colorRankMean = 0;
    tStart = clock();
    for (int i = 0; i < combined->points.size() / 2; i++)
    {
        if (combined->points[i].z == combined->points[i].z)
        {
            tree->nearestKSearch(combined->points[i], 16, kIndices, kDistances);
            //tree->radiusSearch(combined->points[i], 0.015, kIndices, kDistances);
            int minIndice = -1; double minDistance = 1;
            for (int j = 1; j < kIndices.size(); j++)
            {
                if (kIndices[j] > combined->points.size() / 2 && kDistances[j] < minDistance)
                {
                    minDistance = kDistances[j];
                    minIndice = j;
                }
            }
            if (minIndice != -1)
            {
                colorRankMean += colorRank(&(combined->points[i]), &(combined->points[kIndices[minIndice]]));
                markPair(combined->points[i], combined->points[kIndices[minIndice]]);
                sum++;
            }
        }
    }
    tStop = clock();
    std::cout << "Find neighbors completed." << std::endl <<
                 "Pairs:" << sum << " Color Rank:" << colorRankMean <<
                 " Color Rank Mean:" << colorRankMean / sum << std::endl <<
                 "Duration : " << (double)(tStop - tStart) / CLK_TCK << std::endl;

    clouds.clear();
    clouds.push_back(combined);
    Visualization::showMultiClouds(clouds, 1);
    delete returnVal;
}
