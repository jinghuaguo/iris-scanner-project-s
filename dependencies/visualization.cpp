#include "visualization.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <cstdio>
#include <cmath>

int Visualization::showMultiClouds(std::vector<CloudPtr> &clouds, int viewPorts)
{
    if (viewPorts < 1)
    {
        viewPorts = clouds.size();
        if (viewPorts < 1)
            return -2;
    }
    pcl::visualization::PCLVisualizer viewer("Viewer");

//    int width = (int)sqrt((double)viewPorts);
//    int height = (int)(viewPorts / width);
//    if (width * height < viewPorts) height++;

//    std::vector<int> portRef;
//    for (int i = 0; i < viewPorts; i++)
//        portRef.push_back(i);

//    for (int i = 0; i < viewPorts; i++)
//    {
//        portRef[i] = i;
//        viewer.createViewPort((1.0 / width) * (i % width),
//                              (1.0 / height) * (i / height),
//                              (1.0 / width) * (i % width + 1),
//                              (1.0 / height) * (i / height + 1),
//                              portRef[i]);
//    }

    viewer.initCameraParameters();
    std::string str_buf;
    char count_buf[10];

    for (int i = 0; i < clouds.size(); i++)
    {
        str_buf.clear();
        str_buf.push_back('p');
        sprintf_s(count_buf, "%d", i);
        str_buf.append(count_buf);

        viewer.addPointCloud(clouds[i], str_buf/*, portRef[i % viewPorts]*/);
    }

    viewer.setShowFPS(false);
    viewer.setCameraPosition(-1, 1, -10, 0, -1, 0);
    viewer.setPosition(200, 75);
    viewer.setSize(800, 600);
    viewer.spin();

    return 0;
}
