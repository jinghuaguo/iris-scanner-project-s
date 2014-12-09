#ifndef _VISUALIZATION_H_
#define _VISUALIZATION_H_

#include "typedef.h"

#include <vector>

class Visualization
{
public:
    static int showMultiClouds(std::vector<CloudPtr> &clouds, int viewPorts = 0);
};

#endif
