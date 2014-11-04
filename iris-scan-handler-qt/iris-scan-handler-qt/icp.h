#ifndef ICP_H
#define ICP_H

#include "stdafx.h"
#include "utils.h"
#include "registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

class ICP : Registration
{
public:
    ICP();
    int align(const OriCloudPtr &target_ori, const OriCloudPtr &source_ori, Eigen::Matrix4f &align_result);
    int getNormal(const OriCloudPtr &cloud_in, NormalCloudPtr &cloud_out);
    int getNormal(const OriCloudPtr &cloud_in, NormalCloudPtr &cloud_out, pcl::search::KdTree<OriPoint>::Ptr &tree);

    int _ICPIterationTimes;
    int _ICPIterationTimesPer;
    float _ICPDownSampleLeafSize;
    float _ICPMaxCorrespondenceDistance;
    double _ICPTransformEpsilon;
    int _GetNormalKSearch;
};

#endif // ICP_H
