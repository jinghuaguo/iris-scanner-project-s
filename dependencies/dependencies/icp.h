#ifndef ICP_H
#define ICP_H

#include "typedef.h"
#include "registration.h"
#include "utils.h"

#include <pcl/registration/icp_nl.h>
#include <pcl/features/normal_3d.h>

class ICP : Registration
{
public:
    ICP();
    int align(const CloudPtr &target_ori, const CloudPtr &source_ori, Eigen::Matrix4f &align_result);
    int getNormal(const CloudPtr &cloud_in, NormalCloudPtr &cloud_out);
    int getNormal(const CloudPtr &cloud_in, NormalCloudPtr &cloud_out, pcl::search::KdTree<Point>::Ptr &tree);

    int _ICPIterationTimes;
    int _ICPIterationTimesPer;
    float _ICPDownSampleLeafSize;
    float _ICPMaxCorrespondenceDistance;
    double _ICPTransformEpsilon;
    int _GetNormalKSearch;
};

#endif // ICP_H
