#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "stdafx.h"

class Registration
{
public:
    Registration();

    int keyPointRegistration(const OriCloudPtr &cloud_target, const OriCloudPtr &cloud_source, const std::vector<int> &keyPoints_target, const std::vector<int> keyPoints_source, Matrix4f &matrix);
    void getEstimateMatrix(OriCloudPtr cloud_target, OriCloudPtr cloud_source, pcl::Correspondences &corrs, Eigen::Matrix4f &matrix);

    const int maxEstimationMethods;
    int estimationMethod;
};

#endif // REGISTRATION_H
