#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "typedef.h"

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>

class Registration
{
public:
    Registration();

    int keyPointRegistration(const CloudPtr &cloud_target, const CloudPtr &cloud_source, const std::vector<int> &keyPoints_target, const std::vector<int> keyPoints_source, Matrix4f &matrix);
    void getEstimateMatrix(CloudPtr cloud_target, CloudPtr cloud_source, pcl::Correspondences &corrs, Eigen::Matrix4f &matrix);

    const int maxEstimationMethods;
    int estimationMethod;
};

#endif // REGISTRATION_H
