#include "registration.h"

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>

Registration::Registration() :
    maxEstimationMethods(4),
    estimationMethod(2)
{
}

int Registration::keyPointRegistration(const OriCloudPtr &cloud_target, const OriCloudPtr &cloud_source, const std::vector<int> &keyPoints_target, const std::vector<int> keyPoints_source, Matrix4f &matrix)
{
    pcl::Correspondences correspondences;
    int corrsSum = (keyPoints_target.size() < keyPoints_source.size() ? keyPoints_target.size() : keyPoints_source.size());
    if (corrsSum < 6)
        return -1;
    for (int i = 0; i < corrsSum; i++)
    {
        pcl::Correspondence correspondence;
        correspondence.index_query = keyPoints_source[i];
        correspondence.index_match = keyPoints_target[i];
        correspondences.push_back(correspondence);
    }
    this->getEstimateMatrix(cloud_target, cloud_source, correspondences, matrix);
    std::cout << matrix << std::endl;
    return 0;
}

void Registration::getEstimateMatrix(OriCloudPtr cloud_target, OriCloudPtr cloud_source, pcl::Correspondences &corrs, Eigen::Matrix4f &matrix)
{
    pcl::registration::TransformationEstimation<OriPoint, OriPoint> *estimation;
    switch (Registration::estimationMethod)
    {
    case 0:
        estimation = new pcl::registration::TransformationEstimationLM<OriPoint, OriPoint>();
        break;
    case 1:
        estimation = new pcl::registration::TransformationEstimationSVD<OriPoint, OriPoint>();
        break;
    case 2:
        estimation = new pcl::registration::TransformationEstimationDualQuaternion<OriPoint, OriPoint>();
        break;
    case 3:
        estimation = new pcl::registration::TransformationEstimation2D<OriPoint, OriPoint>();
        break;
    default:
        return;
        break;
    }
    estimation->estimateRigidTransformation((*cloud_source), (*cloud_target), corrs, matrix);
}
