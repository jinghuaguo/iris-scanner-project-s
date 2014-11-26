#include "registration.h"

Registration::Registration() :
    maxEstimationMethods(4),
    estimationMethod(2)
{
}

int Registration::keyPointRegistration(const CloudPtr &cloud_target, const CloudPtr &cloud_source, const std::vector<int> &keyPoints_target, const std::vector<int> keyPoints_source, Matrix4f &matrix)
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

void Registration::getEstimateMatrix(CloudPtr cloud_target, CloudPtr cloud_source, pcl::Correspondences &corrs, Eigen::Matrix4f &matrix)
{
    pcl::registration::TransformationEstimation<Point, Point> *estimation;
    switch (Registration::estimationMethod)
    {
    case 0:
        estimation = new pcl::registration::TransformationEstimationLM<Point, Point>();
        break;
    case 1:
        estimation = new pcl::registration::TransformationEstimationSVD<Point, Point>();
        break;
    case 2:
        estimation = new pcl::registration::TransformationEstimationDualQuaternion<Point, Point>();
        break;
    case 3:
        estimation = new pcl::registration::TransformationEstimation2D<Point, Point>();
        break;
    default:
        return;
        break;
    }
    estimation->estimateRigidTransformation((*cloud_source), (*cloud_target), corrs, matrix);
}
