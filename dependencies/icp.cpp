#include "icp.h"

ICP::ICP()
{
    this->_ICPIterationTimes = 50;
    this->_ICPIterationTimesPer = 20;
    this->_ICPDownSampleLeafSize = 0.05f;
    this->_ICPMaxCorrespondenceDistance = 0.5f;
    this->_ICPTransformEpsilon = 0.1;
    this->_GetNormalKSearch = 15;
}

int ICP::align(const CloudPtr &target_ori, const CloudPtr &source_ori, Eigen::Matrix4f &align_result)
{
    CloudPtr target(new Cloud()); CloudPtr source(new Cloud());
    pcl::copyPointCloud(*target_ori, *target);
    pcl::copyPointCloud(*source_ori, *source);
    Utils::downSamplePointCloud(target, target, _ICPDownSampleLeafSize);
    Utils::downSamplePointCloud(source, source, _ICPDownSampleLeafSize);
    NormalCloudPtr nc_t(new NormalCloud);
    NormalCloudPtr nc_s(new NormalCloud);
    pcl::copyPointCloud<Point, NormalPoint>((*target), (*nc_t));
    pcl::copyPointCloud<Point, NormalPoint>((*source), (*nc_s));
    ICP::getNormal(target, nc_t);
    ICP::getNormal(source, nc_s);
    pcl::IterativeClosestPointNonLinear<NormalPoint, NormalPoint> reg;
    reg.setTransformationEpsilon(_ICPTransformEpsilon);
    reg.setMaxCorrespondenceDistance(_ICPMaxCorrespondenceDistance);
    reg.setInputSource(nc_s);
    reg.setInputTarget(nc_t);
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev;
    NormalCloudPtr reg_result = nc_s;
    reg.setMaximumIterations(_ICPIterationTimesPer);
    for (int i = 0; i < _ICPIterationTimes; ++i)
    {
        nc_s = reg_result;
        reg.setInputSource(nc_s);
        reg.align(*reg_result);
        Ti = reg.getFinalTransformation() * Ti;
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);
        prev = reg.getLastIncrementalTransformation();
    }
    align_result = Ti;
    return 0;
}

int ICP::getNormal(const CloudPtr &cloud_in, NormalCloudPtr &cloud_out)
{
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    getNormal(cloud_in, cloud_out, tree);
    return 0;
}

int ICP::getNormal(const CloudPtr &cloud_in, NormalCloudPtr &cloud_out, pcl::search::KdTree<Point>::Ptr &tree)
{
    tree->setInputCloud(cloud_in);
    pcl::NormalEstimation<Point, NormalPoint> norm_est;
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(_GetNormalKSearch);
    norm_est.setInputCloud(cloud_in);
    norm_est.compute(*cloud_out);
    Utils::combineField(cloud_in, cloud_out);
    return 0;
}
