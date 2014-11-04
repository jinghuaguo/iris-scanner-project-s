#include "stdafx.h"
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

int ICP::align(const OriCloudPtr &target_ori, const OriCloudPtr &source_ori, Eigen::Matrix4f &align_result)
{
    OriCloudPtr target(new OriCloud()); OriCloudPtr source(new OriCloud());
    pcl::copyPointCloud(*target_ori, *target);
    pcl::copyPointCloud(*source_ori, *source);
    std::cout << "Downsampling point clouds..." << std::endl;
    Utils::downSamplePointCloud(target, target, _ICPDownSampleLeafSize);
    Utils::downSamplePointCloud(source, source, _ICPDownSampleLeafSize);
    std::cout << "Estimating Normals..." << std::endl;
    NormalCloudPtr nc_t(new NormalCloud);
    NormalCloudPtr nc_s(new NormalCloud);
    pcl::copyPointCloud<OriPoint, NormalPoint>((*target), (*nc_t));
    pcl::copyPointCloud<OriPoint, NormalPoint>((*source), (*nc_s));
    ICP::getNormal(target, nc_t);
    ICP::getNormal(source, nc_s);
    std::cout << "Aligning..." << std::endl;
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
        //std::cout << "Iteration #" << i + 1 << " ..." << std::endl;
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

int ICP::getNormal(const OriCloudPtr &cloud_in, NormalCloudPtr &cloud_out)
{
    pcl::search::KdTree<OriPoint>::Ptr tree(new pcl::search::KdTree<OriPoint>());
    getNormal(cloud_in, cloud_out, tree);
    return 0;
}

int ICP::getNormal(const OriCloudPtr &cloud_in, NormalCloudPtr &cloud_out, pcl::search::KdTree<OriPoint>::Ptr &tree)
{
    tree->setInputCloud(cloud_in);
    pcl::NormalEstimation<OriPoint, NormalPoint> norm_est;
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(_GetNormalKSearch);
    norm_est.setInputCloud(cloud_in);
    norm_est.compute(*cloud_out);
    Utils::combineField(cloud_in, cloud_out);
    return 0;
}
