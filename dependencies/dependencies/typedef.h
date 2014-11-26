#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointXYZRGBNormal NormalPoint;
typedef pcl::PointCloud<NormalPoint> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;
typedef pcl::Normal OnlyNormalPoint;
typedef pcl::PointCloud<OnlyNormalPoint> OnlyNormalCloud;
typedef OnlyNormalCloud::Ptr OnlyNormalCloudPtr;\
typedef Eigen::Matrix4f Matrix4f;

const Matrix4f Identity4f = Matrix4f::Identity();

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#endif // TYPEDEF_H
