#ifndef STDAFX_H
#define STDAFX_H

#ifndef Q_MOC_RUN

#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#endif

typedef pcl::PointXYZRGBA OriPoint;
typedef pcl::PointCloud<OriPoint> OriCloud;
typedef OriCloud::Ptr OriCloudPtr;
typedef pcl::PointXYZRGBNormal NormalPoint;
typedef pcl::PointCloud<NormalPoint> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;
typedef pcl::Normal OnlyNormalPoint;
typedef pcl::PointCloud<OnlyNormalPoint> OnlyNormalCloud;
typedef OnlyNormalCloud::Ptr OnlyNormalCloudPtr;
typedef pcl::Vertices SurfacePoint;
typedef pcl::PointCloud<SurfacePoint> SurfaceCloud;
typedef SurfaceCloud::Ptr SurfaceCloudPtr;
typedef OriPoint Point;
typedef OriCloud Cloud;
typedef OriCloudPtr CloudPtr;
typedef Eigen::Matrix4f Matrix4f;

const Matrix4f Identity4f = Matrix4f::Identity();
const double ISProjMaxVersion = 0.1;

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#endif // STDAFX_H
