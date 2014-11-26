#ifndef GRAYSCALEREGISTRATION_H
#define GRAYSCALEREGISTRATION_H

#include "typedef.h"
#include "utils.h"
#include "registration.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

class GrayScaleRegistration : Registration
{
public:
    GrayScaleRegistration();
    IplImage *getIplImage(const CloudPtr &input_cloud);
    void getGrayScaleRegMatrix(const CloudPtr &cloud_target, const CloudPtr &cloud_source, Matrix4f &matrix);
    void getSURFDescriptors(const CloudPtr &cloud, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    void getSURFDescriptors(const CloudPtr &cloud, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, int surf_threshold);

    double _GSRMaxRemoveDistanceThreshold;
    int _GSRSurfThreshold;
};
#endif // GRAYSCALEREGISTRATION_H
