#include "grayscaleregistration.h"
#include "registration.h"

GrayScaleRegistration::GrayScaleRegistration()
{
    this->_GSRMaxRemoveDistanceThreshold = 4;
    this->_GSRSurfThreshold = 750;
}

IplImage* GrayScaleRegistration::getIplImage(const CloudPtr &input_cloud)
{
    IplImage *image = cvCreateImage(cvSize((*input_cloud).width, (*input_cloud).height), IPL_DEPTH_8U, 3);
    cv::Scalar pixel;
    for (int i = 0; i < (*image).height; i++)
    {
        for (int j = 0; j < (*image).width; j++)
        {
            pixel.val[0] = (*input_cloud).points[i*(*image).width + j].b;
            pixel.val[1] = (*input_cloud).points[i*(*image).width + j].g;
            pixel.val[2] = (*input_cloud).points[i*(*image).width + j].r;
            cvSet2D(image, i, j, pixel);
        }
    }
    return image;
}

void GrayScaleRegistration::getSURFDescriptors(const CloudPtr &cloud, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    this->getSURFDescriptors(cloud, keypoints, descriptors, _GSRSurfThreshold);
}

void GrayScaleRegistration::getSURFDescriptors(const CloudPtr &cloud, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors, int surf_threshold)
{
    IplImage *oriImage = getIplImage(cloud);
    IplImage *grayImage = cvCreateImage(cvGetSize(oriImage), IPL_DEPTH_8U, 1);
    cvCvtColor(oriImage, grayImage, CV_RGB2GRAY);
    cv::SurfFeatureDetector detector(surf_threshold);
    detector.detect(grayImage, keypoints);
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(grayImage, keypoints, descriptors);
}

void GrayScaleRegistration::getGrayScaleRegMatrix(const CloudPtr &cloud_target, const CloudPtr &cloud_source, Matrix4f &matrix)
{
    cv::Mat descriptorsSource, descriptorsTarget;
    std::vector<cv::KeyPoint> keypointsSource, keypointsTarget;

    int init_threshold = _GSRSurfThreshold;
    while (keypointsSource.size() < 10 || keypointsTarget.size() < 10)
    {
        this->getSURFDescriptors(cloud_target, keypointsTarget, descriptorsTarget, init_threshold);
        this->getSURFDescriptors(cloud_source, keypointsSource, descriptorsSource, init_threshold);

        init_threshold /= 2;
        if (init_threshold <= 4)
        {
            matrix = Identity4f;
            return;
        }
    }

    pcl::Correspondences correspondences;

    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;
    matcher.match(descriptorsSource, descriptorsTarget, matches);

    std::vector<cv::Point2f> sourcePoints;
    std::vector<cv::Point2f> targetPoints;
    for (int i = 0; i < matches.size(); i++)
    {
        sourcePoints.push_back(keypointsSource[matches[i].queryIdx].pt);
        targetPoints.push_back(keypointsTarget[matches[i].trainIdx].pt);
    }

    std::vector<uchar> m_RANSACStatus;
    cv::findHomography(sourcePoints, targetPoints, CV_RANSAC, 3.0, m_RANSACStatus);

    int temp_x_target, temp_y_target, temp_index_target, temp_x_source, temp_y_source, temp_index_source;
    for (int i = 0; i < matches.size(); i++)
    {
        if (m_RANSACStatus[i] == 1)
        {
            temp_x_target = (int)(targetPoints[i].x + 0.5);
            temp_y_target = (int)(targetPoints[i].y + 0.5);
            temp_index_target = temp_y_target * cloud_target->width + temp_x_target;

            temp_x_source = (int)(sourcePoints[i].x + 0.5);
            temp_y_source = (int)(sourcePoints[i].y + 0.5);
            temp_index_source = temp_y_source * cloud_source->width + temp_x_source;

            if ((*cloud_source).points[temp_index_source].z != (*cloud_source).points[temp_index_source].z)
                continue;
            if ((*cloud_target).points[temp_index_target].z != (*cloud_target).points[temp_index_target].z)
                continue;

            pcl::Correspondence correspondence;
            correspondence.index_query = temp_index_source;
            correspondence.index_match = temp_index_target;
            correspondences.push_back(correspondence);
        }
    }

    this->getEstimateMatrix(cloud_target, cloud_source, correspondences, matrix);
}
