#include <iostream>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/pcd_io.h>

#include "stdafx.h"
#include "project.h"
#include "grayscaleregistration.h"

#include <string>
#include <sstream>
#include <cmath>

#include <QString>
#include <QTextStream>
#include <QFile>
#include <QDir>
#include <QFileInfo>

const std::string currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[85];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y %m %d %H %M %S", &tstruct);
    return buf;
}

Project *cPrj;

void cropCloud(OriCloudPtr &c, int cropSize = 20)
{
    int w = c->width;
    int h = c->height;
    for (int i = 0; i < w; i++)
        for (int j = 0; j < cropSize; j++)
        {
            c->points[j * w + i].z = sqrt(-1.0f);
        }
    for (int i = 0; i < w; i++)
        for (int j = h - cropSize; j < h; j++)
        {
            c->points[j * w + i].z = sqrt(-1.0f);
        }
    for (int i = 0; i < cropSize; i++)
        for (int j = 0; j < h; j++)
        {
            c->points[j * w + i].z = sqrt(-1.0f);
        }
    for (int i = w - cropSize; i < w; i++)
        for (int j = 0; j < h; j++)
        {
            c->points[j * w + i].z = sqrt(-1.0f);
        }
}

template <typename PointType>
class OpenNIViewer
{
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    OpenNIViewer(pcl::Grabber& grabber)
        : cloud_viewer_(new pcl::visualization::PCLVisualizer("Initalizing..."))
        , grabber_(grabber)
    {
        matL1 = Identity4f;
        matL2 = Identity4f;
        matL3 = Identity4f;
    }

    bool isShaking(Matrix4f &mat)
    {
        if      (abs(mat(0) - 1) > 0.1 || abs(mat(5) - 1) > 0.1 || abs(mat(10) - 1) > 0.1 ||
                         abs(mat(1)) > 0.1 || abs(mat(2)) > 0.1 || abs(mat(4)) > 0.1 || abs(mat(6)) > 0.1 ||
                         abs(mat(8)) > 0.1 || abs(mat(9)) > 0.1 ||
                         abs(mat(12)) > 0.07 || abs(mat(13)) > 0.07 || abs(mat(14)) > 0.07)
            return true;

        if (mat == Identity4f)
            return true;
        return false;
    }

    bool isIllegal(Matrix4f &mat)
    {
        if      (abs(mat(0) - 1) > 0.3 || abs(mat(5) - 1) > 0.3 || abs(mat(10) - 1) > 0.3 ||
                 abs(mat(1)) > 0.4 || abs(mat(2)) > 0.4 || abs(mat(4)) > 0.4 || abs(mat(6)) > 0.4 ||
                 abs(mat(8)) > 0.4 || abs(mat(9)) > 0.4 ||
                 abs(mat(12)) > 1 || abs(mat(13)) > 1 || abs(mat(14)) > 1)
            return true;

        if (mat == Identity4f)
            return true;
        return false;
    }

    CloudPtr cloud_iter, cloud_realtime_last, cloud_swap;
    bool trackLost, shaking, shakingBefore, needRefreshOutput;

    Matrix4f matL2, matL3;
    unsigned int cloudClock;

    void cloud_callback(const CloudConstPtr& cloud)
    {
        std::stringstream outputStream;

        boost::mutex::scoped_lock lock(cloud_mutex_);
        cloud_ = cloud;
        if (cloudClock == 0xff)
            cloudClock = 0;
        else
            cloudClock++;

        if (track)
        {
            GrayScaleRegistration gsr;

            if (cloudClock % 4 == 0)
            {
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c2t(new pcl::PointCloud<pcl::PointXYZRGBA>());
                (*c2t) = (*cloud);
                if (cloud_realtime_last != 0)
                {
                    Matrix4f gsrResult;
                    gsr.getGrayScaleRegMatrix(cloud_realtime_last, c2t, gsrResult);
                    shaking = isShaking(gsrResult);
                    if (shakingBefore != shaking)
                    {
                        outputStream << "Capture state : \n" << (shaking ? "| SHAKING, or features are not enough." : "| Available for capture") << std::endl;
                        shakingBefore = shaking;

                        if (!shaking && cPrj->getCloudSize() > 0)
                        {
                            if (cloud_iter != 0)
                            {
                                gsr.getGrayScaleRegMatrix(cloud_iter, c2t, matL2);
                                if (isIllegal(matL2))
                                {
                                    cloud_iter = cPrj->cRaw[cPrj->getCloudSize() - 1];
                                    matL2 = Identity4f;
                                    matL3 = Identity4f;
                                    outputStream << "Iteration track lost. Reset the tracking." << std::endl;
                                }
                                else
                                {
                                    matL3 = matL2 * matL3;
                                    outputStream << "Iteration track got." << std::endl << "Current transform matrix : " << std::endl << matL3 << std::endl;
                                    cloud_iter = c2t;
                                }
                            }
                            else
                            {
                                cloud_iter = c2t;
                            }
                        }
                        needRefreshOutput = true;
                    }
                }
                cloud_realtime_last = c2t;
            }
        }

        if (capture)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c2s(new pcl::PointCloud<pcl::PointXYZRGBA>());
            (*c2s) = (*cloud);

            QString name = QString::fromStdString(path) + "\\" + QString::fromStdString(currentDateTime()) + ".pcd";
            cPrj->addCloud(c2s, name);
            cPrj->cIsSaved[cPrj->getCloudSize() - 1] = false;

            if (!track || cloud_iter == 0 || matL3 == Identity4f)
            {
                cloud_iter = c2s;
            }
            else
            {
                cPrj->addCorrespondence(cPrj->getCloudSize() - 2, cPrj->getCloudSize() - 1);
                cPrj->rMatrix[cPrj->getCorrespondenceSize() - 1] = matL3;
                cloud_iter = c2s;
                matL3 = Identity4f;
                std::cout << "New cloud and the iteration correspondence are added." << std::endl;
            }

            if (crop)
                cropCloud(c2s, 25);

            std::stringstream ss;
            ss << "One cloud captured : [" << cPrj->cPath[cPrj->getCloudSize() - 1].toStdString() << "]." << endl << "Total number of unsaved clouds : " << cPrj->getCloudSize() << ".";
            cloud_viewer_->updateText(ss.str(), 20, 20, 14, 0, 1, 1, "Status");
            trackLost = false;
            capture = false;
        }

        if (save)
        {
            for (int i = 0; i < cPrj->getCloudSize(); i++)
                if (cPrj->cIsSaved[i] == false)
                {
                    cPrj->savePointCloudInProject(i, cPrj->cPath[i]);
                    cPrj->cIsSaved[i] = true;
                }
            QString mark;
            cPrj->generateMark(mark);

            QFile file(cPrj->name);
            if (!file.open(QIODevice::WriteOnly))
            {
                cloud_viewer_->updateText("Cannot save the project file.", 20, 20, 14, 1, 0, 0, "Status");
                save = false;
                return;
            }

            QTextStream ts(&file);
            ts << mark;

            cloud_viewer_->updateText("Save completed.", 20, 20, 14, 1, 1, 0, "Status");
            save = false;
        }

        if (needRefreshOutput)
        {
            if (outputStream.str() != "")
                cloud_viewer_->updateText(outputStream.str(), 500, 20, 12, 0.6, 0.6, 0.6, "Para");
            needRefreshOutput = false;
        }
    }

    void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
    {
        if (event.getKeyCode() == ' ' && event.keyDown())
        {
            capture = true;
        }
        if (event.getKeyCode() == 13 && event.keyDown())
        {
            save = true;
        }
    }

    void mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
    {

    }

    void run()
    {
        cloud_viewer_->registerMouseCallback(&OpenNIViewer::mouse_callback, *this);
        cloud_viewer_->registerKeyboardCallback(&OpenNIViewer::keyboard_callback, *this);
        boost::function<void(const CloudConstPtr&)> cloud_cb = boost::bind(&OpenNIViewer::cloud_callback, this, _1);
        boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);
        bool cloud_init = false;

        grabber_.start();
        while (!cloud_viewer_->wasStopped())
        {
            CloudConstPtr cloud;
            cloud_viewer_->spinOnce();
            if (cloud_mutex_.try_lock())
            {
                cloud_.swap(cloud);
                cloud_mutex_.unlock();
            }
            if (cloud)
            {
                if (!cloud_init)
                {
                    cloud_viewer_->setPosition(150, 50);
                    cloud_viewer_->setSize(800, 600);
                    cloud_viewer_->setShowFPS(false);
                    cloud_init = !cloud_init;
                }
                if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud"))
                {
                    cloud_viewer_->addText("IRIS Scan Grabber\nOne Component of IRIS Scan Handler.\n\nUsage : \nPress <SPACE> to capture,\nPress <ENTER> to save the captured,\nPress <Q> to leave without saving.", 20, 20, 14, 1, 1, 1, "Status");
                    cloud_viewer_->addText(".", 500, 20, 8, 0, 0, 0, "Para");
                    cloud_viewer_->setCameraPosition(0, 0, -5, 0, 1, 0);
                    cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
                    cloud_viewer_->setWindowName("IRIS Scan Grabber");
                    trackLost = false;
                    shakingBefore = true;
                    needRefreshOutput = true;
                    cloudClock = 0;
                }
            }
        }
        grabber_.stop();
        cloud_connection.disconnect();
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    pcl::Grabber& grabber_;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
bool save = false, capture = false, crop = true, track = true;

std::string device_id("");
std::string path("");

int main(int argc, char **argv)
{
    if (argc >= 2)
    {
        path = argv[1];
        if (argc >= 3)
        {
            for (int i = 2; i < argc; i++)
            {
                std::string arg(argv[i]);
                if (arg == "NoCrop")
                    crop = false;
                else if (arg == "NoTrack")
                    track = false;
            }
        }
    }

    pcl::OpenNIGrabber::Mode image_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;
    pcl::OpenNIGrabber::Mode depth_mode = pcl::OpenNIGrabber::OpenNI_Default_Mode;

    openni_wrapper::OpenNIDriver& driver = openni_wrapper::OpenNIDriver::getInstance();
    if (driver.getNumberDevices() <= 0)
    {
        cout << "No device found. Exiting..." << endl;
        return 0;
    }

    QString name = QString::fromStdString(path) + "\\" + QString::fromStdString(currentDateTime()) + ".isproj";
    cPrj = new Project(name);

    pcl::OpenNIGrabber grabber(device_id, depth_mode, image_mode);
    OpenNIViewer<pcl::PointXYZRGBA> openni_viewer(grabber);
    openni_viewer.run();

    return 0;
}
