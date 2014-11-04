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
    }

    void cloud_callback(const CloudConstPtr& cloud)
    {
        boost::mutex::scoped_lock lock(cloud_mutex_);
        cloud_ = cloud;
        if (capture)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c2s(new pcl::PointCloud<pcl::PointXYZRGBA>());
            (*c2s) = (*cloud);
            if (crop)
                cropCloud(c2s, 25);

            QString name = QString::fromStdString(path) + "\\" + QString::fromStdString(currentDateTime()) + ".pcd";
            cPrj->addCloud(c2s, name);
            cPrj->cIsSaved[cPrj->getCloudSize() - 1] = false;
            std::stringstream ss;
            ss << "One cloud captured : [" << cPrj->cPath[cPrj->getCloudSize() - 1].toStdString() << "]." << endl << "Total number of unsaved clouds : " << cPrj->getCloudSize() << ".";
            cloud_viewer_->updateText(ss.str(), 20, 20, 14, 0, 1, 1, "Status");
            capture = false;
        }
        if (save)
        {
            for (int i = 0; i < cPrj->getCloudSize(); i++)
                if (cPrj->cIsSaved[i] == false)
                {
                    cPrj->savePointCloudInProject(i, cPrj->cPath[i]);
                    cPrj->cIsSaved[i] == true;
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
                    cloud_init = !cloud_init;
                }
                if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud"))
                {
                    cloud_viewer_->addText("IRIS Scan Grabber\nOne Component of IRIS Scan Handler.\n\nUsage : \nPress <SPACE> to capture,\nPress <ENTER> to save the captured,\nPress <Q> to leave without saving.", 20, 20, 14, 1, 1, 1, "Status");
                    cloud_viewer_->setCameraPosition(0, 0, -5, 0, 1, 0);
                    cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
                    cloud_viewer_->setWindowName("IRIS Scan Grabber");
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
