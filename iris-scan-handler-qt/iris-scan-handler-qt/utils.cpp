#include "stdafx.h"
#include "utils.h"

double Utils::getDistance(const OriPoint &p1, const OriPoint &p2)
{
    return sqrt((abs((double)p1.x - (double)p2.x) * abs((double)p1.x - (double)p2.x)) + (abs((double)p1.y - (double)p2.y) * abs((double)p1.y - (double)p2.y)) + (abs((double)p1.z - (double)p2.z) * abs((double)p1.z - (double)p2.z)));
}

int Utils::readPointClouds(const int &argc, char** &argv, std::vector<OriCloudPtr> &cloud_list)
{
    if (argc == 1)
    {
        pcl::console::print_error("    No point cloud to read. \n");
        pcl::console::print_info("    Usage : -[*.pcd](n)-\n");
        return 1;
    }
    for (int i = 1; i < argc; ++i)
    {
        OriCloudPtr cloud(new OriCloud);
        readSinglePointCloud(argv[i], cloud);
        cloud_list.push_back(cloud);
    }
    return 0;
}

int Utils::readSinglePointCloud(const std::string &path, const OriCloudPtr cloud)
{
    std::cout << "> " << path << std::endl;
    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));
    if (ext == "pcd" || ext == "PCD") //pcd
    {
        if (pcl::io::loadPCDFile<OriPoint>(path, *cloud) == -1)
        {
            pcl::console::print_error("    Failed to read the point cloud : %s .\n", path);
            return -1;
        }
    }
    else if (ext == "ply" || ext == "PLY") //ply
    {
        if (pcl::io::loadPLYFile<OriPoint>(path, *cloud) == -1)
        {
            pcl::console::print_error("    Failed to read the point cloud : %s .\n", path);
            return -1;
        }
    }
    return 0;
}

int Utils::readSinglePointCloud(const std::string &path, const pcl::PCLPointCloud2Ptr cloud)
{
    std::cout << "> " << path << std::endl;
    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));
    if (ext == "pcd" || ext == "PCD") //pcd
    {
        if (pcl::io::loadPCDFile(path, *cloud) == -1)
        {
            pcl::console::print_error("    Failed to read the point cloud : %s .\n", path);
            return -1;
        }
    }
    else if (ext == "ply" || ext == "PLY") //ply
    {
        if (pcl::io::loadPLYFile(path, *cloud) == -1)
        {
            pcl::console::print_error("    Failed to read the point cloud : %s .\n", path);
            return -1;
        }
    }
    return 0;
}

void Utils::savePointCloud(const OriCloudPtr &cloud, const std::string &path, const std::string type)
{
    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));

    if (type == "binary")
    {
        if (ext == "pcd" || ext == "PCD")
        {
            if (pcl::io::savePCDFileBinaryCompressed(path, *cloud) == 0)
            {
                std::cout << "Saved successfully." << std::endl;
            }
            else
            {
                std::cout << "Failed in saving the point cloud." << std::endl;
            }
        }
    }

    if (type == "ascii")
    {
        if (ext == "pcd" || ext == "PCD")
        {
            if (pcl::io::savePCDFileASCII(path, *cloud) == 0)
            {
                std::cout << "Saved successfully." << std::endl;
            }
            else
            {
                std::cout << "Failed in saving the point cloud." << std::endl;
            }
        }
    }

    if (ext == "ply" || ext == "PLY")
    {
        if (pcl::io::savePLYFileASCII(path, *cloud) == 0)
        {
            std::cout << "Saved successfully." << std::endl;
        }
        else
        {
            std::cout << "Failed in saving the point cloud." << std::endl;
        }
    }

    if (ext == "txt" || ext == "TXT")
    {
        if (saveTXTFile(path, *cloud) == 0)
        {
            std::cout << "Saved successfully." << std::endl;
        }
        else
        {
            std::cout << "Failed in saving the point cloud." << std::endl;
        }
    }
}

void Utils::savePointCloud(const pcl::PolygonMeshPtr polygonMesh, const std::string &path)
{
    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));
    if (ext == "ply" || ext == "PLY")
    {
        if (pcl::io::savePLYFile(path, *polygonMesh) == 0)
        {
            std::cout << "Saved successfully." << std::endl;
        }
        else
        {
            std::cout << "Failed in saving the point cloud." << std::endl;
        }
    }
}

int Utils::saveTXTFile(std::string path, OriCloud &cloud)
{
    std::stringstream buf;
    char chbuf[30];
    for (int i = 0; i < cloud.points.size(); i++)
    {
        sprintf(chbuf, "%.9f ", cloud.points[i].x);
        buf << chbuf;
        sprintf(chbuf, "%.9f ", cloud.points[i].y);
        buf << chbuf;
        sprintf(chbuf, "%.9f ", cloud.points[i].z);
        buf << chbuf;
        sprintf(chbuf, "%d ", cloud.points[i].r);
        buf << chbuf;
        sprintf(chbuf, "%d ", cloud.points[i].g);
        buf << chbuf;
        sprintf(chbuf, "%d ", cloud.points[i].b);
        buf << chbuf;
        buf << std::endl;
    }
    std::ofstream os(path);
    os << buf.str();
    os.close();
    std::cout << "Saved successfully." << std::endl;
    return 0;
}

int Utils::combinePointCloud(const OriCloudPtr &cloud1, const OriCloudPtr &cloud2, const OriCloudPtr &combinedCloud, float leaf_size)
{
    OriCloudPtr cloudp(new OriCloud());
    (*cloudp) = (*cloud1) + (*cloud2);
    downSamplePointCloud(cloudp, combinedCloud, leaf_size);
    std::cout << "Combined successfully." << std::endl;
    return 0;
}

int Utils::combinePointCloud(const std::vector<OriCloudPtr> clouds, const OriCloudPtr &combinedCloud, float leaf_size)
{
    OriCloudPtr cloudp(new OriCloud());
    for (auto i = clouds.begin(); i != clouds.end(); i++)
        (*cloudp) += (**i);
    downSamplePointCloud(cloudp, combinedCloud, leaf_size);
    return 0;
}

int Utils::downSamplePointCloud(const OriCloudPtr &cloud_in, const OriCloudPtr &cloud_out, float leaf_size)
{
    OriCloudPtr cloudp(new OriCloud());
    (*cloudp) = (*cloud_in);
    pcl::VoxelGrid<OriPoint> sor;
    sor.setInputCloud(cloudp);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_out);
    std::cout << "Downsampled completed. " << (*cloudp).points.size() << "->" << (*cloud_out).points.size() << std::endl;
    return 0;
}

int Utils::transformPointCloud(const OriCloudPtr &cloud_in, const OriCloudPtr &cloud_out, const Eigen::Matrix4f &transformMatrix)
{
    OriCloudPtr transformedCloudPtr(new OriCloud);
    pcl::transformPointCloud<OriPoint>((*cloud_in), (*cloud_out), transformMatrix);
    return 0;
}

int Utils::combineField(const OriCloudPtr &cloud_in, const NormalCloudPtr &cloud_to_combine)
{
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_to_combine->points[i].x = cloud_in->points[i].x;
        cloud_to_combine->points[i].y = cloud_in->points[i].y;
        cloud_to_combine->points[i].z = cloud_in->points[i].z;
        cloud_to_combine->points[i].r = cloud_in->points[i].r;
        cloud_to_combine->points[i].g = cloud_in->points[i].g;
        cloud_to_combine->points[i].b = cloud_in->points[i].b;
    }
    return 0;
}

int Utils::subtractField(const NormalCloudPtr &cloud_in, const OriCloudPtr &cloud_out)
{
    cloud_out->clear();
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        OriPoint pt;
        pt.x = cloud_in->points[i].x;
        pt.y = cloud_in->points[i].y;
        pt.z = cloud_in->points[i].z;
        pt.r = cloud_in->points[i].r;
        pt.g = cloud_in->points[i].g;
        pt.b = cloud_in->points[i].b;
        pt.a = cloud_in->points[i].a;
        cloud_out->points.push_back(pt);
    }
    return 0;
}

const double colorChart[] = {0, 0, 255, 127, 255, 0, 100, 149, 237, 220, 20, 60, 255, 140, 0, 255, 20, 147, 30, 144, 255, 173, 255, 47, 255, 0, 255, 255, 255, 0};

void Utils::getColorFromChart(const int index, double &r, double &g, double &b)
{
    int x = index % 10 * 3;
    r = colorChart[x] / 255.0;
    g = colorChart[x + 1] / 255.0;
    b = colorChart[x + 2] / 255.0;
    return;
}
