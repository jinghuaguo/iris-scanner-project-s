#include "utils.h"

double Utils::getDistance(const Point &p1, const Point &p2)
{
    return sqrt((abs((double)p1.x - (double)p2.x) * abs((double)p1.x - (double)p2.x)) + (abs((double)p1.y - (double)p2.y) * abs((double)p1.y - (double)p2.y)) + (abs((double)p1.z - (double)p2.z) * abs((double)p1.z - (double)p2.z)));
}

int Utils::readPointClouds(const int &count, char** &paths, std::vector<CloudPtr> &cloud_list)
{
    for (int i = 1; i < count; ++i)
    {
        CloudPtr cloud(new Cloud);
        readSinglePointCloud(paths[i], cloud);
        cloud_list.push_back(cloud);
    }
    return 0;
}

int Utils::readSinglePointCloud(const std::string &path, const CloudPtr cloud)
{
    if (path.empty())
        return -6;

    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));
    if (ext == "pcd" || ext == "PCD")
    {
        if (pcl::io::loadPCDFile<Point>(path, *cloud) == -1)
            return -1;
    }
    else if (ext == "ply" || ext == "PLY")
    {
        if (pcl::io::loadPLYFile<Point>(path, *cloud) == -1)
            return -1;
    }
    return 0;
}

int Utils::readSinglePointCloud(const std::string &path, const pcl::PCLPointCloud2Ptr cloud)
{
    if (path.empty())
        return -6;

    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));
    if (ext == "pcd" || ext == "PCD")
    {
        if (pcl::io::loadPCDFile(path, *cloud) == -1)
            return -1;
    }
    else if (ext == "ply" || ext == "PLY")
    {
        if (pcl::io::loadPLYFile(path, *cloud) == -1)
            return -1;
    }
    return 0;
}

int Utils::savePointCloud(const CloudPtr &cloud, const std::string &path)
{
    if (path.empty())
        return -6;

    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));

    if (ext == "pcd" || ext == "PCD")
    {
        if (pcl::io::savePCDFileBinaryCompressed(path, *cloud) != 0)
            return -1;
    }

    if (ext == "ply" || ext == "PLY")
    {
        if (pcl::io::savePLYFileASCII(path, *cloud) != 0)
            return -1;
    }

    if (ext == "txt" || ext == "TXT")
    {
        if (saveTXTFile(path, *cloud) != 0)
            return -1;
    }

    return 0;
}

int Utils::savePointCloud(const pcl::PolygonMeshPtr polygonMesh, const std::string &path)
{
    if (path.empty())
        return -6;

    std::string ext(path.substr(path.rfind('.') == std::string::npos ? path.length() : path.rfind('.') + 1));

    if (ext == "ply" || ext == "PLY")
    {
        if (pcl::io::savePLYFile(path, *polygonMesh) != 0)
            return -1;
    }
    else
        return -6;

    return 0;
}

int Utils::saveTXTFile(std::string path, Cloud &cloud)
{
    if (path.empty())
        return -6;

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
    return 0;
}

int Utils::combinePointCloud(const CloudPtr &cloud1, const CloudPtr &cloud2, const CloudPtr &combinedCloud, float leaf_size)
{
    CloudPtr cloudp(new Cloud());
    (*cloudp) = (*cloud1) + (*cloud2);
    if (leaf_size > 0)
        downSamplePointCloud(cloudp, combinedCloud, leaf_size);
    else
        (*combinedCloud) = (*cloudp);
    return 0;
}

int Utils::combinePointCloud(const std::vector<CloudPtr> clouds, const CloudPtr &combinedCloud, float leaf_size)
{
    CloudPtr cloudp(new Cloud());
    for (auto i = clouds.begin(); i != clouds.end(); i++)
        (*cloudp) += (**i);
    if (leaf_size > 0)
        downSamplePointCloud(cloudp, combinedCloud, leaf_size);
    else
        (*combinedCloud) = (*cloudp);
    return 0;
}

int Utils::downSamplePointCloud(const CloudPtr &cloud_in, const CloudPtr &cloud_out, float leaf_size)
{
    CloudPtr cloudp(new Cloud());
    (*cloudp) = (*cloud_in);
    pcl::VoxelGrid<Point> sor;
    sor.setInputCloud(cloudp);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_out);
    std::cout << (*cloudp).points.size() << "->" << (*cloud_out).points.size() << std::endl;
    return 0;
}

int Utils::transformPointCloud(const CloudPtr &cloud_in, const CloudPtr &cloud_out, const Eigen::Matrix4f &transformMatrix)
{
    CloudPtr transformedCloudPtr(new Cloud);
    pcl::transformPointCloud<Point>((*cloud_in), (*cloud_out), transformMatrix);
    return 0;
}

int Utils::combineField(const CloudPtr &cloud_in, const NormalCloudPtr &cloud_out)
{
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_out->points[i].x = cloud_in->points[i].x;
        cloud_out->points[i].y = cloud_in->points[i].y;
        cloud_out->points[i].z = cloud_in->points[i].z;
        cloud_out->points[i].r = cloud_in->points[i].r;
        cloud_out->points[i].g = cloud_in->points[i].g;
        cloud_out->points[i].b = cloud_in->points[i].b;
    }
    return 0;
}

int Utils::subtractField(const NormalCloudPtr &cloud_in, const CloudPtr &cloud_out)
{
    cloud_out->clear();
    for (int i = 0; i < cloud_in->points.size(); i++)
    {
        Point pt;
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

void Utils::getColorFromColorChart(const int index, double &r, double &g, double &b)
{

    const double colorChart[] = {0, 0, 255, 127, 255, 0, 100, 149, 237, 220, 20, 60, 255, 140, 0, 255, 20, 147, 30, 144, 255, 173, 255, 47, 255, 0, 255, 255, 255, 0};
    int x = index % 10 * 3;
    r = colorChart[x] / 255.0;
    g = colorChart[x + 1] / 255.0;
    b = colorChart[x + 2] / 255.0;
    return;
}

double Utils::getColorChartElement(int index)
{
    const double colorChart[] = {0, 0, 255, 127, 255, 0, 100, 149, 237, 220, 20, 60, 255, 140, 0, 255, 20, 147, 30, 144, 255, 173, 255, 47, 255, 0, 255, 255, 255, 0};
    return colorChart[index];
}
