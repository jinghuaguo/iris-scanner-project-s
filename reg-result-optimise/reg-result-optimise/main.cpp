#include "utils.h"
#include "typedef.h"
#include "visualization.h"

#include "vector"
#include "iostream"

int main(int argc, char *argv[])
{
    argc = 2;
    argv = new char*[argc];
    argv[0] = "D:\\PCDs\\20140817\\0003.pcd";
    argv[1] = "D:\\PCDs\\20140817\\0001.pcd";
    std::vector <CloudPtr> clouds;
    int *returnVal = new int[1];
    (*returnVal) = Utils::readPointClouds(argc, argv, clouds);
    if ((*returnVal) != 0)
        std::cerr << "An error occoured." << (*returnVal) << std::endl;
    std::cout << clouds.size() << " cloud(s) have been loaded." << std::endl;
    Visualization::showMultiClouds(clouds);
    delete returnVal;
}
