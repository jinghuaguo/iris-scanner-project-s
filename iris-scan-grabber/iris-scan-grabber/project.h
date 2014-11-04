#ifndef PROJECT_H
#define PROJECT_H

#include "stdafx.h"
#include "QString"
#include "QList"

class Project
{
public:
    //Logic
    QList<int> selClouds;
    QList<int> selCorrs;
    QList<int> procClouds;

    //Clouds
    QList<OriCloudPtr> cRaw;
    QList<QString> cPath;
    QList<QList<int>> cKeyPoints;
    QList<bool> cIsSaved;
    QList<int> cAttachMesh;

    //Correspondences
    QList<int> rSource, rTarget;
    QList<Matrix4f> rMatrix;
    QList<bool> rIsTopo;

    //Surface
    QList<pcl::PolygonMeshPtr> sMesh;
    QList<pcl::PCLPointCloud2Ptr> sCloud;

    //Props
    QString name;
    bool isSaved;

    Project(QString _name);
    int addCloud(const OriCloudPtr &p, QString &name, bool forceCheckRepeat = true, bool addCopy = true);
    int addCorrespondence(int target, int source, bool isTopo = false, bool forceCheckRepeat = true);
    int addMesh(const int index);
    int getCloudSize();
    int getCorrespondenceSize();
    int getMeshSize();
    void removeAllTopoCorrespondences();
    int getCorrespondenceIndex(int target, int source);
    void removeCloud(int index);
    void removeCorrespondence(int index);
    void removeMesh(int index);
    void generateMark(QString &output);
    void rebuildTopoCorrespondences(int base = -1);
    const Matrix4f &getTransformMatrix(int target, int source);
    void savePointCloudInProject(int index, QString path);

};

#endif // PROJECT_H
