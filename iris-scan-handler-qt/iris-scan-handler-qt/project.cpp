#include "project.h"
#include "utils.h"

#include "QStringList"
#include "QFileInfo"

Project::Project(QString _name)
{
    this->name = _name;
    this->isSaved = true;
}

int Project::addCloud(const OriCloudPtr &p, QString &name, bool forceCheckRepeat, bool addCopy)
{
    if (forceCheckRepeat)
    {
        for (int i = 0; i < this->getCloudSize(); i++)
        {
            if (name == this->cPath[i])
            {
                if (addCopy)
                {
                    name = name + "[Another]";
                    this->addCloud(p, name, true, true);
                    return 0;
                }
                else
                    return -1 * i;
            }
        }
    }
    this->cRaw.push_back(p);
    this->cPath.push_back(name);
    this->cIsSaved.push_back(false);
    this->cAttachMesh.push_back(-1);
    QList<int> keyPoints;
    this->cKeyPoints.push_back(keyPoints);
    this->isSaved = false;
    return this->cRaw.size();
}

int Project::addCorrespondence(int target, int source, bool isTopo, bool forceCheckRepeat)
{
    if (forceCheckRepeat)
    {
        for (int i = 0; i < this->getCorrespondenceSize(); i++)
        {
            if ((target == this->rTarget[i] && source == this->rSource[i]) || (source == this->rTarget[i] && target == this->rSource[i]))
            {
                if (this->rIsTopo[i])                       // The existed one is topologied.
                {
                    this->removeCorrespondence(i);          // Remove the existed one, else
                    break;
                }
                else
                {
                    return -1;                              // There already existed one correspondence. Abort.
                }
            }
        }
    }
    this->rTarget.push_back(target);
    this->rSource.push_back(source);
    Matrix4f mat = Matrix4f::Identity();
    this->rMatrix.push_back(mat);
    this->rIsTopo.push_back(isTopo);
    this->isSaved = false;
    return this->rSource.size();
}

int Project::addMesh(const int index)
{
    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
    this->sMesh.push_back(mesh);
    this->cAttachMesh[index] = this->sMesh.size() - 1;
    this->isSaved = false;
    return this->sMesh.size();
}

int Project::getCloudSize()
{
    return this->cRaw.size();
}

int Project::getCorrespondenceSize()
{
    return this->rSource.size();
}

int Project::getMeshSize()
{
    return this->sMesh.size();
}

int Project::getCorrespondenceIndex(int target, int source)
{
    if (target == source) return -1;
    for (int i = 0; i < this->getCorrespondenceSize(); i++)
    {
        if (this->rSource[i] == source && this->rTarget[i] == target)
            return i;
    }
    return -1;
}

void Project::removeCloud(int index)
{
    this->cPath.erase(this->cPath.begin() + index);
    this->cRaw.erase(this->cRaw.begin() + index);
    this->cKeyPoints.erase(this->cKeyPoints.begin() + index);
    this->cAttachMesh.erase(this->cAttachMesh.begin() + index);
    this->cIsSaved.erase(this->cIsSaved.begin() + index);
    for (int i = 0; i < this->getCorrespondenceSize(); i++)
    {
        if (this->rSource[i] == index || this->rTarget[i] == index)
            this->removeCorrespondence(i);
        if (this->rSource[i] > index)
            this->rSource[i]--;
        if (this->rTarget[i] > index)
            this->rTarget[i]--;
    }
    this->isSaved = false;
}

void Project::removeCorrespondence(int index)
{
    this->rTarget.erase(this->rTarget.begin() + index);
    this->rSource.erase(this->rSource.begin() + index);
    this->rMatrix.erase(this->rMatrix.begin() + index);
    this->rIsTopo.erase(this->rIsTopo.begin() + index);
    this->isSaved = false;
}

void Project::removeMesh(int index)
{
    if (this->cAttachMesh[index] != -1)
    {
        int cache = this->cAttachMesh[index];
        this->sMesh.erase(this->sMesh.begin() + cache);
        this->cAttachMesh[index] = -1;
        for (int i = 0; i < this->getCloudSize(); i++)
            if (this->cAttachMesh[i] > cache)
                this->cAttachMesh[i]--;
    }
}

void Project::generateMark(QString &output)
{
    this->removeAllTopoCorrespondences();
    output.clear();
    output += "# IRIS Scan Handler Project File\n";
    output += "VERSION|0.1\n";
    output += "CLOUDS|" + QString::number(this->getCloudSize()) + "\n";
    for (int i = 0; i < this->getCloudSize(); i++)
    {
        if (this->cIsSaved[i])
        {
            output += "CLOUD|" + this->cPath[i] + "|" + QString::number(this->cKeyPoints[i].size());
            for (int j = 0; j < this->cKeyPoints[i].size(); j++)
                output += "|" + QString::number(this->cKeyPoints[i][j]);
            output += "\n";
        }
    }
    output += "CORRESPONDNECES|" + QString::number(this->getCorrespondenceSize()) + "\n";
    for (int i = 0; i < this->getCorrespondenceSize(); i++)
    {
        output += "CORRESPONDENCE|" + QString::number(this->rTarget[i]) + "|" + QString::number(this->rSource[i]);
        for (int j = 0; j < 16; j++)
            output += "|" + QString::number(this->rMatrix[i](j));
        output += "\n";
    }
    output += "PROCCLOUDS|" + QString::number(this->procClouds.size());
    for (int i = 0; i < this->procClouds.size(); i++)
    {
        output += "|" + QString::number(this->procClouds[i]);
    }
    output += "\n";
}

void Project::rebuildTopoCorrespondences(int base)
{
    this->removeAllTopoCorrespondences();
    int size = this->getCorrespondenceSize();
    for (int i = 0; i < size; i++)
    {
        this->addCorrespondence(this->rSource[i], this->rTarget[i], true, false);
        this->rMatrix[this->getCorrespondenceSize() - 1] = this->rMatrix[i].inverse();
    }

    if (base != -1)
    {
        QList<int> queue;
        QList<Matrix4f> mat;
        int left = 0;
        queue.push_back(base);
        mat.push_back(Identity4f);
        while (left < queue.size())
        {
            for (int i = 0; i < this->getCorrespondenceSize(); i++)
                if (this->rTarget[i] == queue[left] && !(queue.contains(this->rSource[i])))
                {
                    queue.push_back(this->rSource[i]);
                    Matrix4f tmp = mat[left] * this->rMatrix[i];
                    mat.push_back(tmp);
                    if (this->getCorrespondenceIndex(base, *(queue.end() - 1)) == -1)
                    {
                        this->addCorrespondence(base, *(queue.end() - 1), true, false);
                        this->rMatrix[this->getCorrespondenceSize() - 1] = tmp;
                    }
                }
            left++;
        }
    }
}

const Matrix4f &Project::getTransformMatrix(int target, int source)
{
    int index = this->getCorrespondenceIndex(target, source);
    if (index >= 0) return this->rMatrix[index];
    else return Identity4f;
}

int Project::readMark(QString mark)
{
    if (mark[0] == '#') // Comments
        return 0;
    QStringList split = mark.split('|');
    if (split[0] == "VERSION")
    {
        double para = split[1].toDouble();
        if (para < ISProjMaxVersion)
        {
            return -6; // Version is too high
        }
    }
    else if (split[0] == "CLOUD")
    {
        OriCloudPtr cloud(new OriCloud());
        QFileInfo fi(split[1]);
        if (fi.exists())
        {
            Utils::readSinglePointCloud(split[1].toStdString(), cloud);
            this->addCloud(cloud, split[1]);
            this->cIsSaved[this->getCloudSize() - 1] = true;
            int para = split[2].toInt();
            for (int i = 0; i < para; i++)
                this->cKeyPoints[this->getCloudSize() - 1].push_back(split[3 + i].toInt());
        }
        else
        {
            return -1; // File not exist.
        }
    }
    else if (split[0] == "CORRESPONDENCE")
    {
        this->addCorrespondence(split[1].toInt(), split[2].toInt());
        for (int i = 0; i < 16; i++)
        {
            this->rMatrix[this->getCorrespondenceSize() - 1](i) = split[3 + i].toFloat();
        }
    }
    else if (split[0] == "PROCCLOUDS")
    {
        for (int i = 0; i < split[1].toInt(); i++)
        {
            this->procClouds.push_back(split[2 + i].toInt());
        }
    }
    return 0;
}

void Project::removeAllTopoCorrespondences()
{
    if (this->getCorrespondenceSize() < 1) return;
    int i = 0;
    do
    {
        if (this->rIsTopo[i])
            this->removeCorrespondence(i);
        else
            i++;
    }
    while (i != this->getCorrespondenceSize());
}
void Project::savePointCloudInProject(int index, QString path)
{
    Utils::savePointCloud(this->cRaw[index], path.toStdString());
    if (!this->cIsSaved[index])
        this->cPath[index] = path;
    this->cIsSaved[index] = true;
}
