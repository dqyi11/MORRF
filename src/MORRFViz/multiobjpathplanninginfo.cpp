#include <iostream>
#include <fstream>
#include <QPixmap>
#include <QJsonArray>
#include <QFile>
#include <QJsonDocument>
#include <list>
#include "multiobjpathplanninginfo.h"

MultiObjPathPlanningInfo::MultiObjPathPlanningInfo() {
    mInfoFilename = "";
    mMapFilename = "";
    mMapFullpath = "";
    mObjectiveNum = 0;

    mStart.setX(-1);
    mStart.setY(-1);
    mGoal.setX(-1);
    mGoal.setY(-1);

    mMinDistEnabled = false;

    mSubproblemNum = 4;
    mMaxIterationNum = 100;
    mSegmentLength = 5.0;

    mMapWidth = 0;
    mMapHeight = 0;

    mppObstacle = NULL;

    mMethodType = MORRF::WEIGHTED_SUM;
}

bool MultiObjPathPlanningInfo::initObstacleInfo() {
    mppObstacle = new int*[mMapWidth];
    for(int i=0;i<mMapWidth;i++) {
        mppObstacle[i] = new int[mMapHeight];
        for(int j=0;j<mMapHeight;j++) {
            mppObstacle[i][j] = 255;
        }
    }
    bool ret = getPixInfo(mMapFullpath, mppObstacle);
    //dumpObstacleInfo("testMap.txt");
    return ret;
}

bool MultiObjPathPlanningInfo::dumpObstacleInfo( QString filename ) {
    if( mppObstacle == NULL ) {
        return false;
    }
    std::ofstream mapInfoFile;
    mapInfoFile.open(filename.toStdString().c_str());
    for( int i=0; i<mMapWidth; i++ ) {
        for( int j=0; j<mMapHeight; j++ ) {
            mapInfoFile << mppObstacle[i][j] << " ";
        }
        mapInfoFile << std::endl;
    }
    mapInfoFile.close();
    return true;
}

std::vector<int**> MultiObjPathPlanningInfo::getFitnessDistributions() {
    std::vector<int**> fitnessDistributions;
    for(std::vector<QString>::iterator it=mObjectiveFiles.begin();it!=mObjectiveFiles.end();it++)  {
        QString fitnessName = (*it);
        int** fitness = new int*[mMapWidth];
        for(int i=0;i<mMapWidth;i++) {
            fitness[i] = new int[mMapHeight];
            for(int j=0;j<mMapHeight;j++) {
                fitness[i][j] = 0;
            }
        }
        bool sucess = getPixInfo(fitnessName, fitness);
        fitnessDistributions.push_back(fitness);
    }
    return fitnessDistributions;
}

bool MultiObjPathPlanningInfo::getPixInfo(QString filename, int**& pixInfo) {
    if(pixInfo==NULL) {
        return false;
    }
    QPixmap map(filename);
    //qDebug() << "GET PIX INFO " << filename;
    QImage grayImg = map.toImage();
    int width = map.width();
    int height = map.height();

    for(int i=0;i<width;i++) {
        for(int j=0;j<height;j++) {
            QRgb col = grayImg.pixel(i,j);
            int gVal = qGray(col);
            if(gVal < 0 || gVal > 255) {
                qWarning() << "gray value out of range";
            }/* else if ( gVal >= 0 && gVal < 255 ) {
                qDebug() << " val " << gVal;
            } */
            pixInfo[i][j] = gVal;
        }
    }
    return true;
}

void MultiObjPathPlanningInfo::initFuncsParams() {

    mFuncs.clear();
    mDistributions.clear();
    std::vector<int**> fitnessDistributions = getFitnessDistributions();

    if(mMinDistEnabled==true) {
        mFuncs.push_back(MultiObjPathPlanningInfo::calcDist);
        mDistributions.push_back(NULL);

        for(int k=0;k<mObjectiveNum-1;k++) {
            mFuncs.push_back(MultiObjPathPlanningInfo::calcCost);
            mDistributions.push_back(fitnessDistributions[k]);
        }
    }
    else {
        for(int k=0;k<mObjectiveNum;k++) {
            mFuncs.push_back(MultiObjPathPlanningInfo::calcCost);
            mDistributions.push_back(fitnessDistributions[k]);
        }
    }
}

void MultiObjPathPlanningInfo::read(const QJsonObject &json) {
    mInfoFilename;
    mMapFilename = json["mapFilename"].toString();
    mMapFullpath = json["mapFullpath"].toString();
    mMapWidth = json["mapWidth"].toInt();
    mMapHeight = json["mapHeight"].toInt();

    mMethodType = (MORRF::MORRF_TYPE)json["methodType"].toInt();

    mObjectiveNum = json["objectiveNum"].toInt();
    mStart = QPoint(json["startX"].toInt(), json["startY"].toInt());
    mGoal = QPoint(json["goalX"].toInt(), json["goalY"].toInt());

    mMinDistEnabled = json["minDistEnabled"].toBool();
    mObjectiveFiles.clear();
    QJsonArray objArray = json["objectiveFiles"].toArray();
    for(int objIdx = 0; objIdx < objArray.size(); objIdx++) {
        QJsonObject objObject = objArray[objIdx].toObject();
        QString objFile = objObject["filepath"].toString();
        mObjectiveFiles.push_back(objFile);
    }

    mSubproblemNum = json["subproblemNum"].toInt();
    mMaxIterationNum = json["maxIterationNum"].toInt();
    mSegmentLength = json["segmentLength"].toDouble();
}

void MultiObjPathPlanningInfo::write(QJsonObject &json) const {
    json["mapFilename"] = mMapFilename;
    json["mapFullpath"] = mMapFullpath;
    json["mapWidth"] = mMapWidth;
    json["mapHeight"] = mMapHeight;
    json["methodType"] = (int)mMethodType;

    json["objectiveNum"] = mObjectiveNum;
    json["startX"] = mStart.x();
    json["startY"] = mStart.y();
    json["goalX"] = mGoal.x();
    json["goalY"] = mGoal.y();

    json["minDistEnabled"] = mMinDistEnabled;
    QJsonArray objArray;
    for(unsigned int i=0;i<mObjectiveFiles.size();i++) {
        QString filepath = mObjectiveFiles[i];
        QJsonObject objObject;
        objObject["filepath"] = filepath;
        objArray.append(objObject);
    }
    json["objectiveFiles"] = objArray;

    json["subproblemNum"] = mSubproblemNum;
    json["maxIterationNum"] = mMaxIterationNum;
    json["segmentLength"] = mSegmentLength;
}

bool MultiObjPathPlanningInfo::saveToFile( QString filename ) {
    QFile saveFile(filename);

    if( false == saveFile.open(QIODevice::WriteOnly) ) {
        qWarning("Couldn't open file.");
        return false;
    }

    QJsonObject infoObject;
    write(infoObject);
    QJsonDocument saveDoc(infoObject);
    saveFile.write(saveDoc.toJson());
    return true;

}

bool MultiObjPathPlanningInfo::loadFromFile( QString filename ) {
    QFile loadFile(filename);

    if( false==loadFile.open(QIODevice::ReadOnly) ) {
        qWarning("Couldn't open file.");
        return false;
    }

    QByteArray saveData = loadFile.readAll();
    QJsonDocument loadDoc = QJsonDocument::fromJson( saveData );
    read(loadDoc.object());
    return true;
}

void MultiObjPathPlanningInfo::loadPaths( std::vector<Path*> paths ) {
    mFoundPaths.clear();
    for(std::vector<Path*>::iterator it=paths.begin(); it!=paths.end(); it++) {
        Path* p = *it;
        mFoundPaths.push_back(p);
    }
}

void MultiObjPathPlanningInfo::exportPaths( QString filename ) {
    QFile file(filename);
    if( file.open(QIODevice::ReadWrite) )  {
        QTextStream stream( & file );
        // Save scores
        for( std::vector<Path*>::iterator it=mFoundPaths.begin(); it!=mFoundPaths.end(); it++ ) {
            Path* p = *it;
            for(unsigned int k=0;k<mObjectiveNum;k++) {
                stream << p->m_cost[k] << "\t";
            }
            stream << "\n";
        }
        stream << "\n";
        // Save paths
        for( std::vector<Path*>::iterator it=mFoundPaths.begin(); it!=mFoundPaths.end(); it++ ) {
            Path* p = *it;
            for(unsigned int i=0;i<p->m_waypoints.size();i++) {
                stream << p->m_waypoints[i][0] << " " << p->m_waypoints[i][1] << "\t";
            }
            stream << "\n";
        }
    }
}
