#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>
#include <QJsonObject>
#include <list>
#include <vector>
#include <QDebug>
#include <math.h>

#include "morrf.h"

class MultiObjPathPlanningInfo {
public:
    MultiObjPathPlanningInfo();

    bool getObstacleInfo(int** obstacleInfo);
    std::vector<int**> getFitnessDistributions();

    bool getPixInfo(QString filename, int** pixInfo);
    void initFuncsParams();

    bool saveToFile(QString filename);
    bool loadFromFile(QString filename);

    void read(const QJsonObject &json);
    void write(QJsonObject &json) const;

    void loadPaths(std::vector<Path*> paths);
    void exportPaths(QString filename);

    static double calcDist(POS2D pos_a, POS2D pos_b, int** distribution) {
        double dist = 0.0;
        if (pos_a == pos_b)
            return dist;
        double delta_x = fabs(pos_a[0]-pos_b[0]);
        double delta_y = fabs(pos_a[1]-pos_b[1]);
        dist = sqrt(delta_x*delta_x+delta_y*delta_y);

        if(dist < 0.0) {
            qWarning() << "Dist negative " << dist ;
        }
        return dist;
    }

    static double calcCost(POS2D pos_a, POS2D pos_b, int** pp_distribution) {
        double cost = 0.0;
        if ( pos_a == pos_b ) {
            return cost;
        }
        if( pp_distribution == NULL ) {
            return cost;
        }

        float x1 = pos_a[0];
        float y1 = pos_a[1];
        float x2 = pos_b[0];
        float y2 = pos_b[1];

        const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
        if (steep) {
            std::swap(x1, y1);
            std::swap(x2, y2);
        }

        if (x1 > x2) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        const float dx = x2 - x1;
        const float dy = fabs(y2 - y1);

        float error = dx / 2.0f;
        const int ystep = (y1 < y2) ? 1 : -1;
        int y = (int)y1;

        const int maxX = (int)x2;

        for(int x=(int)x1; x<maxX; x++) {
            if(steep) {
                cost += pp_distribution[y][x];
            }
            else {
                cost += pp_distribution[x][y];
            }

            error -= dy;
            if(error < 0) {
                y += ystep;
                error += dx;
            }
        }

        return cost;
    }

    /* Member variables */
    QString mInfoFilename;
    QString mMapFilename;
    QString mMapFullpath;
    int mMapWidth;
    int mMapHeight;

    int mObjectiveNum;
    QPoint mStart;
    QPoint mGoal;

    bool mMinDistEnabled;
    std::vector<QString> mObjectiveFiles;

    std::vector<COST_FUNC_PTR> mFuncs;
    std::vector<int**>         mDistributions;

    int mSubproblemNum;
    int mMaxIterationNum;
    double mSegmentLength;

    MORRF::MORRF_TYPE mMethodType;

    std::vector<Path*> mFoundPaths;
};

#endif // MULTIOBJPATHPLANNINGINFO_H