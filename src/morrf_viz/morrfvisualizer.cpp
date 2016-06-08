#include <QtGui>
#include "morrf_viz/morrfvisualizer.h"

MORRFVisualizer::MORRFVisualizer(QWidget *parent) :
    QLabel(parent) {
    mpMORRF = NULL;
    mCurrentTreeIdx = 0;
}

void MORRFVisualizer::setMORRF(MORRF* pMorrf) {
    mpMORRF = pMorrf;
}

void MORRFVisualizer::prevTree() {
    if(mpMORRF) {
        if(mCurrentTreeIdx > 0) {
            mCurrentTreeIdx --;
        }
        else {
            mCurrentTreeIdx = mMOPPInfo.mSubproblemNum+mMOPPInfo.mObjectiveNum-1;
        }
    }
}

void MORRFVisualizer::nextTree() {
    if(mpMORRF) {
        if( mCurrentTreeIdx < mMOPPInfo.mSubproblemNum+mMOPPInfo.mObjectiveNum-1 ) {
            mCurrentTreeIdx ++;
        }
        else {
            mCurrentTreeIdx = 0;
        }
    }
}

void MORRFVisualizer::paintEvent(QPaintEvent * e) {
    QLabel::paintEvent(e);

    if(mpMORRF) {
        RRTree * pTree = NULL;
        if( mCurrentTreeIdx < mMOPPInfo.mObjectiveNum ) {
            pTree = mpMORRF->get_reference_tree(mCurrentTreeIdx);
        }
        else {
            pTree = mpMORRF->get_subproblem_tree(mCurrentTreeIdx - mMOPPInfo.mObjectiveNum);
        }

        if( pTree ) {
            QPainter painter(this);
            QPen paintpen(QColor(0,255,0));
            paintpen.setWidth(1);
            painter.setPen(paintpen);

            for(std::list<RRTNode*>::iterator it= pTree->m_nodes.begin(); it!=pTree->m_nodes.end();it++) {
                RRTNode* pNode = (*it);
                if( pNode ) {
                    for( std::list<RRTNode*>::iterator itc= pNode->m_child_nodes.begin(); itc!=pNode->m_child_nodes.end(); itc++ ) {
                        RRTNode* pChildNode = (*itc);
                        if( pChildNode ) {
                            painter.drawLine(QPoint(pNode->m_pos[0], pNode->m_pos[1]), QPoint(pChildNode->m_pos[0], pChildNode->m_pos[1]));
                        }
                    }
                }
            }
            painter.drawText(QRect(10, height()-20, 60, 30),Qt::AlignLeft, "Index:"+QString::number(pTree->m_index));

            if( pTree->mp_current_best ) {
                QPen paintpen(QColor(255,140,0));
                paintpen.setStyle(Qt::DashLine);
                paintpen.setWidth(2);
                painter.setPen(paintpen);

                int point_num = pTree->mp_current_best->m_waypoints.size();

                if( point_num > 0) {
                    for( int i=0; i<point_num-1; i++) {
                        painter.drawLine(QPoint(pTree->mp_current_best->m_waypoints[i][0], pTree->mp_current_best->m_waypoints[i][1]),
                                         QPoint(pTree->mp_current_best->m_waypoints[i+1][0], pTree->mp_current_best->m_waypoints[i+1][1]));
                    }
                }
            }
        }

        /*
        if( mCurrentTreeIdx < mMOPPInfo.mFoundPaths.size() ) {
            Path * p = mMOPPInfo.mFoundPaths[mCurrentTreeIdx];
            QPainter painter(this);
            QPen paintpen(QColor(255,140,0));
            paintpen.setWidth(2);
            painter.setPen(paintpen);

            int point_num = p->m_waypoints.size();

            if( point_num > 0) {
                for( int i=0; i<point_num-1; i++) {
                    painter.drawLine(QPoint(p->m_waypoints[i][0], p->m_waypoints[i][1]), QPoint(p->m_waypoints[i+1][0], p->m_waypoints[i+1][1]));
                }
            }
        }*/
    }

    if( mMOPPInfo.mStart.x() >= 0 && mMOPPInfo.mStart.y() >= 0 ) {
        QPainter painter(this);
        QPen paintpen(QColor(255,0,0));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(mMOPPInfo.mStart);
    }

    if( mMOPPInfo.mGoal.x() >= 0 && mMOPPInfo.mGoal.y() >= 0 ) {
        QPainter painter(this);
        QPen paintpen(QColor(0,0,255));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(mMOPPInfo.mGoal);
    }
}
