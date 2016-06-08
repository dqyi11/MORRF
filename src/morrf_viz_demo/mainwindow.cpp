#include "mainwindow.h"
#include <QFileDialog>
#include <configobjdialog.h>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>
#include <QApplication>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) {
    mpViz = new MORRFVisualizer();

    createActions();
    createMenuBar();

    mpMap = NULL;
    mpMORRF = NULL;

    mpConfigObjDialog = new ConfigObjDialog(this);
    mpConfigObjDialog->hide();
    setCentralWidget(mpViz);

    mpStatusLabel = new QLabel();
    mpStatusProgressBar = new QProgressBar();
    statusBar()->addWidget(mpStatusProgressBar);
    statusBar()->addWidget(mpStatusLabel);

    updateTitle();
}

MainWindow::~MainWindow() {
    if(mpConfigObjDialog) {
        delete mpConfigObjDialog;
        mpConfigObjDialog = NULL;
    }
    if(mpViz) {
        delete mpViz;
        mpViz = NULL;
    }
}

void MainWindow::createMenuBar() {
    mpFileMenu = menuBar()->addMenu("&File");
    mpFileMenu->addAction(mpOpenAction);
    mpFileMenu->addAction(mpSaveAction);
    mpFileMenu->addAction(mpExportAction);

    mpEditMenu = menuBar()->addMenu("&Edit");
    mpEditMenu->addAction(mpLoadMapAction);
    mpEditMenu->addAction(mpLoadObjAction);
    mpEditMenu->addAction(mpRunAction);

    mpContextMenu = new QMenu();
    setContextMenuPolicy(Qt::CustomContextMenu);

    mpContextMenu->addAction(mpAddStartAction);
    mpContextMenu->addAction(mpAddGoalAction);
}

void MainWindow::createActions() {
    mpOpenAction = new QAction("Open", this);
    mpSaveAction = new QAction("Save", this);
    mpExportAction = new QAction("Export", this);
    mpLoadMapAction = new QAction("Load Map", this);
    mpLoadObjAction = new QAction("Load Objectives", this);
    mpRunAction = new QAction("Run", this);

    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
    connect(mpExportAction, SIGNAL(triggered()), this, SLOT(onExport()));
    connect(mpLoadMapAction, SIGNAL(triggered()), this, SLOT(onLoadMap()));
    connect(mpLoadObjAction, SIGNAL(triggered()), this, SLOT(onLoadObj()));
    connect(mpRunAction, SIGNAL(triggered()), this, SLOT(onRun()));

    mpAddStartAction = new QAction("Add Start", this);
    mpAddGoalAction = new QAction("Add Goal", this);

    connect(mpAddStartAction, SIGNAL(triggered()), this, SLOT(onAddStart()));
    connect(mpAddGoalAction, SIGNAL(triggered()), this, SLOT(onAddGoal()));
    connect(this, SIGNAL(customContextMenuRequested(const QPoint)),this, SLOT(contextMenuRequested(QPoint)));
}

void MainWindow::onOpen() {
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("Json Files (*.json)"));
    if( loadConfiguration( tempFilename ) ) {
        openMap(mpViz->mMOPPInfo.mMapFullpath);
        if(mpConfigObjDialog) {
            mpConfigObjDialog->updateDisplay();
        }
        repaint();
    }
}

void MainWindow::onSave() {
    QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Json Files (*.json)"));
    saveConfiguration( tempFilename );
}

void MainWindow::onExport() {
    QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));
    exportPaths(pathFilename);
}

void MainWindow::onLoadMap() {
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open Map File"), "./", tr("Map Files (*.*)"));

    QFileInfo fileInfo(tempFilename);
    QString filename(fileInfo.fileName());
    mpViz->mMOPPInfo.mMapFilename = filename;
    mpViz->mMOPPInfo.mMapFullpath = tempFilename;
    qDebug("OPENING ");
    qDebug(mpViz->mMOPPInfo.mMapFilename.toStdString().c_str());

    openMap(mpViz->mMOPPInfo.mMapFullpath);
}


bool MainWindow::openMap(QString filename) {
    if(mpMap) {
        delete mpMap;
        mpMap = NULL;
    }
    mpMap = new QPixmap(filename);
    if(mpMap) {
        mpViz->mMOPPInfo.mMapWidth = mpMap->width();
        mpViz->mMOPPInfo.mMapHeight = mpMap->height();
        mpViz->setPixmap(*mpMap);
        updateTitle();
        return true;
    }
    return false;
}

void MainWindow::onLoadObj() {
    mpConfigObjDialog->exec();
    updateTitle();
}

void MainWindow::onRun() {
    if (mpViz->mMOPPInfo.mMapWidth <= 0 || mpViz->mMOPPInfo.mMapHeight <= 0) {
        QMessageBox msgBox;
        msgBox.setText("Map is not initialized.");
        msgBox.exec();
        return;
    }
    if (mpViz->mMOPPInfo.mObjectiveNum < 2) {
        QMessageBox msgBox;
        msgBox.setText("Objective Number is less than 2.");
        msgBox.exec();
        return;
    }
    if(mpViz->mMOPPInfo.mStart.x()<0 || mpViz->mMOPPInfo.mStart.y()<0) {
        QMessageBox msgBox;
        msgBox.setText("Start is not set.");
        msgBox.exec();
        return;
    }
    if(mpViz->mMOPPInfo.mGoal.x()<0 || mpViz->mMOPPInfo.mGoal.y()<0) {
        QMessageBox msgBox;
        msgBox.setText("Goal is not set.");
        msgBox.exec();
        return;
    }
    if(mpMORRF) {
        delete mpMORRF;
        mpMORRF = NULL;
    }

    mpViz->mMOPPInfo.initObstacleInfo();
    mpViz->mMOPPInfo.initFuncsParams();
    QString msg = "RUNNING MORRF ... \n";
    msg += "ObjNum( " + QString::number(mpViz->mMOPPInfo.mObjectiveNum) + " ) \n";
    msg += "SubproblemNum( " + QString::number(mpViz->mMOPPInfo.mSubproblemNum) + " ) \n";
    msg += "SegmentLen( " + QString::number(mpViz->mMOPPInfo.mSegmentLength) + " ) \n";
    msg += "MaxIterationNum( " + QString::number(mpViz->mMOPPInfo.mMaxIterationNum) + " ) \n";
    qDebug(msg.toStdString().c_str());

    mpMORRF = new MORRF(mpMap->width(), mpMap->height(), mpViz->mMOPPInfo.mObjectiveNum, mpViz->mMOPPInfo.mSubproblemNum, mpViz->mMOPPInfo.mSegmentLength, mpViz->mMOPPInfo.mMethodType);

    mpMORRF->add_funcs(mpViz->mMOPPInfo.mFuncs, mpViz->mMOPPInfo.mDistributions);
    POS2D start(mpViz->mMOPPInfo.mStart.x(), mpViz->mMOPPInfo.mStart.y());
    POS2D goal(mpViz->mMOPPInfo.mGoal.x(), mpViz->mMOPPInfo.mGoal.y());

    mpMORRF->init(start, goal);

    //mpViz->mMOPPInfo.dumpObstacleInfo("map1.txt");
    mpMORRF->load_map(mpViz->mMOPPInfo.mppObstacle);
    mpViz->setMORRF(mpMORRF);

    //mpMORRF->dump_map_info("map.txt");
    mpMORRF->dump_weights("weights.txt");

    while(mpMORRF->get_current_iteration() <= mpViz->mMOPPInfo.mMaxIterationNum) {
        QString msg = "CurrentIteration " + QString::number(mpMORRF->get_current_iteration()) + " ";
        msg += "(" + QString::number(mpMORRF->get_ball_radius()) + ")";
        qDebug(msg.toStdString().c_str());

        mpMORRF->extend();

        QApplication::processEvents();

        updateStatus();
        repaint();
    }

    std::vector<Path*> paths = mpMORRF->get_paths();
    mpViz->mMOPPInfo.loadPaths(paths);
    repaint();
}

void MainWindow::onAddStart() {
    mpViz->mMOPPInfo.mStart = mCursorPoint;
    repaint();
}

void MainWindow::onAddGoal() {
    mpViz->mMOPPInfo.mGoal = mCursorPoint;
    repaint();
}

void MainWindow::contextMenuRequested(QPoint point) {
    mCursorPoint = point;
    mpContextMenu->popup(mapToGlobal(point));
}

void MainWindow::updateTitle() {
    QString title = "ObjNum( " + QString::number(mpViz->mMOPPInfo.mObjectiveNum) + " )";
    title += " ==> " + mpViz->mMOPPInfo.mMapFilename;
    setWindowTitle(title);
}

void MainWindow::updateStatus() {
    if(mpViz==NULL || mpMORRF==NULL) {
        return;
    }
    if(mpStatusProgressBar) {
        mpStatusProgressBar->setMinimum(0);
        mpStatusProgressBar->setMaximum(mpViz->mMOPPInfo.mMaxIterationNum);
        mpStatusProgressBar->setValue(mpMORRF->get_current_iteration());
    }
    if(mpStatusLabel) {
        QString status = "";
        status += "TreeIdx: " + QString::number(mpViz->getCurrentTreeIndex());
        mpStatusLabel->setText(status);
    }
    repaint();
}

void MainWindow::keyPressEvent(QKeyEvent *event) {
    if(event->key() == Qt::Key_Right) {
        if(mpViz) {
            mpViz->nextTree();
            updateStatus();
            repaint();
        }
    }
    else if(event->key() == Qt::Key_Left) {
        if(mpViz) {
            mpViz->prevTree();
            updateStatus();
            repaint();
        }
    }
}

bool MainWindow::setupPlanning(QString filename) {
    if(mpViz) {
        mpViz->mMOPPInfo.loadFromFile(filename);
        return true;
    }
    return false;
}

bool MainWindow::exportPaths(QString filename) {
    if(mpViz) {
        mpViz->mMOPPInfo.exportPaths(filename);
        return true;
    }
    return false;
}

bool MainWindow::planPath(QString filename) {

    if( true == setupPlanning(filename) ) {

        initMORRF();
        while(mpMORRF->get_current_iteration() <= mpViz->mMOPPInfo.mMaxIterationNum) {
            mpMORRF->extend();
        }

        QString output_filename = filename + ".txt";
        if( true == exportPaths(output_filename) ) {
            return true;
        }
    }
    return false;
}

void MainWindow::initMORRF() {
    if(mpMORRF) {
        delete mpMORRF;
        mpMORRF = NULL;
    }
    mpViz->mMOPPInfo.initObstacleInfo();
    mpViz->mMOPPInfo.initFuncsParams();
    QString msg = "RUNNING MORRF ... \n";
    msg += "ObjNum( " + QString::number(mpViz->mMOPPInfo.mObjectiveNum) + " ) \n";
    msg += "SubproblemNum( " + QString::number(mpViz->mMOPPInfo.mSubproblemNum) + " ) \n";
    msg += "SegmentLen( " + QString::number(mpViz->mMOPPInfo.mSegmentLength) + " ) \n";
    msg += "MaxIterationNum( " + QString::number(mpViz->mMOPPInfo.mMaxIterationNum) + " ) \n";
    qDebug(msg.toStdString().c_str());

    mpMORRF = new MORRF(mpMap->width(), mpMap->height(), mpViz->mMOPPInfo.mObjectiveNum, mpViz->mMOPPInfo.mSubproblemNum, mpViz->mMOPPInfo.mSegmentLength, mpViz->mMOPPInfo.mMethodType);
    mpMORRF->add_funcs(mpViz->mMOPPInfo.mFuncs, mpViz->mMOPPInfo.mDistributions);
    POS2D start(mpViz->mMOPPInfo.mStart.x(), mpViz->mMOPPInfo.mStart.y());
    POS2D goal(mpViz->mMOPPInfo.mGoal.x(), mpViz->mMOPPInfo.mGoal.y());

    mpMORRF->init(start, goal);
    mpMORRF->load_map(mpViz->mMOPPInfo.mppObstacle);
    mpViz->setMORRF(mpMORRF);
}

bool MainWindow::loadConfiguration(QString filename) {
    if(mpViz) {
        return mpViz->mMOPPInfo.loadFromFile(filename);
    }
    return false;
}

bool MainWindow::saveConfiguration(QString filename) {
    if(mpViz) {
        return mpViz->mMOPPInfo.saveToFile(filename);
    }
    return false;
}
