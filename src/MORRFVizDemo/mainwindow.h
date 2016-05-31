#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QPixmap>
#include <QProgressBar>
#include "morrf_viz/morrfvisualizer.h"

class ConfigObjDialog;
class MORRF;

class MainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    MORRFVisualizer * mpViz;

    bool planPath(QString filename);
protected:
    void createMenuBar();
    void createActions();
    bool openMap(QString filename);

    void keyPressEvent(QKeyEvent *event);
    void updateStatus();

    bool setupPlanning(QString filename);
    bool exportPaths(QString filename);
    void initMORRF();

    bool loadConfiguration(QString filename);
    bool saveConfiguration(QString filename);

private:
    void updateTitle();

    QMenu*        mpFileMenu;
    QAction*      mpOpenAction;
    QAction*      mpSaveAction;
    QAction*      mpExportAction;
    QMenu*        mpEditMenu;
    QAction*      mpLoadMapAction;
    QAction*      mpLoadObjAction;
    QAction*      mpRunAction;
    QMenu*        mpContextMenu;
    QAction*      mpAddStartAction;
    QAction*      mpAddGoalAction;
    QLabel*       mpStatusLabel;
    QProgressBar* mpStatusProgressBar;
    QPixmap*      mpMap;

    QPoint mCursorPoint;

    ConfigObjDialog * mpConfigObjDialog;
    MORRF           * mpMORRF;


private slots:
    void contextMenuRequested(QPoint point);
    void onOpen();
    void onSave();
    void onExport();
    void onLoadMap();
    void onLoadObj();
    void onRun();
    void onAddStart();
    void onAddGoal();
};

#endif // MAINWINDOW_H
