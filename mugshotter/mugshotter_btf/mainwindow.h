#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include <stdio.h>
#include <QtCore/QCoreApplication>
#include <QtCore>
#include <QtGui>
#include <QMainWindow>
#include <QTime>
#include <QDir>
#include <QtGui/QFileDialog>
#include <QSettings>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "mugshot.h"
#include "paintcanvas.h"
#include <QDebug>



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QString nopath, vidpath,bgpath,lastpath ,projectSaveDirectory;

public slots:
    void loadMugShot();
    void timerEvent(QTimerEvent *);


private slots:
    void on_videoButton_clicked();

    void on_bgButton_clicked();

    void on_saveButton_clicked();

    void on_startButton_clicked();

    void startPainter();

    void on_resetButton_clicked();

    void on_confirmButton_clicked();
    void processPolygons();


private:
    Ui::MainWindow *ui;
    CvCapture* capture;
    bool isPlaying,isMugShoting,isDrawing;
    QImage qimage;
    void updateImage(IplImage* cvimage);
    Mugshot* mugShotter;
    IplImage* updateFrame();
    IplImage* updatePng();
    bool checkreadytoPlay();
    QTimer* timer;
    QGraphicsScene *scene;
    QGraphicsRectItem *rectangle;
    QGraphicsPixmapItem *pixItem;
    PaintCanvas *paintCanvas;
    int center, width, height,x,y;
    float avgX, avgY;
    int xTot, yTot,maxX,maxY,minX,minY;

    QMessageBox::StandardButton replyNull;
    QPoint tempCentroid;
    QPoint tempMax;
    QPoint tempMin;
    vector<QPoint> centroids;
    vector<QPoint> minXY;
    vector<QPoint> maxXY;
    QPoint tempPoint;
    vector<QPoint> tempV;
    vector< vector<QPoint> >::iterator row;
    vector<QPoint>::iterator col;


};

#endif // MAINWINDOW_H


