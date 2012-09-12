#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define _USE_MATH_DEFINES

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
#include <QSettings>
#include <QInputDialog>
#include<math.h>



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
    QString polyText;
    QStringList parsedPolygons;
    QStringList polyPart;
    bool loadedPolygons;


public slots:
    void loadMugShot();
    void timerEvent(QTimerEvent *);
    void createCirParams(int x, int y);


private slots:
    void on_videoButton_clicked();

    void on_bgButton_clicked();

    void on_saveButton_clicked();

    void on_startButton_clicked();

    void startPainter();

    void on_resetButton_clicked();

    void on_confirmButton_clicked();
    void processPolygons();
    void loadSettings();
    void loadPolygons();

    void on_subviewButton_toggled(bool checked);

    void on_thresholdSpinBox_valueChanged(int arg1);

    void on_scrubSlider_valueChanged(int value);

    void on_skipSpinBox_valueChanged(int arg1);

    void on_polygonButton_clicked();

    void on_cirRectButton_clicked();
    void on_cirSpinBox_valueChanged(int arg1);
    void on_wSlider_valueChanged(int arg1);
    void on_hSlider_valueChanged(int arg1);
    void on_cirCancelButton_clicked();
    void on_cirOkButton_clicked();


    QPoint rot2d(QPoint p,double t);


private:
    Ui::MainWindow *ui;
    CvCapture* capture;
    bool isPlaying,isMugShoting,isDrawing, isScrubbing;
    QImage qimage;
    void updateImage(IplImage* cvimage);
    Mugshot* mugShotter;
    IplImage* updateFrame(IplImage* img);
    IplImage* updatePng(IplImage* img);
    bool checkreadytoPlay();
    QTimer* timer;
    QGraphicsScene *scene;
    QGraphicsRectItem *rectangle;
    QGraphicsPixmapItem *pixItem;
    PaintCanvas *paintCanvas;
    int center, width, height,x,y;
    float avgX, avgY, scrubRatio;
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
    QFont font;

};

#endif // MAINWINDOW_H


