
#ifndef MODELMAKER_H
#define MODELMAKER_H

#include <QMainWindow>
#include <QtCore/QCoreApplication>
#include <QtCore>
#include <QtGui>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QPixmap>
#include <QtGui/QLabel>
#include <QtGui/QImage>
#include <QtGui/QFileDialog>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <paintcanvas.h>
#include <QMap>

#include <QString>
#include <QPoint>
#include <QWidget>
#include<QDebug>
#include<QFile>

using namespace cv;

/** The ModelMaker class accepts GUI commands, communicates with a widget painting device, and exports a model file with a standard orientation,
and the head and centroid encoded into the alpha channel.
**/

namespace Ui {
class ModelMaker;
}

class ModelMaker : public QMainWindow
{
    Q_OBJECT

public:
    explicit ModelMaker(QWidget *parent = 0);
    ~ModelMaker();
    QString current, nopath,lastpath,videopath,savepath,bgpath;
    bool isPlaying,subtract,everythingok,ok;
    int currentFrame;

    Mat bgimg;


    QString polyText;
    QStringList parsedPolygons;
    QStringList polyPart;
    bool loadedPolygons, saveOut, isReady, isScrubbing;
    int center, width, height,x,y;
    float avgX, avgY, scrubRatio;
    int xTot, yTot,maxX,maxY,minX,minY;
    QGraphicsScene *scene;
    QGraphicsRectItem *rectangle;
    QGraphicsPixmapItem *pixItem;

    PaintCanvas *paintCanvas;
    QMessageBox::StandardButton replyNull;
    QPoint tempCentroid;
    QPoint tempMax;
    QPoint tempMin;
    vector<QPoint> centroids;
    vector<QPoint> minXY;
    vector<QPoint> maxXY;
    QPoint tempPoint;
    vector<QPoint> tempVariable;
    vector< vector<QPoint> >::iterator row;
    vector<QPoint>::iterator col;
    QFont font;
    vector<QPoint> uCent,uMax,uMin, polygon;
    int targetX,targetY,croppedWidth,croppedHeight,temp,centerX,centerY,x1,y1,x2,y2,ant;
    cv::Mat cropframe,rotframe,imgRot;
    QString polyinfo;
    double angle;

    int subThreshold;

    void checkReady();


private slots:
    void resizeEvent(QResizeEvent *);

    void on_actionLoad_Video_triggered();

    void on_seekbar_valueChanged(int value);

    void on_actionLoad_Background_triggered();

    void on_playButton_clicked();

    void on_subviewButton_clicked(bool checked);

    void startpainter();
    void processPolygons();
    Mat subtractBack(Mat img);
    void extractModel(Mat origframe);
    void getCenter();
    void getHead();

    Mat rotateImage(const Mat& source, double anglerad);


    void on_actionSave_Direcory_triggered();

    void on_loadBackgroundpushButton_clicked();

    void on_loadVideopushButton_clicked();

    void on_seekbar_sliderMoved(int position);

    void on_threshbar_sliderMoved(int position);

private:
    Ui::ModelMaker *ui;

    VideoCapture capture;
    QImage qimage;

    void displayImage(Mat cvimage);
    Mat nextFrame();
    void timerEvent(QTimerEvent*);

    int framewidth, frameheight;
    float xscale, yscale;
    Mat currentimg;

};

#endif // MODELMAKER_H
