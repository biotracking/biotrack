#ifndef MUGSHOT_H
#define MUGSHOT_H
#include <stdio.h>
#include <QString>
#include <QPoint>
#include <QWidget>
#include<QDebug>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace cv;

class Mugshot
{
public:
    Mugshot(QString videopath, QString bgImagepath,  QString savepath);
    ~Mugshot();

//    IplImage* frame;
    cv::Mat cropframe;
    cv::Mat rotframe;
    IplImage* frameorig;
    IplImage* tempframe;
    IplImage img;
    IplImage imgRot;

    IplImage* frametest;
    IplImage* bgImage;
    IplImage* frameGray;


    int targetX,targetY,maxXY,croppedWidth,croppedHeight,temp,centerX,centerY,x1,y1,x2,y2,ant,currentFrame;
    double angle;
    bool ok;
    int xcent;
    int ycent;
    int offset;
    int antWidth;
    int antHeight;
    int totalspots;
    double anglechange; //18 degrees in radians
    vector<QPoint> uCent,uMax,uMin;




    Mat rotateImage(const Mat& source, double anglerad);

//    IplImage* loop(IplImage* img);
    void loop(IplImage* img);
    QString path;
};

#endif // MUGSHOT_H
