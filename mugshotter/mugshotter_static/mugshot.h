#ifndef MUGSHOT_H
#define MUGSHOT_H
#include <stdio.h>
#include <QString>
#include <QPoint>
#include <QWidget>
#include<QDebug>
#include<QFile>
#include <QDir>
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
    IplImage* maskframe;

    IplImage maskimg;
    IplImage imgRot;

    IplImage* frametest;
    IplImage* bgImage;
    IplImage* frameGray;
    int threshold;
    int pixval;
    int nonblack;
    Vec3b pixel;
    Point a;
    uchar blue;
    uchar green;
    uchar red;
    Point coordinate;




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
    vector<QPoint> uCent,uMax,uMin, polygon;
    vector<QString>polyNames;
    vector<vector<QPoint> > polygons;
    vector< vector<QPoint> >::iterator row;
    vector<QPoint>::iterator col;
    QPoint tempPoint;
    vector<QPoint> tempV;
    int lineType;
    bool retSubtract;
    int skip;
    QDir sub;



    Mat rotateImage(const Mat& source, double anglerad);
    void loop(IplImage* img);
    IplImage* subtractBack(IplImage* img);
    QString path;
    CvMat maskPoly(Mat img, vector<QPoint> points);
    void writePoly();

};

#endif // MUGSHOT_H
