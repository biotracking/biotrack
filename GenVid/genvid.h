#ifndef GENVID_H
#define GENVID_H

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
#include <QMap>

using namespace cv;

namespace Ui {
    class GenVid;
}

class GenVid : public QMainWindow
{
    Q_OBJECT

public:
    explicit GenVid(QWidget *parent = 0);
    ~GenVid();
    QString lastpath, videopath,nopath,btfpath,current,savepath;
    QStringList btfFiles;
    VideoWriter vOut;
    int currentExport;
    //ImgProcessor layerGen;
    void processFiles();
    Mat checkFile(Mat img);
    Mat currentimg;

    void checkReady();
    void runExporter(Mat img);
    Point rot2d(Point p,Point org,double t);

    Point org, r1,r2,r3,r4,arrowPTtip,a2,a3,a4,rparam,antXY;
    bool cmd,isPlaying,isExporting, everythingok,ok,boxOn,arrowOn,idOn,trailOn, cirOn,xyOn,angleOn,cirU,trailU,arrU;
    double fontSize, scrubRatio;
    int currentFrame, trailSize, boxStroke, lineType, r,g,b,rad,cirStroke,trailStroke, arrowSize, arrowStroke;
    QColor tempcolor;

    //A vector of every MSec, for a particular MSec a vector of ants, each ant is a qstringlis of parameters.
    QStringList tempant, prevant;
    vector <QStringList> antsAtMsec;
    QStringList antColorR,antColorG,antColorB;
    QStringList currenttrack,previoustrack;
    vector< vector<QStringList> > Msec;
    vector<Point> singleTrail, tempTrail;
    vector< vector<Point> > antTrails;
    int randInt(int, int);
    QMap<QString, QColor> map;


public slots:
    void on_exportButton_clicked();
    void on_actionLoad_BTF_triggered();

private slots:
    void on_actionLoad_Video_triggered();

    void on_horizontalSlider_valueChanged(int value);

    void on_playButton_clicked();

    void on_checkBox_clicked();

    void on_checkID_clicked();

    void on_checkArrow_clicked();

    void on_checkTrail_clicked();

    void on_spinBox_valueChanged(int arg1);

    void on_spinBox_2_valueChanged(int arg1);

    void on_spinBox_3_valueChanged(int arg1);

    void on_spinBox_5_valueChanged(int arg1);

    void on_pushButton_pressed();

    void on_horizontalSlider_sliderMoved(int position);

    void on_spinBox_7_valueChanged(int arg1);

    void on_spinBox_6_valueChanged(int arg1);

    void on_spinBox_8_valueChanged(int arg1);

    void on_spinBox_4_valueChanged(int arg1);

    void on_checkBox_2_clicked();

    void on_radioButton_clicked();

    void on_radioButton_2_clicked();

    void on_radioButton_3_clicked();

    void on_checkID_2_clicked();

    void on_checkID_3_clicked();

    void on_spinBox_9_valueChanged(int arg1);

    void on_videopathpushButton_clicked();

    void on_videopathpushButton_2_clicked();
    void refresh();

signals:

private:
    Ui::GenVid *ui;
    VideoCapture capture;
    QImage qimage;
    QString idfilepath,xfilepath,yfilepath,anglefilepath,timefilepath,timestampfilepath,videoDir;
//    QTextStream idStamp,xStamp,yStamp,angleStamp,timeStamp;
    QString time,timestamp,xs,ys,ant_id,angle;
    QStringList timeList,timeStampList,xsList,ysList,ant_List,angleList;

    void updateImage(Mat img);
    Mat updateFrame();
    void updater();
    void timerEvent(QTimerEvent*);
    float btfLength;
       int minlength;


};

#endif // GENVID_H
