#ifndef BACKGROUNDER_H
#define BACKGROUNDER_H

#include <QMainWindow>
//OPENCV INCLUDES
#if defined(WIN32) && !defined(linux)
#include "opencv.hpp" //FOR WINDOWS
#elif defined(linux) && !defined(WIN32)
#include <cv.h> //for linux
#include <highgui.h> //for linux

#else
//Error! Got to be linux or windows not both or neither
#endif
//END OPENCV INCLUDES

#include <vector>
//#include "stdio.h"
#include <stdio.h>


#include <QTime>
#include <QDir>
//#include <QtGui/QFileDialog>
#include <QFileDialog> //qt5 includes moved
#include <QSettings>
#include <QtCore/QCoreApplication>
#include <QtCore>


//#include <QtGui/QMainWindow>
//#include <QtGui/QPushButton>
//#include <QtGui/QPixmap>
#include <QMainWindow>
#include <QPushButton>
#include <QPixmap>

//#include <QtGui/QLabel>
#include <QLabel>

//#include <QtGui/QImage>
#include <QImage>
 #include <QDebug>
#include "BackgroundCalculator.h"
using namespace cv;
using namespace std;



/**
  * This gui class selects files and chooses the backgrounding algorithm
  *
  **/

namespace Ui {
class Backgrounder;
}

class Backgrounder : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit Backgrounder(QWidget *parent = 0);
    ~Backgrounder();

    QString videopath;
    QString bgpath;
    QString nopath;
    QString lastpath;


    bool checkVideo();
    void loadVideo();
    void updatedisplay_videoin(int framenum);

    void updatedisplay_bgout();


    VideoCapture capture;
    int videowidth;
    int videoheight;


    Mat currentFrameViz;
    int INFrame;
    int OUTFrame;
    int TOTALVIDEOFRAMES;
    int TOTALRANGEofFRAMES;

    int avemode;


    //BackgroundCalculator backgroundCalculator;

    BackgroundCalculator* backgroundCalculator;
    QString savelocation;
    bool isRunning;



  void timerEvent(QTimerEvent*);
    
private slots:
    void on_startframeSlider_sliderMoved(int position);

    void on_loadVideoButton_clicked();

    void on_endframeSlider_sliderMoved(int position);

    void on_inFramespinBox_editingFinished();

    void on_outFramespinBox_editingFinished();

    void on_stopButton_toggled(bool checked);

    void on_saveBGButton_clicked();


private:
    Ui::Backgrounder *ui;
};

#endif // BACKGROUNDER_H
