#ifndef ABOSMAINWINDOW_H
#define ABOSMAINWINDOW_H

#define SAVE_TO_XML 1

#define LOW_PASS_POWER 0.4

#include <QtGui/QMainWindow>
#include <QtGui/QMessageBox>
#include <QFile>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <qthread.h>
#include <QVector>
#include <QPainter>
#include <libxml/tree.h>
#include <libxml/parser.h>

#include "abospool.h"
#include "imagecapture.h"
#include "AbosThread.h"
#include "blobs.h"

#include <fstream>

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    MainWindow(	AbosPool *image_buf, 
				QVector<AbosThread*> modules,
            QVector<ImageCapture*> captures, 
				QVector<AbosPool*> pools,
            list<Blobs> *sliders,
                                int framerate, QString slider_output_path = "", QWidget *parent = 0);
    ~MainWindow();

    void startThreads(bool start);


public slots:

private:
    Ui::MainWindow *ui;
    int timer_id;
    AbosPool* image_pool;  // read captured frames from here
    int fps;

    int input_width;
    int input_height;

    int areaMax;
    double velMax;
    int pixelsMax;

    QImage qimage;
    cv::Mat share_cvimage;
    QPixmap pixmap;

    QVector<AbosThread*> thread_vector;
    QVector<ImageCapture*> capture_vector;
    QVector<AbosPool*> pool_vector;
    list<Blobs> *blobs_vec;
    ImageCapture* capture;

    //Blobs previousBlobs;

   bool isCaptureConfigured;
	bool p_status;
	bool v_status;
	bool isStart;

   xmlDocPtr read_doc;
   xmlNodePtr read_root_node, read_node;

   xmlDocPtr write_doc;
   xmlNodePtr write_root_node, write_node;
   char buff[256];

   ofstream csvSliderOutput;

   PoolFrame* readFrame();
   void initParameters();
   void produceSharedQImage(cv::Mat cvimage);
   void updateUI();
   void updateRawSliders(Blobs sliders);
   void updateExpectedSliders(unsigned long long int frameNo);
   void addToXml(int, int, int, int, int, int, int, int, int, int);


   float    size1LowPass;
   float    size2LowPass;
   float       x1LowPass;
   float       x2LowPass;
   float       y1LowPass;
   float       y2LowPass;
   float activityLowPass;
   float  densityLowPass;
   bool lowPassInited;

   int previousFish1Label;
   int previousFish2Label;
   bool gotFirstTwoFish;

   unsigned long long currentFrameNumber;

protected:
   void timerEvent(QTimerEvent*);  // overide timerEvent function to draw image sequence
   void closeEvent(QCloseEvent*);

private slots:

    void on_actionVideo_File_triggered();
    void on_actionProsilica_Camera_triggered();
    void on_actionStop_Tracking_triggered();
    void on_actionExit_triggered();
    void on_actionStart_Tracking_triggered();
};

#endif // ABOSMAINWINDOW_H
