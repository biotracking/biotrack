#ifndef ABOSMAINWINDOW_H
#define ABOSMAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QtGui/QMessageBox>
#include <cv.h>
#include <highgui.h>
#include <qthread.h>
#include <QVector>

//#include "imagedatapool.h"
#include "abospool.h"
#include "imagecapture.h"
#include "AbosThread.h"

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
				QVector<AbosPool*> pools, 
				int framerate, QWidget *parent = 0);
    ~MainWindow();

    void startThreads(bool start);


private:
    Ui::MainWindow *ui;
    AbosPool* image_pool;  // read captured frames from here
    int fps;

    int input_width;
    int input_height;

    QImage qimage;
    cv::Mat share_cvimage;

    QVector<AbosThread*> thread_vector;
	QVector<AbosPool*> pool_vector;

    PoolFrame* readFrame();
	const cv::Mat* readIDImage();
    void produceSharedQImage(cv::Mat cvimage);

protected:
    void timerEvent(QTimerEvent*);  // overide timerEvent function to draw image sequence
	void closeEvent(QCloseEvent*);




private slots:
    void on_actionExit_triggered();
};

#endif // ABOSMAINWINDOW_H
