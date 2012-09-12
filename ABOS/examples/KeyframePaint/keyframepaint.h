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
#include <QList>
#include <QMainWindow>
#include <QButtonGroup>
#include <QScrollArea>
 class PaintCanvas;

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
    QButtonGroup *buttongroup;
    QScrollArea *imageScroll;
    int zoomsize;

public slots:


private:
    bool saveFile(const QByteArray &fileFormat);

    PaintCanvas *paintCanvas;

    Ui::MainWindow *ui;
    int timer_id;
    AbosPool* image_pool;  // read captured frames from here
    int fps;

    int input_width;
    int input_height;

    QImage qimage;
    cv::Mat share_cvimage;

    QVector<AbosThread*> thread_vector;
	QVector<AbosPool*> pool_vector;

    void updateUI();
    PoolFrame* readFrame();
	const cv::Mat* readIDImage();
    void produceSharedQImage(cv::Mat cvimage);

protected:
    void timerEvent(QTimerEvent*);  // overide timerEvent function to draw image sequence
	void closeEvent(QCloseEvent*);




private slots:

        void on_stopButton_clicked();
        void on_playButton_clicked();
        void on_pauseButton_clicked();
        void on_FramespinBox_editingFinished();
        void on_nextFrameButton_clicked();
        void on_previousFrameButton_clicked();

        void on_targetpaint_clicked();
        void on_nottargetpaint_clicked();
        void on_clearButton_clicked();
        void on_saveFrameButton_clicked();
        void on_backgroundmaskbutton_clicked();
        void on_zoomButton_clicked();
        void on_currentTimeBar_sliderMoved(int position);
        void on_pensizespinBox_valueChanged(int arg1);
        void on_markHeadbutton_clicked();
        void on_markTailbutton_clicked();
};

#endif // ABOSMAINWINDOW_H
