#ifndef MULTITRACK_H
#define MULTITRACK_H

#include <QtCore/QCoreApplication>
#include <QtCore>


#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QPixmap>

#include <QtGui/QLabel>

#include <QtGui/QImage>

//Possible Windows Includes
//#include <C:\opencv\include\opencv\cv.h>
//#include <C:\opencv\include\opencv\highgui.h>



#include "ICPTracker.h"
#include <QTime>
#include <QDir>
#include <QtGui/QFileDialog>
#include <QSettings>
#include "ui_multitrack.h"



class Multitrack : public QMainWindow
{
	Q_OBJECT

public:
    Multitrack(QWidget *parent = 0, Qt::WFlags flags = 0);
    ~Multitrack();
	QString projectSaveDirectory;
		QString videopath;

        QString modelfolder;
		QString maskpath;
		QString bgpath;
		QString nopath;
		QString lastpath;
		void readSettings();
		void writeSettings();

		

	

public slots:
	void toggleStopButton();
	void toggleBlobsView();
    void toggleContourTracking();
    void bgThresholdSpinValueChanged(int value);
	void blobBirthAreaThresholdValueChanged();
	void resolutionFractionValueChanged();
	void trackdistanceValueChanged();
	void trackDeathValueChanged();
	void separationValueChanged();
	void nextFrame();
    void saveHTMLinteractions();
	void saveBTF();
    void loadBackgroundFile();
	void loadVideoFile();
	void loadModelFile();
	void chooseMaskFile();
	void chooseProjectDirectory();
	void icpReset();

	void loadSettings();
	void loadDefaults();
	void saveSettings();

private:
    Ui::MultitrackClass ui;
	ICPTracker* icpTracker;
    QLabel* imageLabel;
	QImage qimage;
    VideoCapture capture;

    float vidFPS;
	int videowidth;
	int videoheight;
	unsigned char *imageData;
	int imageWidth;
	int imageHeight;
	bool isPlaying;
    bool isVideoShowing;
    QString statusMessage;
    void updateImage(Mat dataimage);
	void updateStatusBar();
	void messageToStatusBar(QString message);
    Mat updateFrame();

    int resFractionMultiplier; // eg. 4 for 1/4 resolution, 2 for 1/2 resolution, 1 for full resolution
	
	bool checkreadytoPlay();
	
    void loadNewTracker();
    bool trackerCheck();
    bool trackerChecked;
    bool completedTracking;

    Mat previousImg;
    Mat bgImage;
    Mat bgSubImage;
    Mat bgSubImageGray;
    Mat bgSubImageGraySmall;

    Mat maskImage;
    Mat modelImage;

    int displayWidth;
    int displayHeight;
		
    int pHour;
   int pMin;
   int pSec;
   int pMsec;
   int pausedMillis;


    QTime myQTime;
	int oldframeTime;

protected:
    void timerEvent(QTimerEvent*);
	 void closeEvent(QCloseEvent *event);

private slots:
     void on_displaycomboBox_currentIndexChanged(int index);
     void on_trackDistanceViewCheck_toggled(bool checked);
     void on_modelViewcheckBox_toggled(bool checked);
     void on_showTrailscheckBox_toggled(bool checked);
     void on_showBoxcheckBox_toggled(bool checked);
     void on_separationViewCheck_toggled(bool checked);
     void on_bgSubThresholdSpinBox_valueChanged(int arg1);
     void on_bgsubSlider_sliderMoved(int position);
     void on_blobBirthAreaThresholdSpinBox_valueChanged(int arg1);
     void on_matchSlider_sliderMoved(int position);
     void on_trackdeathSpinBox_valueChanged(int arg1);
     void on_deathSlider_sliderMoved(int position);
     void on_separationSlider_sliderMoved(int position);
     void on_separationSpinBox_valueChanged(int arg1);
     void on_trackdistanceSpinBox_valueChanged(int arg1);
     void on_searchSlider_sliderMoved(int position);
     void on_ICP_MaxIterspinBox_valueChanged(int arg1);
     void on_ICP_TransEpsilondoubleSpinBox_valueChanged(double arg1);
     void on_ICP_EuclideanDistdoubleSpinBox_valueChanged(double arg1);
     void on_visualizationcheckBox_toggled(bool checked);
};

#endif // MULTITRACK_H
