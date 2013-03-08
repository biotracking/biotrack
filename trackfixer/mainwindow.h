#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include <QtGui/QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QGraphicsScene>
#include <QGraphicsLineItem>
#include <QGraphicsRectItem>
#include <opencv2/opencv.hpp>
//#include <qthread.h>
#include <vector>


namespace Ui
{
    class MainWindow;
}

class TrackletRectItem : public QGraphicsRectItem
{
public:
    //TrackletRectItem(qreal x, qreal y, qreal width, qreal height, QGraphicsItem* parent=0, QGraphicsScene* scene=0);
    bool flipped, selected, nuked;
    int startFrame, endFrame, antID;
    std::vector<double> ximage, yimage, timage, timestamp;
    QColor color;
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void loadVideo(std::string fname);
    void loadBTF(std::string dname);
    
    double getScaleFactor(){ return scaleFactor;};
    void setScaleFactor(double sf){ scaleFactor = sf;};
    
    double getFPS(){ return fps;};
    void setFPS(double _fps){ fps = _fps;};



private:
    Ui::MainWindow *ui;
    int timestamp_idx,timestamp_frames_idx,type_idx,  id_idx, x_idx, y_idx, t_idx;
    int timerID;
    int frame_count;
    double scaleFactor, fps;
    bool paused;

    QImage qimage;
    cv::Mat share_cvimage;
    cv::VideoCapture cap;
    //std::vector<cv::Mat> loadedVideo;
    //std::vector<std::vector<std::string> > btf_data;
    std::vector<std::string> btf_names;
    //std::vector<std::vector<int> > frame_data;
    //std::vector<std::pair<std::string,int> > flip_data;
    //std::vector<int> removed_tracks;
    QImage Mat2QImage(const cv::Mat3b &src);
    QGraphicsScene scene;
    QGraphicsLineItem* curFrameLine;
    std::vector<TrackletRectItem*> tracklets;
    std::vector<std::vector<TrackletRectItem*> > frame_tracklets;

protected:
    void timerEvent(QTimerEvent*);  // overide timerEvent function to draw image sequence
	void closeEvent(QCloseEvent*);
	void clearBTFData();


private slots:
    void on_actionExit_triggered();
    void on_pushButton_4_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_5_clicked();
	void on_pushButton_6_clicked();
    void on_actionLoad_video_file_triggered();
    void on_horizontalSlider_valueChanged(int value);
    void on_actionLoad_BTF_triggered();
    void on_pushButton_7_clicked();
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_10_clicked();
};

#endif // MAINWINDOW_H
