#ifndef ANT_ID_H
#define ANT_ID_H

#include <QMainWindow>
#include <QDebug>
#include <QDir>
#include <fstream>
#include <opencv/highgui.h>
#include <opencv/cv.h>

using namespace std;
using namespace cv;

namespace Ui {
class Ant_ID;
class UVHistogram;
}

class Ant_ID : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit Ant_ID(QWidget *parent = 0);
    ~Ant_ID();

    MatND getYUVHist(Mat a);

    Mat p_img;
    Mat p_img_2;
    MatND hist;
    MatND hist2;
    double correl,intersect;

    Mat maskedimg;
    Mat img;
    Mat mask;
    Mat maskgrey;
    Mat hist_img;

    int K;
    int numTrainingImgs,numUnknownImages, maxTimgRows,maxTimgCols, numKnownAnts;
    int antIndex, totalImages, numMatch, tresholdValue;
    QString traingingheader,mugheader,outheader, name;
    bool match;


    vector<MatND>trainingHistograms;
    vector<MatND>unknownHistograms;
    vector<int>trainingIDs;
    vector<int>unknownIDs;

private:
    Ui::Ant_ID *ui;
};

#endif // ANT_ID_H
