#ifndef ICPTRACKER_H
#define ICPTRACKER_H



#include <vector>
#include <map>
#include <utility>
#include "Track.h"
#include <QString>
#include <QDir>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <parallel/par_loop.h>
#include "ui_Manytrack.h"




/**
 * This is the main tracker class.
 It takes in a set of file paths, analyzes video and perfoms tracking.
 It also now takes in a pointer to the UI, so fewer Getters and Setters are needed
 */
class ICPTracker
{

public:
    ICPTracker(float fps, QString bgImagepath,  QString modelFolderpath, QString maskImagepath,  Ui::ManytrackClass uiPass);
    ~ICPTracker();


    std::vector<Model> models;
int fewestNumofModelPoints;
int largestNumofModelPoints;
QString mFolderPath;
    void updateImage(Mat img) { image = img; }
    void trackFrame(Mat scene_img, int timeIndex);

    Mat getBackgroundImage() { return bgImage; }
    Mat getBackgroundSubImage() { return bgSubImage; }
    Mat getTrackResultImage() { return trackResultImage; }
    int getBgSubThreshold() { return bgsubstractionThreshold; }
    int getTrackBirthAreaThreshold() { return modelTOdataThreshold; }
    int getResolutionFractionMultiplier() { return resolutionFractionMultiplier; }
    void setBgSubThreshold(int t) { bgsubstractionThreshold = t; }
    void setTrackBirthAreaThreshold(int t) { modelTOdataThreshold = t; }
    void setSeparationThreshold(int t) {
        separationThreshold = t; }
    void setResolutionFraction(int t) {
        resolutionFractionMultiplier = t;
        bgSubImageGraySmall.create(bgImage.rows / resolutionFractionMultiplier, bgImage.cols/ resolutionFractionMultiplier, CV_8UC3);//TODO get correct type
        models=loadModelClouds(mFolderPath);

    }
    void setMatchDistanceThreshold(int t){
        icpMatchDistanceThreshold=t;
    }
    void setTrackDeathThreshold(int t){
        trackDeathThreshold=t;
    }

    void outputInteractionsReport();
    void outputBTF(QString projectdirectory,QString icpprojectname);
//    void setVideoShowing(bool showing) { isVideoShowing = showing; }
  //  void setContourTracking(bool cTracking) { isContourTracking = cTracking;	models=loadModelClouds(mFolderPath); }

    bool showSearchRadius;
    bool showRemovalRadii;
    bool showModel;
    bool showTrails;
    bool showBox;

    double colorRegScale;

    std::vector<Track*> activeTracks;
    std::vector<Track*> inActiveTracks;

    int Ticp_maxIter;
    double Ticp_transformationEpsilon;
    double Ticp_euclideanDistance;

//void    viewerOneOff (pcl::visualization::PCLVisualizer& viewer);

    //************  Filters *******

//    int dilation_type;
//      if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
//      else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
//      else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }
    void Dilation( int dilation_elem, int dilation_size, Mat &src );

      Mat element;
       int dilation_type;



private:
    Ui::ManytrackClass uiICP;
    int numOfTracks;
    Mat bgImage;
    Mat image;
    Mat bgSubImage;
    Mat bgSubImageGray;
    Mat bgSubImageGraySmall;
    Mat trackResultImage;
    Mat maskImageGray;
    Mat modelImage;
    vector<Mat> modelImages;
    //Store Models as a combination of their Pointcloud and their identifying string
    vector<pair <PointCloud<PointXYZRGB> , QString> > ModelPairs;

//    Point modelDimensions;
//    bool isVideoShowing;
    bool isContourTracking;
    float thefps;


//    std::vector<Point> dataPts;
//   vector< pair< pcl::PointCloud<pcl::PointXYZRGB>, QString > > model_clouds_orig;


    pcl::PointCloud<pcl::PointXYZRGB> data_cloud;


    double bgsubstractionThreshold;
    int modelTOdataThreshold;
    int trackDeathThreshold;
    int separationThreshold;
    std::vector<Model> loadModelClouds(QString mPath);

    int numDetectionsinFrame;
vector<Model> modelFilesToMAT(QString modelFolderPath);
Model loadModelPoints(Model modelBGRAimg);


    int frameIndex;
    int icpMatchDistanceThreshold;
    void drawTrackResult(Mat img);
    void drawTrackResultOriginal(Mat img);
    void processInteractions(Track* ta, Track* tb, FILE* fp);
    void match(std::vector<Point>& pts, std::vector<Point>& modelPts, std::vector<int>& closestToTarget, std::vector<int>& closestToModel, double* distance);
    template<typename T>
    void alphaBlendBGRA(const Mat& src1, const Mat& src2, Mat& dst);

    void MattoCloudDetections(Mat img);


    Mat runContourDetection(Mat img);
    int resolutionFractionMultiplier;

//    boost::shared_ptr<pcl::visualization::CloudViewer> cMviewer;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

     //   pcl::visualization::CloudViewer viewer("Cloud Viewer");





protected:

private slots:

    void on_framesspinBox_valueChanged(int arg1);
    void on_framesSlider_sliderMoved(int position);





};

#endif // ICPTRACKER_H
