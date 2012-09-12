#ifndef ICPTRACKER_H
#define ICPTRACKER_H



#include <vector>
#include <map>
#include <utility>
#include "Track.h"
#include <QString>
#include <QDir>

#include "ui_multitrack.h"





/**
 * This is the main tracker class.
 It takes in a set of file paths, analyzes video and perfoms tracking.
 It also now takes in a pointer to the UI, so fewer Getters and Setters are needed
 */
class ICPTracker
{

public:
    ICPTracker(float fps, QString bgImagepath,  QString modelImagepath, QString maskImagepath,  Ui::MultitrackClass uiPass);
    ~ICPTracker();


    void updateImage(Mat img) { image = img; }
    void track(Mat img, int timeIndex);
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
        model_clouds_orig=loadModelPoints(modelImages);

    }
    void setMatchDistanceThreshold(int t){
        trackToBlobDistanceThreshold=t;
    }
    void setTrackDeathThreshold(int t){
        trackDeathThreshold=t;
    }

    void outputInteractionsReport();
    void outputBTF(QString projectdirectory,QString icpprojectname);
    void setVideoShowing(bool showing) { isVideoShowing = showing; }
    void setContourTracking(bool cTracking) { isContourTracking = cTracking;	model_clouds_orig=loadModelPoints(modelImages); }

    bool showSearchRadius;
    bool showRemovalRadii;
    bool showModel;
    bool showTrails;
    bool showBox;

    std::vector<Track*> activeTracks;
    std::vector<Track*> inActiveTracks;

    int Ticp_maxIter;
    double Ticp_transformationEpsilon;
    double Ticp_euclideanDistance;

//void    viewerOneOff (pcl::visualization::PCLVisualizer& viewer);



private:
    Ui::MultitrackClass uiICP;
    int numOfTracks;
    Mat bgImage;
    Mat image;
    Mat bgSubImage;
    Mat bgSubImageGray;
    Mat bgSubImageGraySmall;
    Mat bgSubImageGraySmallScratch;
    Mat trackResultImage;
    Mat maskImageGray;
    Mat modelImage;
    vector<Mat> modelImages;
    Point modelDimensions;
    bool isVideoShowing;
    bool isContourTracking;
    float thefps;

    int maxModelDimension;

    std::vector<Point> dataPts;
   vector< pcl::PointCloud<pcl::PointXYZRGB> > model_clouds_orig;
    pcl::PointCloud<pcl::PointXYZRGB> data_cloud;


    double bgsubstractionThreshold;
    int modelTOdataThreshold;
    int trackDeathThreshold;
    int separationThreshold;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > loadModelPoints(vector<Mat> imgBGRAlist);
    int frameIndex;
    int trackToBlobDistanceThreshold;
    void drawTrackResult(Mat img);
    void drawTrackResultOriginal(Mat img);
    void processInteractions(Track* ta, Track* tb, FILE* fp);
    void match(std::vector<Point>& pts, std::vector<Point>& modelPts, std::vector<int>& closestToTarget, std::vector<int>& closestToModel, double* distance);
    template<typename T>
    void alphaBlendBGRA(const Mat& src1, const Mat& src2, Mat& dst);

    Mat runContourDetection(Mat img);
    int resolutionFractionMultiplier;


    /** Sample Struct
    //	typedef struct TargetBlob
    //	{
    //		Mat Blobmat;
    //        Point centroid;
    //        std::vector<Point> pts;
    //		bool isMatched;
    //	}
    //	TargetBlob;
    //	std::vector<TargetBlob> blobs;

    **/

protected:

};

#endif // ICPTRACKER_H
