#ifndef TRACK_H
#define TRACK_H


#include <QtCore>
#include <QtGui>


/** OPENCV INCLUDES**/
#if defined(WIN32) && !defined(linux)
#include "opencv.hpp" //FOR WINDOWS
#elif defined(linux) && !defined(WIN32)
#include <cv.h> //for linux
#include <highgui.h> //for linux

#else
//Error! Got to be linux or windows not both or neither (or i guess mac!)
#include <cv.h>
#include <highgui.h>
#endif
/** END OPENCV INCLUDES**/


#include <vector>
#include <stdio.h>

#include <iostream>
#include "pcl/kdtree/impl/kdtree_flann.hpp" //this include has to go above others to work!
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include<pcl/search/pcl_search.h>


#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <QDebug>
#include "ui_multitrack.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;


/**
 * This class is responsible for managing the state of an ant
 * track.  The state of a track is the frame index of birth
 * and death, centroid at birthtime.
 * Rotation, timestamp, translation, and ID information are kept for each track
 */
class Track
{

public:
    Track(int index, pcl::PointXYZ initTranslation, int identification, double matchDThresh,int sepThresh, int resFracMultiplier);
    ~Track();

    double getX(int idx=-1);
    double getY(int idx=-1);
    double getScale(int idx=-1);
    double getRotationAngle(int idx=-1);
    void getTemplatePoints(pcl::PointCloud<pcl::PointXYZ>& modelPts, int idx=-1);
    bool isBirthFrame;
    pcl::PointCloud<pcl::PointXYZ> update(pcl::PointCloud<pcl::PointXYZ> dataPTS_cloud,pcl::PointCloud<pcl::PointXYZ> modelPTS_cloud,
                                          int areaThreshold,  int separateThreshold, int matchDThresh,
                                          int ICP_ITERATIONS, double ICP_TRANSEPSILON, double ICP_EUCLIDEANDIST);

    int icp_maxIter;
    double icp_transformationEpsilon;
    double icp_euclideanDistance;
    int getBirthFrameIndex() { return birthFrameIndex; }
    int getDeathFrameIndex() { return deathFrameIndex; }
    int getFrameIndex() { return frameIndex; }
    int getLength() { return absoluteTransforms.size(); }
    int getFrameIndexOfLastUpdate() { return birthFrameIndex + absoluteTransforms.size()-1; }
    void end() { deathFrameIndex = birthFrameIndex + absoluteTransforms.size()-1; }
    bool isActive() { return deathFrameIndex > 0; }
    int getID() { return id; }
    bool wasBirthed() { return isBirthable; }
    int getNumberOfContinuousZombieFrames() { return numberOfContinuousZombieFrames; }
    int birthFrameIndex;
    int frameIndex;
    int deathFrameIndex;
    int id;
    double matchDistanceThreshold;
    double nukeDistanceThreshold; // used when a neighborhood of pts close to a model point is nuked

private:
    Ui::MultitrackClass uitrack;

    pcl::PointXYZ initialTranslation;
    std::vector<Eigen::Matrix4f> transforms; // vector of incremental transformation matrices State = T_n * (T_n-1 * ... T_2 * T_1) * I
    std::vector<int> zombieIndicesIntoAbsTransforms; //notes which abs transforms are zombies (ie transitioning to death)
    int numberOfContinuousZombieFrames;
    bool isBirthable;
    // Absolute transformation matrices are useful for random frame access
    std::vector<Eigen::Matrix4f> absoluteTransforms; // vector of absolute transformation matrices State = T_x * I where x is any state //TODO update this

    //ICP alignment
    Eigen::Matrix4f updateTransformPCL(pcl::PointCloud<pcl::PointXYZ> data_cloud,pcl::PointCloud<pcl::PointXYZ> model_cloud);

    double matchScore;
    bool didConverge;


    void transformPoints(std::vector<Point>& modelPts, Mat transform);
    void transformCloud(pcl::PointCloud<pcl::PointXYZ> modelPTS_cloud, Eigen::Matrix4f transform);
    pcl::PointCloud<pcl::PointXYZ> removeClosestDataCloudPoints(pcl::PointCloud<pcl::PointXYZ> point_cloud_for_reduction,pcl::PointCloud<pcl::PointXYZ> removal_Cloud, int distanceThreshold);
    void removeClosestDataPoints(std::vector<Point> &reducedPts, Point queryPoint, int distanceThreshold);
    int  resolutionFracMultiplier;

    pcl::PointCloud<pcl::PointXYZ> tformedModel_cloud180;


protected:

};

#endif // TRACK_H
