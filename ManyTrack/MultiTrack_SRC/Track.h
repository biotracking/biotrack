#ifndef TRACK_H
#define TRACK_H


#include <QtCore>
#include <QtGui>
#include <utility>

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

#include <fstream>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
//#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
//#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/ros/conversions.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>



//To visualize Clouds
#include <pcl/visualization/cloud_viewer.h>

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
    Track(int index, pcl::PointXYZRGB initTranslation, int identification, double matchDThresh,int sepThresh, int resFracMultiplier);
    ~Track();

    double getX(int idx=-1);
    double getY(int idx=-1);
    double getScale(int idx=-1);
    double getRotationAngle(int idx=-1);
    void getTemplatePoints(pcl::PointCloud<pcl::PointXYZRGB>& modelPts, int idx=-1);
    bool isBirthFrame;
    pcl::PointCloud<pcl::PointXYZRGB> update(pcl::PointCloud<pcl::PointXYZRGB> dataPTS_cloud,    vector< pair<PointCloud<pcl::PointXYZRGB>, QString> > modelPTS_clouds,
                                          int areaThreshold,  int separateThreshold, int matchDThresh,
                                          int ICP_ITERATIONS, double ICP_TRANSEPSILON, double ICP_EUCLIDEANDIST);

    int identify (PointCloud<PointXYZRGB> dataPTS_cloud, vector<pair<PointCloud<PointXYZRGB>, QString> > modelgroup);

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
    int getID() { return IDnum; }
    bool wasBirthed() { return isBirthable; }
    int getNumberOfContinuousZombieFrames() { return numberOfContinuousZombieFrames; }
    int birthFrameIndex;
    int frameIndex;
    int deathFrameIndex;
    int IDnum;
    QString modelType;
    int modelIndex;
    double matchDistanceThreshold;
    double nukeDistanceThreshold; // used when a neighborhood of pts close to a model point is nuked

    float recentFitness;

//    Correspondences stuff
    void estimateKeypoints (const PointCloud<PointXYZ>::Ptr &src,
                       const PointCloud<PointXYZ>::Ptr &tgt,
                       PointCloud<PointXYZ> &keypoints_src,
                       PointCloud<PointXYZ> &keypoints_tgt);
    void estimateNormals (const PointCloud<PointXYZ>::Ptr &src,
                     const PointCloud<PointXYZ>::Ptr &tgt,
                     PointCloud<Normal> &normals_src,
                     PointCloud<Normal> &normals_tgt);
    void estimateFPFH (const PointCloud<PointXYZ>::Ptr &src,
                       const PointCloud<PointXYZ>::Ptr &tgt,
                       const PointCloud<Normal>::Ptr &normals_src,
                       const PointCloud<Normal>::Ptr &normals_tgt,
                       const PointCloud<PointXYZ>::Ptr &keypoints_src,
                       const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
                       PointCloud<FPFHSignature33> &fpfhs_src,
                       PointCloud<FPFHSignature33> &fpfhs_tgt);
  void  findCorrespondences (const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
                         const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
                         Correspondences &all_correspondences);

  void  rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                              const PointCloud<PointXYZ>::Ptr &keypoints_src,
                              const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
                              Correspondences &remaining_correspondences);

  void  computeTransformation (const PointCloud<PointXYZ>::Ptr &src,
                           const PointCloud<PointXYZ>::Ptr &tgt,
                           Eigen::Matrix4f &transform);

private:
    Ui::MultitrackClass uitrack;

    pcl::PointXYZRGB initialTranslation;
    std::vector<Eigen::Matrix4f> transforms; // vector of incremental transformation matrices State = T_n * (T_n-1 * ... T_2 * T_1) * I
    std::vector<int> zombieIndicesIntoAbsTransforms; //notes which abs transforms are zombies (ie transitioning to death)
    int numberOfContinuousZombieFrames;
    bool isBirthable;
    // Absolute transformation matrices are useful for random frame access
    std::vector<Eigen::Matrix4f> absoluteTransforms; // vector of absolute transformation matrices State = T_x * I where x is any state //TODO update this

    //ICP alignment
    Eigen::Matrix4f update2DTransformPCL(pcl::PointCloud<pcl::PointXY> data_cloud,pcl::PointCloud<pcl::PointXY> model_cloud);
    Eigen::Matrix4f updateTransformPCLRGB(pcl::PointCloud<pcl::PointXYZRGB> data_cloud,pcl::PointCloud<pcl::PointXYZRGB> model_cloud);
//        Eigen::Matrix4f updateTransformPCL(pcl::PointCloud<PointT> data_cloud,pcl::PointCloud<PointT> model_cloud);

    double matchScore;
    bool didConverge;


    void transformPoints(std::vector<Point>& modelPts, Mat transform);
    void transformCloud(pcl::PointCloud<pcl::PointXYZRGB> modelPTS_cloud, Eigen::Matrix4f transform);
    pcl::PointCloud<pcl::PointXYZRGB> removeClosestDataCloudPoints(pcl::PointCloud<pcl::PointXYZRGB> point_cloud_for_reduction,pcl::PointCloud<pcl::PointXYZRGB> removal_Cloud, int distanceThreshold);
    void removeClosestDataPoints(std::vector<Point> &reducedPts, Point queryPoint, int distanceThreshold);
    int  resolutionFracMultiplier;

    pcl::PointCloud<pcl::PointXYZRGB> tformedModel_cloud180;






protected:

};

#endif // TRACK_H
