#pragma once

//#include <vector>
#include <opencv/cv.h>

#include "Blob.h"

using namespace std;



/**
  Class used to compute and store the slider values
  */
class Blobs
{
public:
    Blobs() { };
   Blobs(cv::Mat&, vector<vector<cv::Point> >, unsigned long long int);

   vector< Blob > blobs;

   unsigned long long int frameNum;
   int num_blobs;
   //vector<int> blob_size;
   //vector<cv::Point> blob_pos;
   //vector<int> blob_label;
    vector< vector<cv::Point> > contours;

    //vector<cv::Scalar> blob_colors;

   vector<double> vel_avg;

   void drawBlobsInto(cv::Mat& orig);

   Blob* operator[](int index);

   set<Blob*> toSet();
   set<int> getIndices();

   int numberOfMatchedBlobs();
   Blob* getBlobByLabel(int label);

   vector< Blob* > getTwoLargestBlobsWithLabelsNotIn(set< int > labels);

   double computeAverageVelocity();
   int computeTotalArea();

private:

   //cv::Point getCentroid(vector<cv::Point>);
   void calcVelocity(cv::Mat&, vector<vector<cv::Point> >);

};
