#include <cv.h>

class Blobs {

public:

   unsigned long long int frameNumber;
   int blobSize;
   vector<cv::Point> contourPts;
   vector<cv::Point> allPts;
   cv::Point centroid;
   double avgVel;
   double avgColor;
};
