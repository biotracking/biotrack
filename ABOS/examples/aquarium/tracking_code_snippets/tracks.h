#include <cv.h>

class Tracks {

public:
   
   int trackId;
   int killCounter;
   vector<cv::Point> positions;
   double avgColor;
   double avgVel;
   double avgSize;
};
