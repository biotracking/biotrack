#include "AbosThread.h"
#include "abospool.h"
#include "blobs.h"

/*
   This class will compute the slider values
*/
class SlidersThread : public AbosThread
{
public:
   SlidersThread(AbosPool* input_flow, AbosPool* input_mask, AbosPool* output, AbosPool* orig, list<Blobs> *sliders, int fps);
   void run();

private:

   AbosPool *readpool_flow, *readpool_mask, *writepool, *origpool;
   cv::Mat flowImg, maskImg, origImg;
   // shared vector to store slider values
   list<Blobs> *blobs_history;

   void init();
   void publishSliders(cv::Mat&, cv::Mat&, cv::Mat&, unsigned long long int);      
   void matchNewBlobsWithLastFrameBlobs();
};
