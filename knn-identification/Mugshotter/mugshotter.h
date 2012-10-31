#ifndef MUGSHOTTER_H
#define MUGSHOTTER_H

#include "/home/stephen/OpenCV-2.3.1/include/opencv/cv.h"
#include "/home/stephen/OpenCV-2.3.1/include/opencv/highgui.h"
using namespace cv;

class Mugshotter{
public:
    //void show(){return;}; //legacy garbage
private:
	Mat rotateImage(const Mat& source, double anglerad);
	int main(int argc, char *argv[]);
};

template <class T> class Image
{
private:
  IplImage *imgp;
public:
  Image(IplImage *img = 0) {imgp = img;}
  ~Image() {imgp = 0;}
  void operator = (IplImage *img){imgp = img;}
  inline T* operator[](const int rowIndx){
      return ((T*)(imgp->imageData + rowIndx *imgp ->widthStep));}
};
typedef struct{
  unsigned char b, g, r;
}RgbPixel;

typedef struct{
  float b, g, r;
}RgbPixelFloat;

typedef Image<RgbPixel>      RgbImage;
typedef Image<RgbPixelFloat> RgbImageFloat;
typedef Image<unsigned char> BwImage;
typedef Image<float>         BwImageFloat;

#endif
