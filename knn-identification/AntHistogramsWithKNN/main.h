#ifndef MAIN_H
#define MAIN_H

#include <QtCore>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include "/usr/share/qt4/include/Qt/qdom.h"
#include "/home/stephen/OpenCV-2.3.1/include/opencv/cv.h"
#include "/home/stephen/OpenCV-2.3.1/include/opencv/highgui.h"
#include <iostream>
#include <fstream>

using namespace std;

//////////////////////
//    Parameters    //
//////////////////////

const int K = 5;

const int maxFrames = 22000;
const int numParkingSpots = 40;

const int bgSimilarity = 5;
const int binSize = 7;

const int maxImageRows = 400;
const int maxImageCols = 400;

//////////////////////
//     Functions    //
//////////////////////

int main(int argc, char *argv[]);

bool similar(int x, int y, int threshold){
	return abs(x-y)<=threshold && abs(y-x)<=threshold;
}

void identifyClip(int clipNum);

class UVHistogram{
public:
	int agentId;
	double UValues[256];//can be between -128 and 128
	double VValues[256];//can be between -128 and 128
	double intersectionWith(UVHistogram other){
		double Uintersection = 0;
		double Vintersection = 0;
		for(int i=0; i<256; i++){
			Uintersection += (other.UValues[i] < UValues[i])?other.UValues[i]:UValues[i];
			Vintersection += (other.VValues[i] < VValues[i])?other.VValues[i]:VValues[i];
		}
		return sqrt(Uintersection*Uintersection + Vintersection*Vintersection)/sqrt(2);
	}
	void removeLargestPeak(){
		const int bgVariationRadius = 3;
		int largestPeakIndex[2] = {0, 0};
		double largestPeakValue[2] = {UValues[0], VValues[0]};
		for(int i=1; i<256; i++){
			if(UValues[i]>largestPeakValue[0])
				largestPeakIndex[0]=i;
			if(VValues[i]>largestPeakValue[1])
				largestPeakIndex[1]=i;
		}
		for(int i=largestPeakIndex[0]-bgVariationRadius; i<=largestPeakIndex[0]+bgVariationRadius; i++){
			if(i>=0 && i < 256)
				UValues[i] = 0;
		}
		for(int i=largestPeakIndex[1]-bgVariationRadius; i<=largestPeakIndex[1]+bgVariationRadius; i++){
			if(i>=0 && i < 256)
				VValues[i] = 0;
		}
	}
	void normalize(){
		int totalSumU = 0;
		int totalSumV = 0;
		for(int x=0; x<256; x++){
			totalSumU += UValues[x];
			totalSumV += VValues[x];
		}
		for(int x=0; x<256; x++){
			UValues[x] /= totalSumU;
			VValues[x] /= totalSumV;
		}
	}
};

//////////////////////
// Type Definitions //
//////////////////////

typedef struct{
	char y, u, v;
}YuvPixel;

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
  int width(){return imgp->width;}
  int height(){return imgp->height;}
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
