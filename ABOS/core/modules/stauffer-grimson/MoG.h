#ifndef MOG_H
#define MOG_H

#include "GaussianDist.h"
#include <cv.h>


class MoG
{
public:
	MoG(int size, int nChannels, int k, double alpha, double T);
    MoG(int cols, int rows, int nChannels, int k, double alpha, double T);    // size will be set according to cols*rows
    void init(double alpha, double T);

    ~MoG();

    void updateMoG( cv::Mat *frame );
    void setBGFrame( cv::Mat frame );

    static cv::Mat stauffer_grimson(cv::Mat img, int k, int nChannels, double alpha, double T);

private:
    int size;       // size of a frame (typically, rows*cols for images)
    int nChannels;  // number of channels
    int k;          // number of Gaussians in a mixture
    GaussianDist *dists;

    double T_PORTION;
};

#endif
