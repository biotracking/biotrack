#ifndef BLOB_H
#define BLOB_H

#include <vector>

#include <opencv/cv.h>

#include <QPolygon>

using namespace std;

class Blob
{
public:
    Blob(vector< cv::Point > contour);

    int size;
    cv::Point pos;
    cv::Point centroid;
    unsigned long label;
    vector< cv::Point > path;
    bool matched;
    cv::Scalar color;

    float centroidDistanceTo(Blob* other);
    int overlapWith(Blob* other);


private:
    cv::Point computeCentroid(vector<cv::Point>);
    QPolygon getPolygon();
};




#endif // BLOB_H
