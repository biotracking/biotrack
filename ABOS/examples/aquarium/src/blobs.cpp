#include "blobs.h"
#include <iostream>

/**
  Compute and assign the slider values
  */
Blobs::Blobs(cv::Mat& flow, vector<vector<cv::Point> > contours, unsigned long long int frameNum) {
    this->contours = contours;
    this->frameNum = frameNum;
    num_blobs = contours.size();
    vector<vector<cv::Point> > allPts;
    for (unsigned int id = 0; id < contours.size(); id++)
    {
        /*int blob_size = cv::contourArea(cv::Mat(contours.at(id)));

      blob_size.push_back(cv::contourArea(cv::Mat(contours.at(id))));
      
      cv::Point centroid = getCentroid(contours.at(id));
      blob_pos.push_back(centroid);

      cv::Scalar color(rand()&255, rand()&255, rand()&255);

      blob_colors.push_back(color);*/

        Blob newBlob(contours.at(id));
        blobs.push_back(newBlob);

        // retrieve all points inside a contour.
        for (unsigned int id = 0; id < contours.size(); id++) {

            vector<cv::Point> points;
            cv::Mat temp_binary(cv::Size(flow.cols, flow.rows), CV_8UC1, cv::Scalar(0));
            cv::drawContours( temp_binary, contours, id, cv::Scalar(255), CV_FILLED, 8 );
            cv::Rect rect = cv::boundingRect(cv::Mat(contours.at(id)));
            for (int y = rect.y; y < rect.y + rect.height; y++) {
                for (int x = rect.x; x < rect.x + rect.width; x++) {

                    uchar& value = temp_binary.at<uchar>(y, x);
                    if ( value == cv::saturate_cast<uchar>(255)) {
                        points.push_back(cv::Point(x, y));
                    }
                }
            }
            allPts.push_back(points);
            points.clear();
        }
    }
    calcVelocity(flow, allPts);
    //cerr<<frameNum<<" "<<contours.size()<<"\n";

}

Blob* Blobs::operator[](int index)
{
    return & blobs[index];
}

Blob* Blobs::getBlobByLabel(int queryLabel)
{
    for(int b = 0; b < blobs.size(); b++)
    {
        if(blobs[b].label == queryLabel)
            return & blobs[b];
    }
    return NULL;
}

/*Blob* Blobs::getLargestBlob()
{
    int winningSize = -1;
    Blob* winningBlob = NULL;

    for(int b = 0; b < blobs.size(); b++)
    {
        if(blobs[b].size > winningSize)
            winningBlob = &blobs[b];
    }

    return winningBlob;
}

Blob* Blobs::getSecondLargestBlob()
{
    Blob* largestBlob = getLargestBlob();
    int winningSize = -1;
    Blob* winningBlob = NULL;

    for(int b = 0; b < blobs.size(); b++)
    {
        if(blobs[b].size > winningSize && & blobs[b] != largestBlob)
            winningBlob = & blobs[b];
    }

    return winningBlob;
}*/

struct BlobPointerCompare
{
  bool operator () (const Blob* a, const Blob* b)
  {
    if (a->size < b->size)
      return true;
    else
      return false;
  }
};

vector< Blob* > Blobs::getTwoLargestBlobsWithLabelsNotIn(set< int > labels)
{
    set< Blob*, BlobPointerCompare > usableBlobs;
    for(int b = 0; b < blobs.size(); b++)
    {
        if( labels.find(blobs[b].label) == labels.end() )
            usableBlobs.insert(&blobs[b]);
    }

    //cerr << "descending usable blob sizes: ";
    for(set< Blob*, BlobPointerCompare >::reverse_iterator b = usableBlobs.rbegin(); b != usableBlobs.rend(); b++)
    {
        //cerr << (*b)->size << ", ";
    }
    //cerr << endl;

    vector< Blob* > twoLargestUnmatchedBlobs;
    if(usableBlobs.size() > 0)
    {
        set< Blob*, BlobPointerCompare >::reverse_iterator it = usableBlobs.rbegin();
        if(it != usableBlobs.rend())
            twoLargestUnmatchedBlobs.push_back(*it);
        it++;
        if(it != usableBlobs.rend())
            twoLargestUnmatchedBlobs.push_back(*it);
    }

    return twoLargestUnmatchedBlobs;
}

double Blobs::computeAverageVelocity()
{
    double averageVelocity = 0.0;
    for(int v = 0; v < vel_avg.size(); v++)
    {
        double numberOfContributors = v+1;
        double newWeight = 1.0 / numberOfContributors;
        double previousWeight = 1.0 - newWeight;

        averageVelocity = averageVelocity * previousWeight + vel_avg[v] * newWeight;
    }

    return averageVelocity;
}

int Blobs::computeTotalArea()
{
    int totalArea = 0;
    for(int b = 0; b < blobs.size(); b++)
    {
        totalArea += blobs[b].size;
    }
    return totalArea;
}

int Blobs::numberOfMatchedBlobs()
{
    int matchedBlobs = 0;
    for(int b = 0; b < blobs.size(); b++)
    {
        if(blobs[b].matched)
            matchedBlobs++;
    }
    return matchedBlobs;
}

void Blobs::drawBlobsInto(cv::Mat& orig)
{
    for(int id = 0; id < blobs.size(); id++)
    {
        cv::drawContours( orig, contours, id, blobs[id].color, 5, 8 );
    }
}



/**
  Compute the average velocity for every blob
  */
void Blobs::calcVelocity( cv::Mat& flow, vector<vector<cv::Point> > allPts) {

    for (unsigned int i = 0; i < allPts.size(); i++) {
        double blobAvgVel = 0;
        vector<cv::Point> points = allPts.at(i);
        for (unsigned int j = 0; j < points.size(); j++) {
            cv::Point2f& fxy = flow.at<cv::Point2f>(points.at(j).y, points.at(j).x);
            double vel = sqrt(pow(fxy.x, 2) + pow(fxy.y, 2));
            blobAvgVel += vel;
        }
        if (points.size() > 0)
            blobAvgVel /= points.size();
        vel_avg.push_back(blobAvgVel);

    }
}

set<Blob*> Blobs::toSet()
{
    set<Blob*> blobsSet;
    for(int b = 0; b < blobs.size(); b++)
    {
        blobsSet.insert(&blobs[b]);
    }
    return blobsSet;
}

set<int> Blobs::getIndices()
{
    set<int> indices;
    for(int b = 0; b < num_blobs; b++)
    {
        indices.insert(b);
    }
    return indices;
}
