#include "BlobPairing.h"

BlobPairing::DistanceMetric metric;

BlobPairing::BlobPairing(Blob* blob1, Blob* blob2)
{
    this->blob1 = blob1;
    this->blob2 = blob2;
}

float BlobPairing::centroidDistance() const
{
    return blob1->centroidDistanceTo(blob2);
}

int BlobPairing::overlap() const
{
    int _overlap = blob1->overlapWith(blob2);
    //cerr << "overlap: " << _overlap << endl;
    return _overlap;
}

int BlobPairing::sizeDifference() const
{
    int _sizeDifference = abs(blob1->size - blob2->size);
    //cerr << "size difference: " << _sizeDifference << endl;
    return _sizeDifference;
}

bool BlobPairing::operator<(const BlobPairing & rhs) const
{
    switch(metric)
    {
    case CentroidDistanceMetric:
        return centroidDistance() < rhs.centroidDistance();
    case CentroidAndSizeDistanceMetric:
        return centroidDistance() + sizeDifference() < rhs.centroidDistance() + rhs.sizeDifference();
    case OverlapDistanceMetric:
        return overlap() > rhs.overlap();
    }
}

void BlobPairing::setDistanceMetric(DistanceMetric theMetric)
{
    metric = theMetric;
}
