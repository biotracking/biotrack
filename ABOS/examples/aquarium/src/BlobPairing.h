#ifndef BLOBPAIRING_H
#define BLOBPAIRING_H

#include "Blob.h"

class BlobPairing
{
public:
    BlobPairing(Blob* blob1, Blob* blob2);

    float centroidDistance() const;
    int overlap() const;
    int sizeDifference() const;

    bool operator<(const BlobPairing & rhs) const;

    Blob* blob1;
    Blob* blob2;


    enum DistanceMetric { CentroidDistanceMetric, CentroidAndSizeDistanceMetric, OverlapDistanceMetric };
    static void setDistanceMetric(DistanceMetric metric);
};

#endif // BLOBPAIRING_H
