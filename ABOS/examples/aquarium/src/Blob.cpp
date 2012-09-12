#include "Blob.h"
#include <qmath.h>


Blob::Blob(vector< cv::Point > contour)
{
    static unsigned long uniqueLabel = 0;

    size = cv::contourArea(cv::Mat(contour));
    centroid = computeCentroid(contour);
    color = cv::Scalar(rand()&255, rand()&255, rand()&255);
    path = contour;
    matched = false;
    label = uniqueLabel++;

    //cerr << "labeled blob " << label << endl;
}

/**
  Given a set of points, return the centroid
  */
cv::Point Blob::computeCentroid(vector<cv::Point> contour) {
    unsigned int i = 0;
    int x = 0, y = 0;
    while (i < contour.size()) {
        cv::Point point = contour.at(i);
        x += point.x;
        y += point.y;
        i++;
    }
    cv::Point centroid = cv::Point(x / contour.size(), y / contour.size());
    return centroid;
}

float Blob::centroidDistanceTo(Blob* other)
{
    float dX = other->centroid.x - centroid.x;
    float dY = other->centroid.y - centroid.y;

    return qSqrt( dX * dX + dY * dY );
}

vector< cv::Point > QtPolyToCVContour(const QPolygon & qtPolygon)
{
    vector< cv::Point > cvContour;
    for(int p = 0; p < qtPolygon.size(); p++)
    {
        cv::Point point;
        point.x = qtPolygon[p].x();
        point.y = qtPolygon[p].y();
        cvContour.push_back(point);
    }
    return cvContour;
}

int Blob::overlapWith(Blob* other)
{
    QPolygon intersectionPoly = getPolygon().intersected(other->getPolygon());
    if(intersectionPoly.size() < 2)
        return 0;
    vector< cv::Point > intersectionContour = QtPolyToCVContour(intersectionPoly);
    int area = cv::contourArea(intersectionContour);
    return area;
}

QPolygon Blob::getPolygon()
{
    QVector<QPoint> qtPointVector;

    for(int p = 0; p < path.size(); p++)
    {
        QPoint point(path[p].x, path[p].y);
        qtPointVector.push_back(point);
    }

    return QPolygon(qtPointVector);
}
