#ifndef ASCRIBBLEAREA_H
#define ASCRIBBLEAREA_H

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QWidget>
#include <vector>
#include <QMessageBox>


using namespace std;


class PaintCanvas : public QWidget
{
    Q_OBJECT

public:
    PaintCanvas(QWidget *parent = 0);

    bool openImage(const QString &fileName);
    bool saveImage(const QString &fileName, const char *fileFormat);
    void setPenColor(const QColor &newColor);
    void setBrushColor(const QColor &newColor);

    void setPenWidth(int newWidth);

    bool isModified() const { return modified; }
    QColor penColor() const { return myPenColor; }
    int penWidth() const { return myPenWidth; }
    int myPenWidth;
    void drawLineTo(const QPoint &endPoint);
    void drawMarkerDot(const QPoint &endPoint);

    void resizeImage(QImage *image, const QSize &newSize);
    void closepath(void);


    bool modified;
    bool scribbling;
    bool targetpen;
    bool backgroundpen;
    bool backgroundmaskpen;
    bool markHead;
    bool markTail;
    QColor myPenColor;
    QColor myBrushColor;
    QPainterPath maskpath;
    QPainterPath temppath;

    vector<QPainterPath> masks;
    bool maskpathnewpath;
    bool clickedToEndPath;

    QImage image;
    QPoint lastPoint;
    vector<vector<QPoint> > polygons;
    vector<QPoint> temp;
    int numPoly;
    QMessageBox::StandardButton reply;

    //oldproto

public slots:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void clearImage(QPainterPath path);
    void clearMasks();
    void drawPaths();

    void print();

protected:


private:

};

#endif
// ASCRIBBLEAREA_H



