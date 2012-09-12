#include "ascribblearea.h"

#include <iostream>
#include <QtGui>
aScribbleArea::aScribbleArea(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StaticContents);
    modified = false;
    scribbling = false;
    targetpen=true;
    backgroundpen=false;
    backgroundmaskpen=false;
    myPenWidth = 5;
    myPenWidth++;
    myPenColor = Qt::blue;
    myPenColor =Qt::green;
    myBrushColor =Qt::gray;
    maskpathnewpath=true;
    clickedToEndPath=false;

}

bool aScribbleArea::openImage(const QString &fileName)
{
    QImage loadedImage;
    if (!loadedImage.load(fileName))
        return false;

    QSize newSize = loadedImage.size().expandedTo(size());
    resizeImage(&loadedImage, newSize);
    image = loadedImage;
    modified = false;
    update();
    return true;
}



void aScribbleArea::setPenColor(const QColor &newColor)
{
    myPenColor = newColor;
}

void aScribbleArea::setPenWidth(int newWidth)
{
    myPenWidth = newWidth;
}
void aScribbleArea::setBrushColor(const QColor &newColor)
{
    myBrushColor = newColor;
}

void aScribbleArea::clearImage()
{
    image.fill(qRgba(255, 255, 255,0));
    modified = true;
    maskpath= QPainterPath();
    clickedToEndPath=false;
    maskpathnewpath=true;

    update();
    std::cout<<"Cleared  "<<myPenWidth<<std::endl;
}

void aScribbleArea::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {

        clickedToEndPath=false;

        //For mask drawing
        if (backgroundmaskpen){
            if(maskpathnewpath){
                maskpath.moveTo(event->pos());
                maskpathnewpath=false;
                lastPoint = event->pos();

            }

            maskpath.lineTo(event->pos());
//            drawLineTo(QPoint(50,100));
            drawLineTo(event->pos());

            update();

        }
        //Default other drawing
        //        lastPoint = event->pos();
        if(targetpen||backgroundpen)
            scribbling = true;
        lastPoint = event->pos();


    }
    if (event->button()==Qt::RightButton&&backgroundmaskpen){
        //Close off current path
        closepath();
    }
}

void aScribbleArea::closepath(void)
{//Close off current path
    maskpath.closeSubpath();
    maskpathnewpath=true;
    clickedToEndPath=true;
    drawLineTo(lastPoint);
   update();
}

void aScribbleArea::mouseMoveEvent(QMouseEvent *event)
{
    if ((event->buttons() & Qt::LeftButton) && scribbling&&(targetpen||backgroundpen))
        drawLineTo(event->pos());
}

void aScribbleArea::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && scribbling&&(targetpen||backgroundpen)) {
        drawLineTo(event->pos());
        scribbling = false;
    }
}

void aScribbleArea::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    QRect dirtyRect = event->rect();
    painter.drawImage(dirtyRect, image, dirtyRect);
   // painter.save();
}
void aScribbleArea::drawLineTo(const QPoint &endPoint)
{

    std::cout<<"DRAWLINETO MyPenWidth=  "<<myPenWidth<<std::endl;
    QPainter painter(&image);
    painter.setPen(QPen(myPenColor, myPenWidth, Qt::SolidLine, Qt::RoundCap,
                        Qt::RoundJoin));
    painter.drawLine(lastPoint, endPoint);
    modified = true;
    int rad = (myPenWidth / 2) + 2;
    update(QRect(lastPoint, endPoint).normalized()
           .adjusted(-rad, -rad, +rad, +rad));

    //Path Test
    if(clickedToEndPath){
        QPainterPath invertPath;
        painter.setBrush(myBrushColor);

        invertPath.addRect(0,0,image.width(),image.height());
        invertPath.addPath(maskpath);

        painter.drawPath(invertPath);
    }
    //
    update();
    lastPoint = endPoint;
}

void aScribbleArea::resizeEvent(QResizeEvent *event)
{
    if (width() > image.width() || height() > image.height()) {
        //Earlier they wanted a safe margine that the image would draw outside of hence the Qmax
        int newWidth = qMax(width() + 0, image.width());
        int newHeight = qMax(height() + 0, image.height());
        std::cout<<"RESIZEDEVENT  "<<newHeight<<std::endl;

        resizeImage(&image, QSize(newWidth, newHeight));
        update();
    }
    QWidget::resizeEvent(event);
}



void aScribbleArea::resizeImage(QImage *image, const QSize &newSize)
{
    if (image->size() == newSize)
        return;

    QImage newImage(newSize, QImage::Format_ARGB32);
//    QImage newerImage(*image, newSize,QImage::Format_ARGB32);
    newImage.fill(qRgba(255, 0, 255,0));
    QPainter painter(&newImage);
    *image=image->scaled(newSize);
    painter.drawImage(QPoint(0, 0), *image);
    *image = newImage;
    std::cout<<"RESIZEDIMAGE  "<<newSize.height()<<std::endl;


}

bool aScribbleArea::saveImage(const QString &fileName, const char *fileFormat)
{

}
void aScribbleArea::print()
{
#ifndef QT_NO_PRINTER
    QPrinter printer(QPrinter::HighResolution);

    QPrintDialog *printDialog = new QPrintDialog(&printer, this);
    if (printDialog->exec() == QDialog::Accepted) {
        QPainter painter(&printer);
        QRect rect = painter.viewport();
        QSize size = image.size();
        size.scale(rect.size(), Qt::KeepAspectRatio);
        painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
        painter.setWindow(image.rect());
        painter.drawImage(0, 0, image);
    }
#endif // QT_NO_PRINTER
}
