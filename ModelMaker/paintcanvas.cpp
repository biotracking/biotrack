#include "paintcanvas.h"

#include "modelmaker.h"
#include "paintcanvas.h"
#include <iostream>
#include <QMessageBox>
#include <QtGui>


PaintCanvas::PaintCanvas(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StaticContents);
    modified = false;
    scribbling = false;
    targetpen=false;
    backgroundpen=false;
    backgroundmaskpen=false;
    markHead=false;
    markTail=false;
    myPenWidth = 1;
    myPenWidth++;
    myPenColor = Qt::blue;
    myPenColor =Qt::green;
    myBrushColor =Qt::gray;
    maskpathnewpath=true;
    clickedToEndPath=false;
    numPoly=0;
    rad=0;
    drawingPolygon=false;
    drawingCirRects=false;
    settingupCir=false;
    hasClicked =false;
    gettingCenter =false;
    gettingHead=false;
    image.scaled(800,450);


    varRectx=100;
    varRecty=40;
   // qApp->installEventFilter(this);
    this->setMouseTracking(true);


}

bool PaintCanvas::openImage(const QString &fileName)
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



void PaintCanvas::setPenColor(const QColor &newColor)
{
    myPenColor = newColor;

}

void PaintCanvas::setPenWidth(int newWidth)
{
    myPenWidth = newWidth;
}
void PaintCanvas::setBrushColor(const QColor &newColor)
{
    myBrushColor = newColor;
}

void PaintCanvas::clearVisual(){

    image.fill(qRgba(255, 255, 255,0));
    update();
}

void PaintCanvas::clearImage()
{
    image.fill(qRgba(255, 255, 255,0));
    modified = true;
    maskpath= QPainterPath();
    temppath = QPainterPath();
    clickedToEndPath=false;
    maskpathnewpath=true;
    update();
}


//bool PaintCanvas::eventFilter(QObject *obj, QEvent *event)
//{
//  if (event->type() == QEvent::MouseMove)
//  {
//    QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
//    qDebug()<<"Mouse move () "<<mouseEvent->pos().x()<<"  "<<mouseEvent->pos().y();
//  }
//  return false;
//}


void PaintCanvas::mousePressEvent(QMouseEvent *event)
{
    hasClicked = true;
    if(drawingPolygon){
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
                drawLineTo(event->pos());
                temp.push_back(lastPoint);
                update();

            }

            //Default other drawing
            if(targetpen||backgroundpen)
                scribbling = true;
            lastPoint = event->pos();


        }
        if (event->button()==Qt::RightButton&&backgroundmaskpen){
            //Close off current path
            closepath();
            //replyMask = QMessageBox::question(this, "Commit Polygon", "Do you want to create a model from this polygon?", QMessageBox::Yes|QMessageBox::Default, QMessageBox::No|QMessageBox::Escape);
            QMessageBox msgBox;
            msgBox.setText(trUtf8("Commit Polygon"));
           // msgBox.setInformativeText(record.value(cd_titulo).toString());
            msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
            msgBox.setDefaultButton(QMessageBox::Yes);

            replyMask=(QMessageBox::StandardButton)msgBox.exec();

        }
    }
//    else if(drawingCirRects){
//        if (event->button() == Qt::LeftButton) {
//            if (backgroundmaskpen){
//                if(maskpathnewpath){
//                   // maskpath.addEllipse(event->pos().x(),event->pos().y(),20,20);
//                    //qDebug()<<QString("Mouse move (%1,%2)").arg(mouseEvent->pos().x()).arg(mouseEvent->pos().y());
//                   centerCir.setX(event->pos().x());
//                   centerCir.setY(event->pos().y());
//                }
//            }
//        }
//    }


        if(gettingCenter){
centerPoint=event->pos();
        replyCenter = (QMessageBox::StandardButton)QMessageBox::question(this, "Confirm Center", "Do you want to confirm the center location?",QMessageBox::Yes|QMessageBox::Default, QMessageBox::No|QMessageBox::Escape);

//        clearImage();
//        maskpath.moveTo(lastPoint);

//        drawLineTo(event->pos());
//        update();

    }


            if(gettingHead){
headPoint=event->pos();
        replyHead = (QMessageBox::StandardButton)QMessageBox::question(this, "Confirm Head", "Do you want to confirm the head location?", QMessageBox::Yes|QMessageBox::Default, QMessageBox::No|QMessageBox::Escape);


    }


}

void PaintCanvas::closepath(void)
{//Close off current path
    maskpath.closeSubpath();
    maskpathnewpath=true;
    clickedToEndPath=true;
    drawLineTo(lastPoint);
    if (drawingPolygon){
        masks.push_back(maskpath);
    }else if(drawingCirRects){
        radSQ.push_back(maskpath);
    }
    update();
}

//NOTE you must call this.setMouseTracking(true) for this function to work!
void PaintCanvas::mouseMoveEvent(QMouseEvent *event)
{
   // clearImage();
    clearVisual();

            QPainter headpainter(&image);
            headpainter.setPen(QPen(QColor(000,0,255,100), myPenWidth, Qt::SolidLine, Qt::RoundCap,
                                Qt::RoundJoin));
            headpainter.drawEllipse(event->pos(),4,4);//drawLine(lastPoint, endPoint);
            headpainter.drawPath(maskpath);
           // headpainter.fillPath();


  //  drawLineTo(event->pos());

            update();

    if(gettingCenter == true){
        clearImage();
        maskpath.moveTo(lastPoint);
        //drawLineTo(event->pos());
        qDebug()<<event->pos();


        //QPainter headpainter(&image);
        headpainter.setPen(QPen(QColor(200,0,255,200), myPenWidth, Qt::SolidLine, Qt::RoundCap,
                            Qt::RoundJoin));
        headpainter.drawEllipse(event->pos(),2,2);//drawLine(lastPoint, endPoint);

    }

    if(gettingHead == true)
    {
        clearImage();
        //drawLineTo(event->pos());
        qDebug()<<event->pos();


        //QPainter headpainter(&image);
        headpainter.setPen(QPen(QColor(200,255,0,200), myPenWidth, Qt::SolidLine, Qt::RoundCap,
                            Qt::RoundJoin));
        headpainter.drawEllipse(event->pos(),3,3);//drawLine(lastPoint, endPoint);
    }

    if ((event->buttons() & Qt::LeftButton) && scribbling&&(targetpen||backgroundpen)){
       // drawLineTo(event->pos());
    }
//    else if ((event->buttons() & Qt::LeftButton) && drawingCirRects){
//         //maskpath.addEllipse();
//        //clearImage();
//        rad = max(abs(centerCir.x()-(event->pos().x())),abs(centerCir.y()-(event->pos().y())));
//        maskpath.addEllipse(centerCir,rad,rad);
//        //qDebug()<<QString("(%1,%2)").arg(event->pos().x()).arg(event->pos().y());
//        // closepath();
//    }


}

void PaintCanvas::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && scribbling&&(targetpen||backgroundpen)) {
        drawLineTo(event->pos());
        scribbling = false;
    } else if (event->button() == Qt::LeftButton && drawingCirRects){
//        clearImage();
//        qreal rad = max(abs(centerCir.x()-(event->pos().x())),abs(centerCir.y()-(event->pos().y())));
//        maskpath.addEllipse(centerCir,rad,rad);
//        qDebug()<<QString("radius set to (%1,%2)").arg(event->pos().x()).arg(event->pos().y());
//        closepath();
////        maskpath.addRect(centerCir.x()+event->pos().x(),centerCir.y()-30,60,100);
////        closepath();
//        settingupCir=true;
    } else if(gettingCenter){
//        orPoint = event->pos();
//        maskpath.moveTo(lastPoint);
//        drawLineTo(event->pos());
//        replyOrientation = QMessageBox::question(this, "Confirm Orientation", "Do you want to confirm the orientation?", QMessageBox::Yes|QMessageBox::No);

    }
}


void PaintCanvas::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    QRect dirtyRect = event->rect();
    painter.drawImage(dirtyRect, image, dirtyRect);
    QPen pen1(myPenColor, myPenWidth, Qt::SolidLine, Qt::RoundCap,Qt::RoundJoin);
    painter.setPen(pen1);

    painter.setBrush(myBrushColor);

    //Path Test
    if(clickedToEndPath){
        clearImage();
    }
    myBrushColor.setAlpha(100);
    for (vector<QPainterPath>::iterator it = masks.begin(); it!=masks.end(); ++it) {
        temppath = *it;
        painter.drawPath(temppath);
    }

    QPen pen2(myPenColor, 2, Qt::DashLine, Qt::RoundCap,Qt::RoundJoin);
    painter.setPen(pen2);
    myBrushColor.setAlpha(0);

    for (vector<QPainterPath>::iterator it = radSQ.begin(); it!=radSQ.end(); ++it) {
        temppath = *it;
        painter.drawPath(temppath);
    }

}


void PaintCanvas::drawLineTo(const QPoint &endPoint)
{

    QPainter painter(&image);
    painter.setPen(QPen(myPenColor, myPenWidth, Qt::SolidLine, Qt::RoundCap,
                        Qt::RoundJoin));
    painter.drawLine(lastPoint, endPoint);
    modified = true;
    int rad = (myPenWidth / 2) + 2;




   // QPainter headpainter(&image);
    painter.setPen(QPen(QColor(200,0,255,200), myPenWidth, Qt::SolidLine, Qt::RoundCap,
                        Qt::RoundJoin));
    painter.drawEllipse(endPoint,2,2);//drawLine(lastPoint, endPoint);
    update();


//    update(QRect(lastPoint, endPoint).normalized()
//           .adjusted(-rad, -rad, +rad, +rad));

    //Path Test
    if(clickedToEndPath){
       // painter.setBrush(myBrushColor);
     //   painter.drawPath(maskpath);

    }
    //
    update();
    lastPoint = endPoint;
}

void PaintCanvas::resizeEvent(QResizeEvent *event)
{
//    if (width() > image.width() || height() > image.height()) {
        //Earlier they wanted a safe margine that the image would draw outside of hence the Qmax
        int newWidth = qMax(width() + 0, image.width());
        int newHeight = qMax(height() + 0, image.height());
        //std::cout<<"RESIZEDEVENT  "<<newHeight<<std::endl;

        resizeImage(&image, QSize(newWidth, newHeight));
        update();
//    }
    QWidget::resizeEvent(event);
}



void PaintCanvas::resizeImage(QImage *image, const QSize &newSize)
{
    if (image->size() == newSize)
        return;

    QImage newImage(newSize, QImage::Format_ARGB32);
    newImage.fill(qRgba(255, 0, 255,0));
    QPainter painter(&newImage);
    *image=image->scaled(newSize);
    painter.drawImage(QPoint(0, 0), *image);
    *image = newImage;


}

void PaintCanvas::clearMasks(){
    masks.clear();
    temp.clear();
    polygons.clear();
    polyNames.clear();

    clearImage();

}

void PaintCanvas::print()
{
#ifndef QT_NO_PRINTER
//    QPrinter printer(QPrinter::HighResolution);

//    QPrintDialog *printDialog = new QPrintDialog(&printer, this);
//    if (printDialog->exec() == QDialog::Accepted) {
//        QPainter painter(&printer);
//        QRect rect = painter.viewport();
//        QSize size = image.size();
//        size.scale(rect.size(), Qt::IgnoreAspectRatio
//                   //,Qt::KeepAspectRatio
//                   );
//        painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
//        painter.setWindow(image.rect());
//        painter.drawImage(0, 0, image);
    }
#endif // QT_NO_PRINTER
//}

void PaintCanvas::drawPaths()
{
    for (vector<QPainterPath>::iterator it = masks.begin(); it!=masks.end(); ++it) {
        temppath = *it;
    //    painter.drawPath(temppath);
    }
}

void PaintCanvas::setTip(QString text)
{

}

void PaintCanvas::addRect(vector<QPoint> sqPolygon)
{

}

void PaintCanvas::arrow()
{

}




