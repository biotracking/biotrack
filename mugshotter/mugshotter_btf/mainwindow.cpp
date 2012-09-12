/*
  Need to implement a feature that makes sure background image is same dimensions as video.
*/
#include "mainwindow.h"
#include "ui_mainwindow.h"
using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    nopath ="(None Selected)";
    lastpath="";
    vidpath=nopath;
    bgpath=nopath;
    projectSaveDirectory=nopath;
    isPlaying=false;
    isMugShoting=false;
    isDrawing=false;
    ui->setupUi(this);


    ui->user_prompt_label->setText("Choose a video to Mugshot");

    ui->videoButton->setEnabled(true);
    //connect(ui->videoButton, SIGNAL(clicked()),this,SLOT(on_videoButton_clicked()));

    ui->bgButton->setEnabled(true);
    //connect(ui->bgButton,SIGNAL(clicked()),this,SLOT(on_bgButton_clicked()));

    ui->saveButton->setEnabled(true);
    //connect(ui->saveButton,SIGNAL(clicked()),this,SLOT(on_saveButton_clicked()));

    ui->btfButton->setEnabled(true);

    ui->startButton->setEnabled(false);
    //connect(ui->startButton,SIGNAL(clicked()),this,SLOT(on_startButton_clicked()));

    ui->Imagelabel->setToolTip("Your video will load here");
    checkreadytoPlay();

    paintCanvas=ui->paintwidget;

    startTimer(1000/300000000);

}


MainWindow::~MainWindow()
{
    cvReleaseCapture(&capture);

    delete ui;
}

void MainWindow::on_videoButton_clicked()
{
    /*Select Video file*/
    vidpath= QFileDialog::getOpenFileName(this,tr("Open Video File"), lastpath, tr("Video (*.avi *.mov *.mpg *.mpeg)"));
    if ( vidpath.isNull() == false )
    {
        lastpath=vidpath;
        ui->videolabel->setText("..."+vidpath.right(50));
        checkreadytoPlay();

    }
    else{
        vidpath=nopath;
        ui->videolabel->setText(vidpath.right(50));
    }
}

void MainWindow::on_bgButton_clicked()
{
    /*Select Background file*/
    bgpath= QFileDialog::getOpenFileName(this, tr("Open Background File"),lastpath, tr("Images (*.png *.jpg)"));
    if ( bgpath.isNull() == false )
    {
        lastpath=bgpath;
        ui->bglabel->setText("..."+bgpath.right(50));
        checkreadytoPlay();

    }
    else{
        bgpath=nopath;
        ui->bglabel->setText(bgpath.right(50));
    }
}

void MainWindow::on_saveButton_clicked()
{
    /* select a directory using file dialog */
    projectSaveDirectory = QFileDialog::getExistingDirectory(this, tr("Select a save directory"),lastpath);

    if ( projectSaveDirectory.isNull() == false )
    {
        ui->savelabel->setText("..."+projectSaveDirectory.right(50)+"/");
        lastpath=projectSaveDirectory;
        projectSaveDirectory=projectSaveDirectory+"/";
        checkreadytoPlay();

    }
    else{
        projectSaveDirectory=nopath;
        ui->savelabel->setText("defaultdirectory/");
    }
}

void MainWindow::on_startButton_clicked()
{
    isPlaying = !isPlaying;
    processPolygons();
    // Need to implement mugshot.roi(centroids,maxXY,minXY)

    isMugShoting =!isMugShoting;
    QColor newColor = (QColor(100,100,100,100));
    paintCanvas->setPenColor(newColor);
    paintCanvas->setBrushColor(newColor);

}

bool MainWindow::checkreadytoPlay()
{
    QString notprepared="<b>Not Ready!</b>  <br><br><i>Please choose a...</i> <br>";
    if (vidpath==nopath)
        notprepared= notprepared+"   Video Source<br>";
    if (bgpath==nopath)
        notprepared= notprepared+"   Background Image <br>";
    if (projectSaveDirectory==nopath)
        notprepared= notprepared+"   & Directory to Save to<br>";
    if (bgpath!=nopath && vidpath!=nopath && projectSaveDirectory!=nopath){
        isDrawing=!isDrawing;
        loadMugShot();

        QString colour="green"; // you can use also QColor
        QString text="<b>Ready to start Mugshotting!</b> <br> <br> Press Play Button to Begin";
        QString fonttemplate = tr("<font color='%1'>%2</font>");

//        This should happen after parking space drawn
//        ui->user_prompt_label->setText(fonttemplate.arg( colour, text ));
//        ui->startButton->setEnabled(true);

        ui->paintwidget->setToolTip("Left click to add mask points | Right click to close masking shape");
        startPainter();

        return true;
    }
    else{

        QString colour="red"; // you can use also QColor
        QString text=notprepared;
        QString fonttemplate = tr("<font color='%1'>%2</font>");
        ui->user_prompt_label->setText(fonttemplate.arg( colour, text ));

        return false;
    }

}

// Setup Mugshot
void MainWindow::loadMugShot(){
    capture = cvCreateFileCapture(vidpath.toAscii());
    mugShotter = new Mugshot(vidpath.toAscii(), bgpath.toAscii(),projectSaveDirectory.toAscii());
    isPlaying = !isPlaying;

}

// Looped timer event
void MainWindow::timerEvent(QTimerEvent*) {
    QString polyinfo;

    //qDebug()<<paintCanvas->reply;

    if(paintCanvas->reply == QMessageBox::Yes)
    {
        paintCanvas->polygons.push_back(paintCanvas->temp);

        polyinfo = "Polygon added";

        ui->statusBar->showMessage(polyinfo,2000);
        paintCanvas->temp.clear();
        paintCanvas->reply = replyNull;


    } else if(paintCanvas->reply == QMessageBox::No){
        paintCanvas->temp.clear();

        //need to implement clear mask to masks.back()
        //paintCanvas->clearMasks();

        polyinfo = "Polygon cleared";

        ui->statusBar->showMessage(polyinfo,2000);

        paintCanvas->reply = replyNull;
    }



    if (!isPlaying) return;

// Debug Note: Replace updatePng() with updateFrame() for video
    IplImage* img;
    if(isMugShoting){
        img = updateFrame();
    }else{
        img = updatePng();
    }
    updateImage(img);
    //TODO:
    //    updateStatusBar();



}
/*Update intended to display single image once*/
IplImage* MainWindow::updatePng()
{
    IplImage* img;
    img = cvLoadImage(bgpath.toAscii());
    isPlaying=!isPlaying;

    return img;
}

/*Update intended for frames from video*/

IplImage* MainWindow::updateFrame()
{
    IplImage* img;
    // if the current frame is less than total frames minus 2 , keep analysing
    if (cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT)-2 > cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES))
    {

        img = cvQueryFrame(capture);
        mugShotter->loop(img);
    }
    else
    {
        cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES, cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT)-1);
    }

    return img;
}



void MainWindow::updateImage(IplImage *cvimage){
    qimage = QImage((const uchar*)cvimage->imageData, cvimage->width, cvimage->height, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();
    //qDebug()<<"width: "<<qimage.width()<<" height: "<<qimage.height();
    qimage = qimage.scaledToWidth(640);
    //qDebug()<<"New width: "<<qimage.width()<<" New height: "<<qimage.height();
    ui->Imagelabel->setPixmap(QPixmap::fromImage(qimage));

    return;
}



void MainWindow::startPainter()
{
    paintCanvas->backgroundmaskpen=true;
    QColor newColor = (QColor(150,150,150,150));

    paintCanvas->setPenColor(newColor);
    paintCanvas->setBrushColor(newColor);
    int newWidth=3;
    paintCanvas->setPenWidth(newWidth);

    ui->statusBar->showMessage(QString::fromStdString("Left click to add mask points | Right click to close masking shape"),6000);
    ui->parkinglabel->setText("Pick ROI (Regions of Interest) by making polygons");

}


void MainWindow::on_resetButton_clicked()
{
    ui->confirmButton->setEnabled(true);
    ui->startButton->setEnabled(false);
    ui->paintwidget->setEnabled(true);
    paintCanvas->clearMasks();


}

void MainWindow::on_confirmButton_clicked()
{
    ui->parkinglabel->setText("");
    ui->startButton->setEnabled(true);
    ui->paintwidget->setEnabled(false);
    ui->resetButton->setEnabled(true);
    ui->confirmButton->setEnabled(false);
}

void MainWindow::processPolygons()
{
    x=0;
    y=0;
    avgX=0;
    avgY=0;
    xTot=0;
    yTot=0;
    maxX=0;
    maxY=0;
    minX=1000000;
    minY=1000000;


    for (row = paintCanvas->polygons.begin(); row != paintCanvas->polygons.end(); ++row) {
        for (col = row->begin(); col != row->end(); ++col) {
            tempV =*row;
            tempPoint = *col;
            x = tempPoint.x();
            y = tempPoint.y();
            maxX = max(maxX,x);
            maxY = max(maxY,y);
            minX = min(minX,x);
            minY = min(minY,y);
            xTot += tempPoint.x();
            yTot += tempPoint.y();


//            qDebug()<<"point: "<<x<<" , "<<y;
//            qDebug()<<"total: "<<xTot<<" , "<<yTot;

            if(tempPoint==tempV.back()){
                avgX=xTot/tempV.size();
                avgY=yTot/tempV.size();
                tempCentroid.setX(avgX*3);
                tempCentroid.setY(avgY*3);
                centroids.push_back(tempCentroid);
                tempMax.setX(maxX*3);
                tempMax.setY(maxY*3);
                maxXY.push_back(tempMax);
                tempMin.setX(minX*3);
                tempMin.setY(minY*3);
                minXY.push_back(tempMin);
//                qDebug()<<"Testing tempV.back(): "<<tempV.back();
//                qDebug()<<"Testing tempV.size(): "<<tempV.size();
//                qDebug()<<"avgX: "<<avgX<<" avgY: "<<avgY;
//                qDebug()<<"max X Y: "<<maxXY.back()<<" min X Y: "<<minXY.back();

// Reset values after each polygon has been added.
                mugShotter->uCent = centroids;
                mugShotter->uMax = maxXY;
                mugShotter->uMin = minXY;

                //DEBUGGING Draw centroid on screen
//                tempCentroid.setX(avgX);
//                tempCentroid.setY(avgY);
//                paintCanvas->setPenColor(Qt::green);
//                paintCanvas->drawMarkerDot(tempCentroid);

                xTot=0;
                yTot=0;
                maxX=0;
                maxY=0;
                minX=1000000;
                minY=1000000;

            }
        }
    }

////Debugging : Print out vectors
//    for (vector<QPoint>::iterator it = centroids.begin(); it!=centroids.end(); ++it) {
//        qDebug() << *it;
//    }

}
