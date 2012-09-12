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
    mugShotter = NULL;
    paintCanvas = NULL;
    nopath ="(None Selected)";
    lastpath="";
    vidpath=nopath;
    bgpath=nopath;
    projectSaveDirectory=nopath;
    isPlaying=false;
    isMugShoting=false;
    isDrawing=false;
    ui->setupUi(this);
    font.SansSerif;
    font.PreferDefault;
    loadedPolygons=false;




    ui->user_prompt_label->setText("Choose ar video to Mugshot");

    ui->videoButton->setEnabled(true);
    //connect(ui->videoButton, SIGNAL(clicked()),this,SLOT(on_videoButton_clicked()));

    ui->bgButton->setEnabled(true);
    //connect(ui->bgButton,SIGNAL(clicked()),this,SLOT(on_bgButton_clicked()));

    ui->saveButton->setEnabled(true);
    //connect(ui->saveButton,SIGNAL(clicked()),this,SLOT(on_saveButton_clicked()));

    ui->startButton->setEnabled(false);
    //connect(ui->startButton,SIGNAL(clicked()),this,SLOT(on_startButton_clicked()));

    ui->Imagelabel->setToolTip("Your video will load here");

    checkreadytoPlay();

    ui->subviewButton->setEnabled(false);
    ui->thresholdSpinBox->setValue(10);
    paintCanvas=ui->paintwidget;
    ui->scrubSlider->setEnabled(false);
    ui->progressBar->setVisible(false);
    ui->progresslabel->setVisible(false);
    ui->progresslabel->setText("Analyzing Video...");
    connect(ui->actionLoad_Polygons,SIGNAL(triggered()),this,SLOT(loadPolygons()));
    ui->skipSpinBox->setEnabled(false);
    ui->skipSpinBox->setValue(1);
    startTimer(1000/300000000);
//    QPixmap pixmap("/home/blacki/Dropbox/Qt/polygon.png");
//    QIcon polyicon(pixmap);
//    ui->polygonButton->setIcon(polyicon);

    ui->polygonButton->setIcon(QIcon("polygon.png"));
    ui->cirRectButton->setIcon(QIcon("circle.png"));

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
    isMugShoting =!isMugShoting;
    QColor newColor = (QColor(100,100,100,100));
    paintCanvas->setPenColor(newColor);
    paintCanvas->setBrushColor(newColor);
    ui->progressBar->setVisible(true);
    ui->progresslabel->setVisible(true);
    ui->scrubSlider->setEnabled(false);

}

bool MainWindow::checkreadytoPlay()
{
    QString notprepared="<b>Not Ready!</b>  <br><br><i>Please choose a...</i> <br>";
// Debugging uncomment to not have to load vids everytime
//    if (vidpath==nopath)
//        notprepared= notprepared+"   Video Source<br>";
//        vidpath="/mnt/hgfs/Ants/2012_AntFeeder/3.21.12_B144_0.1M/B144_3.21.12_0.1M_clip3_background.png";
//    if (bgpath==nopath)
//        notprepared= notprepared+"   Background Image <br>";
//        bgpath = "/mnt/hgfs/Ants/2012_AntFeeder/3.21.12_B144_0.1M/B144_3.21.12_0.1M_clip3_1088_background.png.png";
    if (projectSaveDirectory==nopath)
        notprepared= notprepared+"   & Directory to Save to<br>";
       // projectSaveDirectory = "/media/PC_100/ants/2.17.12_feedertrials/temp/";
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
        ui->subviewButton->setEnabled(true);
        ui->scrubSlider->setEnabled(true);
        ui->skipSpinBox->setEnabled(true);
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
    if(paintCanvas==NULL) return;

    if(paintCanvas->reply == QMessageBox::Yes)
    {
        paintCanvas->polygons.push_back(paintCanvas->temp);
        bool ok;
        QString text = QInputDialog::getText(this, "ROI Name", "Attach a name to this Region Of Interest", QLineEdit::Normal, "name",&ok);


        //reimplement centroid calculation and fix font.
        //paintCanvas->maskpath.addText(paintCanvas->temp.back().x(),paintCanvas->temp.back().y(),font,text);

        paintCanvas->polyNames.push_back(text);
        polyinfo = text;
        polyinfo += " Region added";
        ui->statusBar->showMessage(polyinfo,2000);
        paintCanvas->temp.clear();
        paintCanvas->reply = replyNull;


    } else if(paintCanvas->reply == QMessageBox::No){

        paintCanvas->masks.pop_back();
        polyinfo = "Polygon cleared";
        paintCanvas->temp.clear();
        ui->statusBar->showMessage(polyinfo,2000);
        paintCanvas->reply = replyNull;
    } else if(paintCanvas->settingupCir){
        createCirParams(paintCanvas->centerCir.x(),paintCanvas->centerCir.y());
    }

    IplImage* img;

    if (!isPlaying) return;

    // Debug Note: Replace updatePng() with updateFrame() for video
        if(isMugShoting){
            img = updateFrame(img);
        }else{
            img = updatePng(img);
        }

        updateImage(img);
        //TODO:
        //    updateStatusBar();
        //cvReleaseImage(&img);


}
/*Update intended to display single image once*/
IplImage* MainWindow::updatePng(IplImage* img)
{
    img = cvLoadImage(bgpath.toAscii());
    isPlaying=!isPlaying;

    return img;
}

/*Update intended for frames from video*/

IplImage* MainWindow::updateFrame(IplImage* img)
{

    // if the current frame is less than total frames minus 2 , keep analysing
    if (cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT)-2 > cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES))
    {
        cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES,mugShotter->currentFrame);

        img = cvQueryFrame(capture);
        //bug's in here
        mugShotter->loop(img);
        ui->statusBar->showMessage("Frame: "+QString::number(cvGetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES))+"/"+QString::number(cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT)));

        if(mugShotter->retSubtract){
            img = mugShotter->subtractBack(img);
        }
    }
    else
    {
        cvSetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES, cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT)-1);
    }

    return img;
}



void MainWindow::updateImage(IplImage *cvimage){

    qimage = QImage((const uchar*)cvimage->imageData, cvimage->width, cvimage->height   , QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();
    qimage= qimage.scaled(QSize(640,363));
    //qimage = qimage.scaledToWidth(640);

    ui->Imagelabel->setPixmap(QPixmap::fromImage(qimage));
    //ui->Imagelabel->resize(QSize(640,363));
    ui->progressBar->setValue((int)(100*(1-(cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_COUNT)-cvGetCaptureProperty(capture,CV_CAP_PROP_POS_FRAMES))/cvGetCaptureProperty(capture,CV_CAP_PROP_FRAME_COUNT))));


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

    ui->statusBar->showMessage(QString::fromStdString("Left click to add points | Right click to close shape"),6000);
    ui->parkinglabel->setText("Pick ROI (Regions of Interest) by making polygons");

}


void MainWindow::on_resetButton_clicked()
{
    ui->confirmButton->setEnabled(true);
    ui->startButton->setEnabled(false);
    ui->paintwidget->setEnabled(true);
    paintCanvas->clearMasks();
    paintCanvas->reply = replyNull;



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
    mugShotter->polygons = paintCanvas->polygons;

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


            qDebug()<<"point: "<<x<<" , "<<y;
            qDebug()<<"total: "<<xTot<<" , "<<yTot;

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
                qDebug()<<"Testing tempV.back(): "<<tempV.back();
                qDebug()<<"Testing tempV.size(): "<<tempV.size();
                qDebug()<<"avgX: "<<avgX<<" avgY: "<<avgY;
                qDebug()<<"max X Y: "<<maxXY.back()<<" min X Y: "<<minXY.back();

// Reset values after each polygon has been added.
                mugShotter->uCent = centroids;
                mugShotter->uMax = maxXY;
                mugShotter->uMin = minXY;
                mugShotter->polyNames = paintCanvas->polyNames;

                //DEBUGGING Draw centroid on screen
                tempCentroid.setX(avgX);
                tempCentroid.setY(avgY);

                xTot=0;
                yTot=0;
                maxX=0;
                maxY=0;
                minX=1000000;
                minY=1000000;

            }
        }
    }
    mugShotter->writePoly();

}

void MainWindow::loadSettings()
{

}

void MainWindow::on_subviewButton_toggled(bool checked)
{
    mugShotter->retSubtract =checked;
    if(checked){
        ui->subviewButton->setText("View Normal");
    } else {
        ui->subviewButton->setText("View Subtraction");
    }
}

void MainWindow::on_thresholdSpinBox_valueChanged(int arg1)
{
    if(mugShotter!=NULL){
        mugShotter->threshold = arg1*1000;
    }
}

void MainWindow::on_scrubSlider_valueChanged(int value)
{
    IplImage* temp;
    scrubRatio = (double)value/100.00;
    mugShotter->currentFrame =cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT);
    double jumpto = (scrubRatio*((double)mugShotter->currentFrame));
    mugShotter->currentFrame = jumpto;
    //qDebug()<<scrubRatio<<" j "<<jumpto;
    isScrubbing=true;
    cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES, jumpto);
    temp = cvQueryFrame(capture);
    if(mugShotter->retSubtract){
        temp = mugShotter->subtractBack(temp);
    }
    updateImage(temp);


    ui->statusBar->showMessage("Frame: "+QString::number((int)jumpto)+"/"+QString::number(cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_COUNT)));
}

void MainWindow::loadPolygons()
{
    //must have set enabled after we are insured a path exists

    //need to make slot trigger user to find polygons.text and need to insure that process polygons is not called twice on the same file.
    //    polygonspath= QFileDialog::getOpenFileName(this, tr("Open Polygons File"),lastpath, tr("Polygons.txt  (*.png *.jpg)"));


    QFile file("/home/stephen/Desktop/Ant_videos/3.21.12_B144_0.1M/clip6_skip0_30spots/polygons.txt");
    if ( file.open(QIODevice::ReadOnly|QIODevice::Text))
    {
        qDebug()<<"opened polgons";
       QTextStream polygonFile( &file );
       QRegExp rx("\\((.*?)\\)");

       QString temp;
          do
          {
              tempV.clear();
              polyText = polygonFile.readLine();
              qDebug()<<"Full text"<<polyText<<"\n";
              parsedPolygons = polyText.split(",");

              temp = parsedPolygons.at(0);
              qDebug()<<"Constructed "<<temp;
              paintCanvas->polyNames.push_back(temp);
              parsedPolygons = polyText.split("),(");

              //qDebug()<<"removed "<<temp<<"\n";
              //construct vector<QPoint> polygon to add to  vector<vector<QPoint> > polygons;

              for ( QStringList::Iterator it = parsedPolygons.begin(); it != parsedPolygons.end(); ++it )
              {
                  temp = *it;

                  polyPart= temp.split(",") ;
                  if(*it==parsedPolygons.at(0)){
                      qDebug()<<polyPart.at(1).mid(1)<<""<<polyPart.at(2);
                      tempPoint.setX(polyPart.at(1).mid(1).toInt());
                      tempPoint.setY(polyPart.at(2).toInt());
                      tempV.push_back(tempPoint);
                  }else if(*it==parsedPolygons.back()){
                      temp = polyPart.at(1);
                      temp.remove(3,1);
                      qDebug()<<polyPart.at(0)<<""<<temp;
                      tempPoint.setX(polyPart.at(0).toInt());
                      tempPoint.setY(temp.toInt());
                      tempV.push_back(tempPoint);
                  }else{
                      qDebug()<<polyPart.at(0)<<""<<polyPart.at(1);
                      tempPoint.setX(polyPart.at(0).toInt());
                      tempPoint.setY(polyPart.at(1).toInt());
                      tempV.push_back(tempPoint);
                  }
              }

              paintCanvas->polygons.push_back(tempV);
              // call proocecss polygons
          }while(!polygonFile.atEnd());

    }else{
        qDebug()<<"Didn't open file";
    }
    file.close();
}

void MainWindow::on_skipSpinBox_valueChanged(int arg1)
{
    if(mugShotter!=NULL){
    mugShotter->skip = arg1;
    }
}

void MainWindow::on_polygonButton_clicked()
{
    paintCanvas->drawingPolygon = true;
    paintCanvas->drawingCirRects = false;
}

void MainWindow::on_cirRectButton_clicked()
{
    paintCanvas->drawingPolygon = false;
    paintCanvas->drawingCirRects = true;
}

void MainWindow::createCirParams(int x, int y)
{
    /*Need to implement keeping track and destroying CirParams objects    */

    paintCanvas->settingupCir = false;
    QMdiArea *cirParams =new QMdiArea;
    cirParams->setParent(paintCanvas);
  //  cirParams->setGeometry(x-50,y-40,100,90);
    cirParams->setGeometry(x,y,100,90);
    cirParams->setBackground(QBrush(QColor(255,255,255,100)));
    cirParams->setVisible(true);

    QSpinBox *integerSpinBox = new QSpinBox;
    integerSpinBox->setSingleStep(2);
    integerSpinBox->setValue(0);
    paintCanvas->numSpaces =0;
    integerSpinBox->setParent(cirParams);
    integerSpinBox->setGeometry(50,0,45,21);
    integerSpinBox->setRange(0,40);
    integerSpinBox->setVisible(true);
    integerSpinBox->setToolTip("Number of parking spaces");
    connect(integerSpinBox,SIGNAL(valueChanged(int)),this,SLOT(on_cirSpinBox_valueChanged(int)));

    QLabel *boxes = new QLabel;
    boxes->setParent(cirParams);
    boxes->setGeometry(5,0,61,21);
    boxes->setText("Boxes");
    boxes->setVisible(true);

    QLabel *width = new QLabel;
    width->setParent(cirParams);
    width->setGeometry(5,22,21,17);
    width->setText("W");
    width->setVisible(true);

    QSlider *wSlider = new QSlider;
    wSlider->setOrientation(Qt::Horizontal);
    wSlider->setGeometry(26,22,71,21);
    wSlider->setParent(cirParams);
    wSlider->setRange(10,150);
    wSlider->setValue(paintCanvas->varRectx);
    wSlider->setVisible(true);
    connect(wSlider,SIGNAL(valueChanged(int)),this,SLOT(on_wSlider_valueChanged(int)));
    on_wSlider_valueChanged(paintCanvas->varRectx);
    wSlider->setToolTip(QString::number(3*paintCanvas->varRectx));

    QLabel *height = new QLabel;
    height->setParent(cirParams);
    height->setGeometry(5,42,21,17);
    height->setText("H");
    height->setVisible(true);

    QSlider *hSlider = new QSlider;
    hSlider->setOrientation(Qt::Horizontal);
    hSlider->setGeometry(26,42,71,21);
    hSlider->setParent(cirParams);
    hSlider->setVisible(true);
    hSlider->setRange(10,150);
    hSlider->setValue(paintCanvas->varRecty);
    connect(hSlider,SIGNAL(valueChanged(int)),this,SLOT(on_hSlider_valueChanged(int)));
    on_hSlider_valueChanged(paintCanvas->varRecty);
    hSlider->setToolTip(QString::number(3*paintCanvas->varRecty));


    QPushButton *cancelButton =new QPushButton;
    cancelButton->setGeometry(5,65,55,21);
    cancelButton->setParent(cirParams);
    cancelButton->setText("Cancel");
    cancelButton->setEnabled(true);
    cancelButton->setVisible(true);
    connect(cancelButton,SIGNAL(clicked()),this,SLOT(on_cirCancelButton_clicked()));


    QPushButton *commitButton =new QPushButton;
    commitButton->setGeometry(62,65,35,21);
    commitButton->setParent(cirParams);
    commitButton->setText("Ok");
    commitButton->setEnabled(true);
    commitButton->setVisible(true);
    connect(commitButton,SIGNAL(clicked()),this,SLOT(on_cirOkButton_clicked()));

}

void MainWindow::on_cirSpinBox_valueChanged(int arg1)
{
    if(arg1==0){
        arg1=1;
    }
    paintCanvas->numSpaces=arg1;
    double theta = 360/arg1;
    theta = theta*M_PI/180;
    double angle=0;
//    paintCanvas->maskpath.addPath(paintCanvas->radSQ.at(0));

    paintCanvas->radSQ.clear();
    paintCanvas->temp.clear();
    paintCanvas->sqPolygons.clear();
    paintCanvas->tempSQ.clear();


    for(int i=0; i<arg1; i++){
        tempPoint = rot2d(QPoint(paintCanvas->rad,-paintCanvas->varRecty),angle);
        paintCanvas->tempSQ.push_back(tempPoint);
        paintCanvas->temp.push_back(tempPoint);

        tempPoint =rot2d(QPoint(paintCanvas->rad+paintCanvas->varRectx,-paintCanvas->varRecty),angle);
        paintCanvas->tempSQ.push_back(tempPoint);
        paintCanvas->temp.push_back(tempPoint);

        tempPoint =rot2d(QPoint(paintCanvas->rad+paintCanvas->varRectx,paintCanvas->varRecty),angle);
        paintCanvas->tempSQ.push_back(tempPoint);
        paintCanvas->temp.push_back(tempPoint);

        tempPoint =rot2d(QPoint(paintCanvas->rad,+paintCanvas->varRecty),angle);
        paintCanvas->tempSQ.push_back(tempPoint);
        paintCanvas->temp.push_back(tempPoint);

        paintCanvas->sqPolygons.push_back(paintCanvas->temp);

        paintCanvas->maskpath.addPolygon(QPolygon(paintCanvas->tempSQ));
        paintCanvas->closepath();
        angle = angle+theta;
       // qDebug()<<angle;
        paintCanvas->tempSQ.clear();
        paintCanvas->temp.clear();
      //  paintCanvas->radSQ.clear();


    }
    //paintCanvas->maskpath.addPolygon((int)(paintCanvas->centerCir.x()+paintCanvas->rad),(int)(paintCanvas->centerCir.y()-30),50,60);

}

QPoint MainWindow::rot2d(QPoint p,double t){
    QPoint ret;
    ret.setX((p.x()*cos(t)-p.y()*sin(t))+paintCanvas->centerCir.x());
    ret.setY((p.x()*sin(t)+p.y()*cos(t))+paintCanvas->centerCir.y());

    return ret;
}
void MainWindow::on_wSlider_valueChanged(int argw){
    paintCanvas->varRectx = argw;
    on_cirSpinBox_valueChanged(paintCanvas->numSpaces);
}
void MainWindow::on_hSlider_valueChanged(int argh){
    paintCanvas->varRecty = argh;
    on_cirSpinBox_valueChanged(paintCanvas->numSpaces);
}
void MainWindow::on_cirCancelButton_clicked(){
    qDebug()<<"cancel";
    paintCanvas->radSQ.clear();
    paintCanvas->temp.clear();
    paintCanvas->sqPolygons.clear();
    paintCanvas->tempSQ.clear();
    paintCanvas->closepath();

}

void MainWindow::on_cirOkButton_clicked(){
    qDebug()<<"Ok";
    int count=0;
//    vector<QVector<QPoint> >::iterator row;

    for (row = paintCanvas->sqPolygons.begin(); row != paintCanvas->sqPolygons.end(); ++row){
        tempV =*row;
        paintCanvas->polygons.push_back(*row);
        paintCanvas->polyNames.push_back("pSpot"+QString::number(count));
        count++;
    }
}
