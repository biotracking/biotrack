#include "genvid.h"
#include "ui_genvid.h"

GenVid::GenVid(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GenVid)
{
    ui->setupUi(this);

    nopath="(none selected)";
    btfpath=nopath;
    videopath=nopath;
    lastpath="";

    rparam.x=20;
    rparam.y=40;
    rad=40;
    currentFrame=0;
    currentExport=0;
    fontSize = 1;
    trailSize =100;
    boxStroke = 2;
    cirStroke =1;
    trailStroke=2;
    lineType = 8;
    arrowSize = 1;
    arrowStroke=2;
    connect(this,SIGNAL(allDone(int)),this->parent(),SLOT(quit()));

    cmd =false;
    isPlaying = false;
    isExporting =false;
    everythingok=false;
    ok=true;
    boxOn =false;
    arrowOn=true;
    idOn =true;
    trailOn =true;
    cirOn = false;
    cirU =false;
    trailU =false;
    arrU =false;
    xyOn =false;
    angleOn =false;


    ui->checkArrow->setChecked(true);
    arrowOn=ui->checkArrow->isChecked();

    ui->checkID->setChecked(true);
    ui->checkTrail->setChecked(true);

    ui->exportButton->setEnabled(false);
    ui->playButton->setEnabled(false);
    ui->progressBar->setVisible(false);
    ui->exportinglabel->setVisible(false);


    ui->spinBox_5->setSingleStep(10);

    ui->spinBox_5->setMaximum(15000);
    ui->spinBox_6->setSingleStep(1);
    ui->spinBox_6->setRange(1,10);
    ui->spinBox_6->setValue(1);
    ui->spinBox_7->setSingleStep(5);
    ui->spinBox_7->setRange(20,200);
   // ui->spinBox_7->setValue(45);
    ui->spinBox_8->setSingleStep(1);
    ui->spinBox_8->setRange(1,10);
    ui->spinBox_8->setValue(2);
   // ui->spinBox_9->setValue(3);


    ui->spinBox_5->setValue(trailSize);


    checkReady();
    startTimer(0);
}

GenVid::~GenVid()
{
    delete ui;
}

void GenVid::on_actionLoad_Video_triggered()
{
    /* select a directory using file dialog */
    videopath = QFileDialog::getOpenFileName (this, tr("Open Video File"),lastpath, tr("Tracked Video (*.avi *.mov *.mpg *.mpeg *.mp4)"));

    if ( videopath.isNull() == false )
    {
        lastpath=videopath;

        ui->pathlabel->setText("..."+videopath.right(50));
        qDebug()<<videopath;

    }else{
        videopath=nopath;
        ui->pathlabel->setText(videopath.right(50));
    }

    checkReady();
}



void GenVid::on_actionLoad_BTF_triggered()
{
    if(!cmd){
        btfpath = QFileDialog::getExistingDirectory(this, tr("Select BTF sources"),lastpath);
    }
    QDir myDir(btfpath);
    QStringList filters;
    filters<<"*.btf";
    myDir.setNameFilters(filters);

    int index=0;
    QRegExp rx("_.*.btf");

    QStringList list = myDir.entryList(filters);
    for (QStringList::iterator it = list.begin(); it != list.end(); ++it) {
        QString current = *it;
        index = current.indexOf(rx,0);
        QString type =  current.mid(index);
        qDebug()<<current<<" current index   "<<current.mid(0,index);
        if(type.contains("id")){

            idfilepath = btfpath+"/"+current;
            qDebug()<<"loaded "<<idfilepath;
        }else
            if(type.contains("frames")){ //Cannot handle an underscore (I used to search for timestamp_frames
                timestampfilepath = btfpath+"/"+current;
                qDebug()<<"loaded TIMESTAMP_FRAMES "<<timestampfilepath;


            }else
                if(type.contains("timestamp.btf")){
                    timefilepath = btfpath+"/"+current;
                    qDebug()<<"loaded "<<timefilepath;

                }else
                    if(type.contains("timage")){

                        anglefilepath = btfpath+"/"+current;
                        qDebug()<<"loaded "<<anglefilepath;
                    }else
                        if(type.contains("ximage")){

                            xfilepath = btfpath+"/"+current;
                            qDebug()<<"loaded "<<xfilepath;
                        }else
                            if(type.contains("yimage")){

                                yfilepath = btfpath+"/"+current;
                                qDebug()<<"loaded "<<yfilepath;
                            }
    }


    if ( btfpath.isNull() == false)
    {
        lastpath=btfpath;
        ui->btflabel->setText("..."+btfpath.right(50));
    }else{
        btfpath=nopath;
        ui->btflabel->setText(btfpath.right(50)+" File is Null");
    }
    checkReady();
}


void GenVid::checkReady()
{

    QString error="ERROR:  ";
    QString colour="red"; // you can use also QColor
    QString fonttemplate = "<font color='%1'>%2</font>";
    QString notprepared="<b>Not Ready!</b>  <i>Please choose a...</i> <br>";

    if (videopath==nopath)
        notprepared= notprepared+"   ||    Video Source   ||   ";
    if (btfpath==nopath)
        notprepared= notprepared+"   ||   BTF Source     ||";
    if (btfpath!=nopath && videopath!=nopath){
        colour="green"; // you can use also QColor
        QString text="<b>Ready to Generate Video</b> <br> <br> Press Export to Begin";
        fonttemplate = tr("<font color='%1'>%2</font>");
        ui->messagelabel->setText(fonttemplate.arg( colour, text ));
        ui->exportButton->setEnabled(true);
        ui->playButton->setEnabled(true);
        ui->playButton->setText("Pause");
        capture.open(videopath.toStdString());


        processFiles();
        everythingok=true;

        isPlaying=!isPlaying;

        minlength = std::min(btfLength,(float)capture.get(CV_CAP_PROP_FRAME_COUNT)-1 );
        qDebug()<<"minlength  "<<minlength;
        ui->horizontalSlider->setRange(0,minlength);


        ui->horizontalSlider->setSingleStep(1);


    }else{
        colour="red"; // you can use also QColor
        QString text=notprepared;
        fonttemplate = tr("<font color='%1'>%2</font>");
        ui->messagelabel->setText(fonttemplate.arg( colour, text ));
        everythingok=false;
    }
}


Mat GenVid::updateFrame()
{

    Mat img;
    if (   minlength> capture.get(CV_CAP_PROP_POS_FRAMES))
    {
        capture.read(img);
    }
    else
    {
        img=currentimg.clone();
        //capture.set(CV_CAP_PROP_POS_FRAMES,capture.get(CV_CAP_PROP_FRAME_COUNT)-1);
    }

    return img;
}



void GenVid::updateImage(Mat cvimage)
{
    qimage = QImage((const uchar*)cvimage.data, cvimage.cols, cvimage.rows, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();
    ui->imagelabel->setPixmap(QPixmap::fromImage(qimage));
    return;
}


void GenVid::timerEvent(QTimerEvent*) {
    updater();
}

void GenVid::updater(){
    if( currentFrame>=minlength){
        isPlaying=false;
        ui->playButton->setText("Play");
    }


    if (isPlaying){
        if(currentFrame<minlength){
            Mat img = updateFrame();
            currentimg=img.clone();
            img = checkFile(img);
            updateImage(img);

            current = QString::number(capture.get(CV_CAP_PROP_POS_FRAMES));
            ui->statusBar->showMessage(QString::number((int)(100*capture.get(CV_CAP_PROP_POS_FRAMES)/(minlength)))+"%  "+
                                       current+"/"+(QString::number(minlength)));

            ui->horizontalSlider->setValue(capture.get(CV_CAP_PROP_POS_FRAMES));
            currentFrame++;
        }
    } else if (isExporting){
        qDebug()<<"Timelist back"<<timeList.back().toInt();

        if( currentFrame<minlength){
            ui->horizontalSlider->setEnabled(false);
            ui->playButton->setText("Stop");
            Mat img = updateFrame();
            currentimg=img.clone();

            img = checkFile(img);
            updateImage(img);
            runExporter(img);

            current = QString::number(capture.get(CV_CAP_PROP_POS_FRAMES));
            ui->statusBar->showMessage(QString::number((int)(100*capture.get(CV_CAP_PROP_POS_FRAMES)/(minlength)))+"%  "+
                                       current+"/"+QString::number((minlength)));
            ui->progressBar->setValue((int)(100*capture.get(CV_CAP_PROP_POS_FRAMES)/(minlength)));
            if(cmd){
                qDebug()<<capture.get(CV_CAP_PROP_POS_FRAMES)/(minlength)<<"% complete:"<<capture.get(CV_CAP_PROP_POS_FRAMES)<<"/"<<minlength;
            }
            qDebug()<<"Still exporting "<<currentFrame;
            currentFrame++;
        } else {
            qDebug()<<"stop exporting! "<<currentFrame;

            ui->progressBar->setValue(0);
            ui->horizontalSlider->setEnabled(true);

            isExporting=!isExporting;
            isPlaying=false;
            vOut.release();
            vOut.~VideoWriter();
            if(cmd){
                this->~GenVid();
            }
        }
    }




}

void GenVid::processFiles()
{

    qDebug()<<"processfiles  ";
    QFile idfile(idfilepath);
    QFile timefile(timefilepath);
    QFile timestampfile(timestampfilepath);

    QFile anglefile(anglefilepath);
    QFile xfile(xfilepath);
    QFile yfile(yfilepath);

    qDebug()<<"Timestampfile paths  "<<timestampfilepath <<"  time "<<timefilepath;
    //reset any values loaded in

    antsAtMsec.clear();
    Msec.clear();
    timeList.clear();
    timestamp.clear();
    time.clear();
    xs.clear();
    ys.clear();

            angle.clear();
    ant_id.clear();
    timeList.clear();
    xsList.clear();

    ysList.clear();

    angleList.clear();
    ant_List.clear();


    if ( idfile.open(QIODevice::ReadOnly|QIODevice::Text) |
         xfile.open(QIODevice::ReadOnly|QIODevice::Text) |
         yfile.open(QIODevice::ReadOnly|QIODevice::Text) |
         anglefile.open(QIODevice::ReadOnly|QIODevice::Text) |
         timefile.open(QIODevice::ReadOnly|QIODevice::Text) |
         timestampfile.open(QIODevice::ReadOnly|QIODevice::Text)

         )
    {
        QTextStream idStamp( &idfile );
        QTextStream xStamp( &xfile );
        QTextStream yStamp( &yfile );
        QTextStream angleStamp( &anglefile );
        QTextStream timeMillisstream( &timefile );
        QTextStream timeStampstream( &timestampfile );


        QString delimiterPattern("\n");

        time = timeMillisstream.readAll();
        timestamp = timeStampstream.readAll();
        xs = xStamp.readAll();
        ys = yStamp.readAll();
        angle = angleStamp.readAll();
        ant_id = idStamp.readAll();

        timeList = time.split(delimiterPattern);
        timeStampList = timestamp.split(delimiterPattern);

        xsList = xs.split(delimiterPattern);
        ysList = ys.split(delimiterPattern);
        angleList = angle.split(delimiterPattern);
        ant_List = ant_id.split(delimiterPattern);

        qDebug()<<"Number of BTF Lines for each file (should be consistent)";
        qDebug()<<timeList.size()<<" "<<timeStampList.size()<<" "<<xsList.size()<<" "<<ysList.size()<<" "<<angleList.size()<<" "<<ant_List.size();

          qDebug()<<"Final Frame  "<<timeStampList.at(timeStampList.size()-2).toInt(); // The very last value of all these is a "" or 0 in Int for some reason
btfLength = timeStampList.at(timeStampList.size()-2).toInt();
        // MSec data structure: A vector of every MSec, for a particular MSec a vector of ants, each ant is a qstringlist of parameters.

        int currentMSec =-33;
        for (std::vector<int>::size_type i = 0; i<timeList.size();i++){
            if(currentMSec == timeList.at(i).toInt()){
                tempant << timeList.at(i)<<xsList.at(i)<<ysList.at(i)<<angleList.at(i)<<ant_List.at(i);
                antsAtMsec.push_back(tempant);
                tempant.clear();
               // currentMSec = timeList.at(i).toInt();
            }else{
                Msec.push_back(antsAtMsec);
                antsAtMsec.clear();
                tempant << timeList.at(i)<<xsList.at(i)<<ysList.at(i)<<angleList.at(i)<<ant_List.at(i);
                antsAtMsec.push_back(tempant);
                tempant.clear();
                currentMSec = timeList.at(i).toInt();
                qDebug()<<"Length of BTF data   currentMSec   "<<currentMSec;

            }
        }
//        Msec.back();
        qDebug()<<"Length of BTF data   currentMSec   "<<currentMSec<<" Msec bcak ";
//        btfLength=currentMSec;
    }


}

Mat GenVid::checkFile(Mat img){

    antsAtMsec = Msec.at(currentFrame);
    for(int i = 0; i<antsAtMsec.size();i++){
        antTrails.push_back(tempTrail);
    }

    for (std::vector<int>::size_type i = 0; i<antsAtMsec.size();i++){
        tempant = antsAtMsec.at(i);
        if(!map.contains(antsAtMsec.at(i).at(4))){
            map[antsAtMsec.at(i).at(4)] = QColor(randInt(0,255),randInt(0,255),randInt(0,255));
        }

        QMapIterator<QString, QColor> im(map);
        while (im.hasNext()) {
            im.next();
            // qDebug() << im.key() << ": " << im.value();
        }

        tempcolor = map[antsAtMsec.at(i).at(4)];
        r = tempcolor.red();
        g = tempcolor.green();
        b = tempcolor.blue();


        //Draw layers on img
        //Set text offset from centroid of ant
        org.x = tempant.at(1).toInt()+30;
        org.y = tempant.at(2).toInt()-30;

        //Parameterize width & hieght of boxes, rotate, and draw boxes.
        r1 =Point(-rparam.x,-rparam.y);
        r2 =Point(-rparam.x,rparam.y);
        r3 =Point(rparam.x,rparam.y);
        r4 =Point(rparam.x,-rparam.y);

        //Give stuff decent names for sanity's sake
        double tempantRadians= tempant.at(3).toDouble();



        if(trailOn){
            //if 100 frames ago existed
            if (currentFrame-trailSize>=0){
                //for every frame from then until now
                for(int j=currentFrame-trailSize;j<currentFrame-1;j++){
                    //at a particular time in the past look across all the ants
                    for (vector<QStringList>::iterator it = Msec.at(j).begin(); it!=Msec.at(j).end(); ++it) {
                        currenttrack=*it;
                        // if that ant is the same as the currrent ant
                        if(currenttrack.at(4)==antsAtMsec.at(i).at(4)){
                            //qDebug()<<j<<"frames ago"<<currenttrack;
                            //  qDebug()<<"Current"<<currenttrack;
                            if(previoustrack.size()!=0){
                                // qDebug()<<"Previous"<<previoustrack;

                                tempcolor = map[currenttrack.at(4)];
                                r = tempcolor.red();
                                g = tempcolor.green();
                                b = tempcolor.blue();
                                //qDebug()<<currenttrack.at(4)<<tempc<<"RGB :"<<r<<g<<b;
                                if(trailU){
                                    line(img,Point(previoustrack.at(1).toInt(),previoustrack.at(2).toInt()),
                                         Point(currenttrack.at(1).toInt(),currenttrack.at(2).toInt()),
                                         Scalar(b,g,r,100),
                                         trailStroke,CV_AA);
                                }else{
                                    line(img,Point(previoustrack.at(1).toInt(),previoustrack.at(2).toInt()),
                                         Point(currenttrack.at(1).toInt(),currenttrack.at(2).toInt()),
                                         Scalar(155,155,155,100),
                                         trailStroke,CV_AA);
                                }
                            }
                            previoustrack.clear();
                            previoustrack<<currenttrack.at(0)<<currenttrack.at(1)<<currenttrack.at(2)<<currenttrack.at(3)<<currenttrack.at(4);
                        }
                    }
                }
                previoustrack.clear();

            } else {
                for(int j=0;j<currentFrame-1;j++){

                    //at a particular time in the past look across all the ants
                    for (vector<QStringList>::iterator it = Msec.at(j).begin(); it!=Msec.at(j).end(); ++it) {
                        currenttrack=*it;
                        // if that ant is the same as the currrent ant
                        if(currenttrack.at(4)==antsAtMsec.at(i).at(4)){
                            //qDebug()<<j<<"frames ago"<<currenttrack;
                            //  qDebug()<<"Current"<<currenttrack;

                            if(previoustrack.size()!=0){
                                //    qDebug()<<"Previous"<<previoustrack;
                                tempcolor = map[currenttrack.at(4)];
                                r = tempcolor.red();
                                g = tempcolor.green();
                                b = tempcolor.blue();
                                //    qDebug()<<tempc<<"RGB :"<<r<<g<<b;
                                if(trailU){
                                    line(img,Point(previoustrack.at(1).toInt(),previoustrack.at(2).toInt()),
                                         Point(currenttrack.at(1).toInt(),currenttrack.at(2).toInt()),
                                         Scalar(b,g,r,100),
                                         trailStroke,CV_AA);
                                }else{
                                    line(img,Point(previoustrack.at(1).toInt(),previoustrack.at(2).toInt()),
                                         Point(currenttrack.at(1).toInt(),currenttrack.at(2).toInt()),
                                         Scalar(155,155,155,100),
                                         trailStroke,CV_AA);
                                }
                            }
                            previoustrack.clear();
                            previoustrack<<currenttrack.at(0)<<currenttrack.at(1)<<currenttrack.at(2)<<currenttrack.at(3)<<currenttrack.at(4);
                        }
                    }
                }
                previoustrack.clear();

            }

        }



        if(cirOn){
            if(cirU){
                circle(img,Point(tempant.at(1).toInt(),tempant.at(2).toInt()),rad,Scalar(b, g, r),cirStroke,CV_AA);
            }else{
                circle(img,Point(tempant.at(1).toInt(),tempant.at(2).toInt()),rad,Scalar(155, 155, 155),cirStroke,CV_AA);
            }
        }

        //attach ant name

        if(idOn){
            if(fontSize==1){
                putText( img,tempant.at(4).toStdString(), Point(org.x,org.y), fontSize, 1.5, Scalar( 255, 255, 255 ),2, CV_AA);
            } else {
                putText( img,tempant.at(4).toStdString(), Point(org.x,org.y-15), fontSize, 1.5, Scalar( 255, 255, 255 ),2, CV_AA);
            }
        }
        if(xyOn){
            if(fontSize==1){
                putText( img,"("+tempant.at(1).toStdString()+","+tempant.at(2).toStdString()+")", Point(org.x,org.y+30), fontSize, 1.5, Scalar( 255, 255, 255 ),2, CV_AA);
            } else {
                putText( img,"("+tempant.at(1).toStdString()+","+tempant.at(2).toStdString()+")", Point(org.x,org.y+60), fontSize, 1.5, Scalar( 255, 255, 255 ),2, CV_AA);
            }
        }
        if(angleOn){

            if(fontSize==1){
                putText( img,QString::number(((tempantRadians*180)/M_PI)).toStdString()+"deg", Point(org.x,org.y+60), fontSize, 1.5, Scalar( 255, 255, 255 ),2, CV_AA);
            } else {
                putText( img,QString::number(((tempantRadians*180)/M_PI)).toStdString()+"deg", Point(org.x,org.y+120), fontSize, 1.5, Scalar( 255, 255, 255 ),2, CV_AA);
            }
        }


        //               //New bounding box
        //               //Works correctly
        RotatedRect boundRect( RotatedRect(
                                   Point2f(tempant.at(1).toInt(),tempant.at(2).toInt()), //arrow center
                                   Size(ui->spinBox_4->value()*3,ui->spinBox_4->value()), //Scale the box
                                   tempantRadians * 180 / 3.1415926    //Rotate the box
                                   )

                               );
        //Points encoded as follows: sides 01 23  front 12 back 30

        Point2f rect_points[4];
        boundRect.points( rect_points );



        Point arrowTip, arrowCenter, arrowLeft, arrowRight;
        arrowCenter = Point(tempant.at(1).toInt(),tempant.at(2).toInt());

        //Arrow tip is in between two corners of bounding box
        arrowTip = Point( (rect_points[2].x + rect_points[3].x)/2 ,
                          (rect_points[2].y + rect_points[3].y)/2);
        //  arrowTip = Point(10,20);
        arrowLeft = Point( ( (rect_points[2].x + rect_points[1].x)/2 + rect_points[2].x)/2 ,
                           ((rect_points[2].y + rect_points[1].y)/2 +rect_points[2].y)/2);

        arrowRight = Point( ( (rect_points[0].x + rect_points[3].x)/2 + rect_points[3].x)/2 ,
                            ((rect_points[0].y + rect_points[3].y)/2 +rect_points[3].y)/2);


        if(arrowOn){
            if(arrU){
                //Central Line
                line(img,arrowCenter,arrowTip,Scalar(b,g,r,100),arrowStroke,CV_AA);
                //ArrowTip left
                line(img,arrowTip,arrowLeft,Scalar(b,g,r,100),arrowStroke,CV_AA);
                //ArrowTip Right
                line(img,arrowTip,arrowRight,Scalar(b,g,r,100),arrowStroke,CV_AA);

            }else{
                line(img,arrowCenter,arrowTip,Scalar(255,0,0,100),arrowStroke,CV_AA);
                line(img,arrowTip,arrowLeft,Scalar(255,0,0,100),arrowStroke,CV_AA);
                line(img,arrowTip,arrowRight,Scalar(255,0,0,100),arrowStroke,CV_AA);

            }
        }

    }

    return img;
}

void GenVid::runExporter(Mat img)
{
    vOut<<img;
    //vOut.write(img);

}

/*
  P is point to be rotated about the origin Org
  org is origin
  t is angle in radians
  */
Point GenVid::rot2d(Point p,Point org,double t){
    Point ret;
    ret.x = p.x - org.x;
    ret.y = p.y - org.y;

    ret.x = ((ret.x*cos(t)-ret.y*sin(t)));
    ret.y = ((ret.x*sin(t)+ret.y*cos(t)));
    ret.x = ret.x+org.x;
    ret.y = ret.y+org.y;

    return ret;
}

void GenVid::on_exportButton_clicked()
{

    isPlaying = false;
    int index = index;
    QRegExp path_rx("/(?!.*/)");
    index = videopath.indexOf(path_rx,0);
    videoDir =  videopath.mid(0,index+1);

    if(!cmd){
        savepath = QFileDialog::getSaveFileName(this,
                                                tr("Save File Name"), videoDir+"out.mov",
                                                tr("Tracked Video (*.mov)"));
    }


    ui->progressBar->setVisible(true);
    ui->exportinglabel->setVisible(true);
    ui->messagelabel->setVisible(false);
    if(isExporting){
        currentExport++;
    }

    isExporting=!isExporting;
    currentFrame =0;
    capture.set(CV_CAP_PROP_POS_FRAMES,0);


    qDebug()<<"Video Path is: "<<videoDir<<"Parse code"<<index;
    vOut.open((savepath).toStdString(),CV_FOURCC('D', 'I', 'V', 'X'),capture.get(CV_CAP_PROP_FPS),Size((int)(capture.get(CV_CAP_PROP_FRAME_WIDTH)),(int)(capture.get(CV_CAP_PROP_FRAME_HEIGHT))),true);

}

void GenVid::on_playButton_clicked()
{
    if(!isExporting){
        isPlaying=!isPlaying;
        if(isPlaying){
            ui->playButton->setText("Pause");
        }else if(!isPlaying){
            ui->playButton->setText("Play");
        }
    }
    else{
        ui->progressBar->setValue(0);
        isExporting=false;
        vOut.~VideoWriter();
        vOut.release();

    }

}

void GenVid::on_checkBox_clicked()
{
    boxOn=!boxOn;
    refresh();

}

void GenVid::on_checkID_clicked()
{
    idOn=!idOn;
    refresh();

}

void GenVid::on_checkArrow_clicked()
{
    arrowOn=ui->checkArrow->isChecked();
  //  arrowOn =!arrowOn;
    refresh();

}

void GenVid::on_checkTrail_clicked()
{
    trailOn =!trailOn;
    refresh();

}

void GenVid::on_spinBox_valueChanged(int arg1)
{
    rparam.x = arg1;
    refresh();

}

void GenVid::on_spinBox_2_valueChanged(int arg1)
{
    rparam.y = arg1;
    refresh();

}

void GenVid::on_spinBox_3_valueChanged(int arg1)
{
    fontSize = arg1;
    refresh();

}

void GenVid::on_spinBox_5_valueChanged(int arg1)
{
    trailSize = arg1;
    refresh();

}

int GenVid::randInt(int low, int high)
{
    // Random number between low and high
    return qrand() % ((high + 1) - low) + low;
}

void GenVid::on_pushButton_pressed()
{
    fontSize =((int)fontSize+1) % 2;
    if(fontSize==1){
        ui->label_3->setText("Small");
    } else {
        ui->label_3->setText("Large");
    }
    refresh();

}

void GenVid::on_horizontalSlider_sliderMoved(int position)
{
    //scrubRatio = (double)position/100;
    //    currentFrame = (int)(scrubRatio*(capture.get(CV_CAP_PROP_FRAME_COUNT)));

    if( position<minlength){

        isPlaying=false;
        ui->playButton->setText("Play");
        currentFrame = position;
        capture.set(CV_CAP_PROP_POS_FRAMES,(double)position);
        qDebug()<<currentFrame;
        Mat img = updateFrame();
        currentimg=img.clone();

        img = checkFile(img);
        updateImage(img);
        current = QString::number(capture.get(CV_CAP_PROP_POS_FRAMES));
        ui->statusBar->showMessage(QString::number((int)(100*capture.get(CV_CAP_PROP_POS_FRAMES)/capture.get(CV_CAP_PROP_FRAME_COUNT)-1))+"%  "+
                                   current+"/"+(QString::number(capture.get(CV_CAP_PROP_FRAME_COUNT)-1)));

    }

}

void GenVid::on_horizontalSlider_valueChanged(int value)
{


}

void GenVid::on_spinBox_7_valueChanged(int arg1)
{
    rad = arg1;
    refresh();

}

void GenVid::on_spinBox_6_valueChanged(int arg1)
{
    //cir stroke
    cirStroke = arg1;
    refresh();

}



void GenVid::on_spinBox_8_valueChanged(int arg1)
{
    //trail thickness
    trailStroke = arg1;
    refresh();
}

void GenVid::on_spinBox_4_valueChanged(int arg1)
{
    //arrow size
    arrowSize =arg1;
    refresh();

}

void GenVid::on_checkBox_2_clicked()
{
    // circle on
    cirOn =!cirOn;
    refresh();

}

void GenVid::on_radioButton_clicked()
{
    //cir unique
    if(cirU){
        ui->radioButton->setText("Gray");
    } else {
        ui->radioButton->setText("Color");
    }
    cirU=!cirU;
    refresh();


}

void GenVid::on_radioButton_2_clicked()
{
    //trail unique
    if(trailU){
        ui->radioButton_2->setText("Gray");
    } else {
        ui->radioButton_2->setText("Color");
    }
    trailU=!trailU;
    refresh();

}

void GenVid::on_radioButton_3_clicked()
{
    //arrow unique
    if(arrU){
        ui->radioButton_3->setText("Blue");
    } else {
        ui->radioButton_3->setText("Color");
    }
    arrU =!arrU;

    refresh();


}

void GenVid::on_checkID_2_clicked()
{
    //xy
    xyOn=!xyOn;
    refresh();

}

void GenVid::on_checkID_3_clicked()
{
    //angle
    angleOn=!angleOn;
    refresh();

}

void GenVid::on_spinBox_9_valueChanged(int arg1)
{
    // arrow stroke
    arrowStroke = arg1;
    refresh();

}



void GenVid::on_videopathpushButton_clicked()
{
    on_actionLoad_Video_triggered();
}

void GenVid::on_videopathpushButton_2_clicked()
{
    on_actionLoad_BTF_triggered();
}

void GenVid::refresh(){
    //Refresh the view
    if(everythingok){
        if(currentFrame<minlength){

            Mat img;
            img= currentimg.clone();
            img = checkFile(img);
            updateImage(img);
        }
    }
}
