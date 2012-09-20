#include "multitrack.h"


Multitrack::Multitrack(QWidget *parent, Qt::WFlags flags)
    : QMainWindow(parent, flags)
{


    trackerChecked=false;
    nopath="(none selected)";
    bgpath=nopath;
    modelfolder=nopath;
    maskpath=nopath;
    videopath=nopath;
    projectSaveDirectory="";
    lastpath="";

    ui.setupUi(this);

    setWindowIcon(QIcon("Multitrack.png"));


    imageData = NULL;
    icpTracker= NULL;
    imageLabel = ui.imageLabel;


    isPlaying = false;
    isVideoShowing=true;
    completedTracking=false;


    /*
    * Connect UI buttons to slots
    */
    ui.stopButton->setEnabled(false);
    ui.saveBTFButton->setEnabled(false);
    //ui.blobsButton->setEnabled(false);
    //connect(ui.blobsButton, SIGNAL(clicked()), this, SLOT(toggleBlobsView()));
    ui.subtractioncheckBox->setChecked(false);
    connect(ui.subtractioncheckBox, SIGNAL(clicked()), this, SLOT(toggleBlobsView()));

    ui.contourTrackingcheckBox->setChecked(false);
    connect(ui.contourTrackingcheckBox, SIGNAL(clicked()), this, SLOT(toggleContourTracking()));


    ui.resetButton->setEnabled(false);
    //ui.resolutionSpinBox->setValue(resFractionMultiplier);
    connect(ui.stopButton, SIGNAL(clicked()), this, SLOT(toggleStopButton()));
    //connect(ui.saveHTMLButton, SIGNAL(clicked()), this, SLOT(saveHTML()));
    connect(ui.saveBTFButton, SIGNAL(clicked()), this, SLOT(saveBTF()));
    connect(ui.resetButton, SIGNAL(clicked()), this, SLOT(icpReset()));
    ui.backgroundButton->setEnabled(true);
    connect(ui.backgroundButton, SIGNAL(clicked()), this, SLOT(loadBackgroundFile()));
    ui.videoButton->setEnabled(true);
    connect(ui.videoButton, SIGNAL(clicked()), this, SLOT(loadVideoFile()));
    ui.modelButton->setEnabled(true);
    connect(ui.modelButton, SIGNAL(clicked()), this, SLOT(loadModelFile()));
    ui.maskButton->setEnabled(true);
    connect(ui.maskButton, SIGNAL(clicked()), this, SLOT(chooseMaskFile()));

    ui.projectDirectoryButton->setEnabled(true);
    connect(ui.projectDirectoryButton, SIGNAL(clicked()), this, SLOT(chooseProjectDirectory()));



    connect(ui.resetButton, SIGNAL(clicked()), this, SLOT(icpReset()));

    connect(ui.bgSubThresholdSpinBox, SIGNAL(valueChanged(int)), this, SLOT(bgThresholdSpinValueChanged(int)));
    //connect(ui.bgsubSlider, SIGNAL(sliderMoved(int)), ui.bgSubThresholdSpinBox, SIGNAL(bgThresholdSpinValueChanged(int)));

    connect(ui.blobBirthAreaThresholdSpinBox, SIGNAL(valueChanged(int)), this, SLOT(blobBirthAreaThresholdValueChanged()));
    connect(ui.resolutionSpinBox, SIGNAL(valueChanged(int)), this, SLOT(resolutionFractionValueChanged()));
    connect(ui.trackdistanceSpinBox, SIGNAL(valueChanged(int)), this, SLOT(trackdistanceValueChanged()));

    connect(ui.trackdeathSpinBox, SIGNAL(valueChanged(int)), this, SLOT(trackDeathValueChanged()));
    connect(ui.separationSpinBox, SIGNAL(valueChanged(int)), this, SLOT(separationValueChanged()));

    connect(ui.actionLoad_Settings, SIGNAL(triggered()), this, SLOT(loadSettings()));
    connect(ui.actionLoad_Defaults, SIGNAL(triggered()), this, SLOT(loadDefaults()));
    connect(ui.actionSave_All, SIGNAL(triggered()), this, SLOT(saveSettings()));

    ui.visualizationLabel->setAttribute(Qt::WA_TranslucentBackground);
    readSettings();

    loadNewTracker();

    on_displaycomboBox_currentIndexChanged(0);

    //Start the Timer Running (This controls the speed of QT's main loop)
    startTimer(0);  // High speed-second timer
    //startTimer(1000/300000000);  // example 0.1-second timer

}

Multitrack::~Multitrack()
{

    capture.release(); // Gives closing errors sometimes
    delete imageData;
    delete icpTracker;

}

void Multitrack::timerEvent(QTimerEvent*) {
    if (!isPlaying){ return;}
    else{
        Mat img = updateFrame();
        if(completedTracking==false){
            icpTracker->track(img, (int)capture.get(CV_CAP_PROP_POS_FRAMES));
            if(ui.display_pushButton->isChecked()){

            updateImage(img);

            Mat qImgARGB;
            qImgARGB = icpTracker->getTrackResultImage();
            QImage qimage;
            qimage = QImage((const uchar*)qImgARGB.data, qImgARGB.cols, qImgARGB.rows,qImgARGB.step, QImage::Format_ARGB32);
            qimage = qimage.rgbSwapped();
            qimage = qimage.scaled(displayWidth, displayHeight);
        //    qimage = qimage.scaled(displayWidth, displayHeight);

            ui.visualizationLabel->setPixmap(QPixmap::fromImage(qimage));



            }
        }
        updateStatusBar();
    }
}

Mat Multitrack::updateFrame()
{
    Mat img;
    // if the current frame is less than total frames minus 1 (because one starts at 0) , keep analysing
    if (capture.get(CV_CAP_PROP_POS_FRAMES)<capture.get(CV_CAP_PROP_FRAME_COUNT)-1 )
    {
        //Advances tracking by one frame
        capture.retrieve(img);
        capture.read(img);
        if( img.empty()){ // extra check if at end or weird file

            //!!  Stop Tracking  !!//

            toggleStopButton();
            ui.stopButton->setEnabled(false);
            ui.resetButton->setEnabled(true);

            Mat imgBlank;
            return imgBlank;
        }
        return img;
    }
    else //Stop Everything!
    {

        //!!  Stop Tracking  !!//
        completedTracking=true;
        toggleStopButton();
        ui.stopButton->setEnabled(false);
        ui.resetButton->setEnabled(true);


        return img;
    }

}

void Multitrack::updateStatusBar()
{
    int curframe = (int)capture.get(CV_CAP_PROP_POS_FRAMES);


    int totalframes= (int)capture.get(CV_CAP_PROP_FRAME_COUNT);

    float totaltime= myQTime.elapsed()+pausedMillis+0.000;

    float framerate= curframe/(float)(totaltime/1000.0);
    int   numtargets=icpTracker->activeTracks.size();

    float timeFormat = totaltime/1000.00;

    totalframes=totalframes-1;  //Fix maybe do this for safety?

    //    QString::sprintf(statusMessage,"frame: %d / %d    elapsed time: %4.3f   average frame rate (fps): %4.2f   Current number of targets: %d  ", curframe, totalframes,timeFormat, framerate,numtargets);
    statusMessage.sprintf("frame: %d / %d    elapsed time: %4.3f   average frame rate (fps): %4.2f   Current number of targets: %d  ", curframe, totalframes,timeFormat, framerate,numtargets);
    if(completedTracking){
        statusMessage.append(" ---  all frames analyzed");
    }
    statusBar()->showMessage(statusMessage);


}

void Multitrack::messageToStatusBar(QString message)
{
    statusBar()->showMessage(tr(message.toAscii()));
}

void Multitrack::nextFrame()
{
    Mat img = updateFrame();
    updateImage(img);
    updateStatusBar();
}


void Multitrack::saveHTMLinteractions()
{
    messageToStatusBar("saving interaction report...");
    icpTracker->outputInteractionsReport();
    messageToStatusBar("done saving interaction report");
}

void Multitrack::saveBTF()
{
    messageToStatusBar("saving BTF report...");
    icpTracker->outputBTF(projectSaveDirectory,ui.projectsavename->text());
    QString messageo= "done saving BTF data for project: " +ui.projectsavename->text();
    messageToStatusBar(messageo.toAscii());
}

void Multitrack::bgThresholdSpinValueChanged(int value)
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath){
        ui.bgSubThresholdSpinBox->setValue(value);
        icpTracker->setBgSubThreshold(ui.bgSubThresholdSpinBox->value());
    }
}


void Multitrack::blobBirthAreaThresholdValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        if(ui.blobBirthAreaThresholdSpinBox->value() > 0){
            icpTracker->setTrackBirthAreaThreshold(ui.blobBirthAreaThresholdSpinBox->value());
        }
}

void Multitrack::resolutionFractionValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setResolutionFraction(ui.resolutionSpinBox->value());
}
void Multitrack::trackdistanceValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setMatchDistanceThreshold(ui.trackdistanceSpinBox->value());
}
void Multitrack::trackDeathValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setTrackDeathThreshold(ui.trackdeathSpinBox->value());
}
void Multitrack::separationValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setSeparationThreshold(ui.separationSpinBox->value());
}

void Multitrack::toggleStopButton()
{
    if (isPlaying)
    {
        //ui.blobsButton->setEnabled(true);
        ui.stopButton->setText(tr("play"));
        //icpTracker->outputInteractionsReport();
        //ui.saveHTMLButton->setEnabled(true);
        ui.saveBTFButton->setEnabled(true);
        ui.resetButton->setEnabled(true);
        ui.backgroundButton->setEnabled(true);
        ui.videoButton->setEnabled(true);
        ui.maskButton->setEnabled(true);
        ui.modelButton->setEnabled(true);

        isPlaying = !isPlaying;
        //There is no Pause and Resume method for Qtime, Should probably use a QTimer
        pHour =myQTime.hour();
        pMin = myQTime.minute();
        pSec= myQTime.second();
        pMsec = myQTime.msec();
        pausedMillis=myQTime.elapsed()+pausedMillis;

    }
    else
    {
        //ui.blobsButton->setEnabled(true);
        ui.stopButton->setText(tr("||"));
        //ui.saveHTMLButton->setEnabled(false);
        ui.saveBTFButton->setEnabled(false);
        ui.resetButton->setEnabled(false);
        //turn off load files when playing
        ui.backgroundButton->setEnabled(false);
        ui.videoButton->setEnabled(false);
        ui.maskButton->setEnabled(false);
        ui.modelButton->setEnabled(false);

        isPlaying = !isPlaying;

        myQTime.setHMS(0,0,0,0);
        myQTime.start();


    }

}

void Multitrack::toggleBlobsView()
{
    if(ui.subtractioncheckBox->isChecked())

    {
        icpTracker->setVideoShowing(false);
    }
    if(!ui.subtractioncheckBox->isChecked())

    {
        icpTracker->setVideoShowing(true);
    }
}

void Multitrack::updateImage(Mat dataimage)
{


    qimage = QImage((const uchar*)dataimage.data, dataimage.cols, dataimage.rows, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();

    qimage = qimage.scaled(displayWidth, displayHeight);
   // qimage = qimage.scaledToWidth(displayWidth);

    imageLabel->setPixmap(QPixmap::fromImage(qimage));


    return;
}

bool Multitrack::checkreadytoPlay()
{
    QString notprepared="<b>Not Ready!</b>  <br><br><i>Please choose a...</i> <br>";
    if (videopath==nopath)
        notprepared= notprepared+"   Video Source<br>";
    if (bgpath==nopath)
        notprepared= notprepared+"   Background Image <br>";
    if (modelfolder==nopath)
        notprepared= notprepared+"   Model Image <br>";


    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath){

        loadNewTracker();


        QString colour="green"; // you can use also QColor
        QString text="<b>Ready to Track!</b> <br> <br> Press Play Button to Begin";
        QString fonttemplate = tr("<font color='%1'>%2</font>");
        ui.imageLabel->setText(fonttemplate.arg( colour, text ));

        return true;
    }
    else{


        QString colour="red"; // you can use also QColor
        QString text=notprepared;
        QString fonttemplate = tr("<font color='%1'>%2</font>");
        ui.imageLabel->setText(fonttemplate.arg( colour, text ));

        return false;
    }

}


void Multitrack::loadBackgroundFile()
{
    /* select a directory using file dialog */
    bgpath = QFileDialog::getOpenFileName (this, tr("Open Background File"),lastpath, tr("Images (*png *.jpg)"));
    if ( bgpath.isNull() == false )
    {
        lastpath=bgpath;
        ui.backgroundFile->setText("..."+bgpath.right(50));

        //Check if we are ready to play
        loadNewTracker();
    }
    else{
        bgpath=nopath;
        ui.backgroundFile->setText(bgpath.right(50));
    }
    loadNewTracker();


}

void Multitrack::loadVideoFile()
{
    /* select a directory using file dialog */
    videopath = QFileDialog::getOpenFileName (this, tr("Open Video File"),lastpath, tr("Multitrack Video (*.avi *.mov *.mpg *.mpeg)"));
    if ( videopath.isNull() == false )
    {
        lastpath=videopath;
        ui.videoFile->setText("..."+videopath.right(50));
    }
    else{
        videopath=nopath;
        ui.videoFile->setText(videopath.right(50));
    }
    loadNewTracker();
}

void Multitrack::loadModelFile()
{
    /* select a directory using file dialog */


       modelfolder = QFileDialog::getExistingDirectory(this, tr("Select Model File Folder"),lastpath);


    /*OLD select a single file*/
//    modelpath = QFileDialog::getOpenFileName (this, tr("Open Model File"),lastpath, tr("Images (*png *.jpg)"));
    if (modelfolder.isNull() == false )
    {
        lastpath=modelfolder;
        ui.modelFile->setText("..."+modelfolder.right(50));

    }
    else{
        modelfolder=nopath;
        ui.modelFile->setText(modelfolder.right(50));
    }
    loadNewTracker();
}

void Multitrack::chooseMaskFile()
{
    /* select a directory using file dialog */
    maskpath = QFileDialog::getOpenFileName (this, tr("Open Mask File"),lastpath, tr("Images (*png *.jpg)"));
    if ( maskpath.isNull() == false )
    {
        lastpath=maskpath;
        ui.maskFile->setText("..."+maskpath.right(50));
    }
    else{
        maskpath=nopath;
        ui.maskFile->setText(maskpath.right(50));
    }
    loadNewTracker();
}

void Multitrack::toggleContourTracking()
{
    //Switch on an off using the entire detection or just the contour ridges
    if(ui.contourTrackingcheckBox->isChecked()){
        //ui.contourLabel->setText("Contour Tracking ON");
        icpTracker->setContourTracking(true);
    }
    else{
        //   ui.contourLabel->setText("Contour Tracking OFF");
        icpTracker->setContourTracking(false);
    }


}


void Multitrack::chooseProjectDirectory()
{
    //Dialog to choose where to save
    /* select a directory using file dialog */
    //SAVE FILE  QString savepath = QFileDialog::getSaveFileName (this, tr("Select BTF directory"),"", tr("BTF data (*.btf)"));
    projectSaveDirectory = QFileDialog::getExistingDirectory (this, tr("Select BTF directory"),lastpath);
    
    if ( projectSaveDirectory.isNull() == false )
    {
        ui.projectDirectoryButton->setText("..."+projectSaveDirectory.right(50)+"/");
        lastpath=projectSaveDirectory;
        projectSaveDirectory=projectSaveDirectory+"/";


    }
    else{
        projectSaveDirectory="";
        ui.projectDirectoryButton->setText("defaultdirectory/");
    }

}

/*
  *Load and Save Settings
  *
  */



void Multitrack::loadDefaults(){
    ///Bring in nice defaults for people
    //SpinBoxes
    ui.bgSubThresholdSpinBox->setValue(55);
    ui.blobBirthAreaThresholdSpinBox->setValue(20);
    ui.resolutionSpinBox->setValue(8);
    ui.trackdistanceSpinBox->setValue(50);
    ui.trackdeathSpinBox->setValue(17);
    ui.separationSpinBox->setValue(4);
    //Check Boxes
    ui.subtractioncheckBox->setChecked(false);
    ui.contourTrackingcheckBox->setChecked(false);

}

void Multitrack::loadSettings(){

    if(isPlaying){
        toggleStopButton();
    }

    QString loadsetpath = QFileDialog::getOpenFileName (this, tr("Open Settings File"),lastpath);
    if ( loadsetpath.isNull() == false )
    {
        QSettings settings(loadsetpath,QSettings::IniFormat);
        //QSettings settings("Biotracking", "Multitrack");

        //SpinBoxes
        ui.bgSubThresholdSpinBox->setValue(settings.value("bgsubthresh",ui.bgSubThresholdSpinBox->value()).toInt());
        ui.blobBirthAreaThresholdSpinBox->setValue(settings.value("blobbirth", ui.blobBirthAreaThresholdSpinBox->value()).toInt());
        ui.resolutionSpinBox->setValue(settings.value("resolutionAnal", ui.resolutionSpinBox->value()).toInt());
        ui.trackdistanceSpinBox->setValue(settings.value("trackdist", ui.trackdistanceSpinBox->value()).toInt());
        ui.trackdeathSpinBox->setValue(settings.value("trackdeath",ui.trackdeathSpinBox->value()).toInt());
        ui.separationSpinBox->setValue(settings.value("separation",ui.separationSpinBox->value()).toInt());

        ui.ICP_MaxIterspinBox->setValue(settings.value("icpmaxiterations",ui.ICP_MaxIterspinBox->value()).toInt());
        ui.ICP_EuclideanDistdoubleSpinBox->setValue(settings.value("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value()).toDouble());
        ui.ICP_TransEpsilondoubleSpinBox->setValue(settings.value("icpepsilon",ui.ICP_TransEpsilondoubleSpinBox->value()).toDouble());

        //Check Boxes
        ui.subtractioncheckBox->setChecked(settings.value("bgsubcheck",false).toBool()); // again, the "false" value will be used in case of problem with QSettings
        ui.contourTrackingcheckBox->setChecked(settings.value("contourtracking", false).toBool());
        ui.modelViewcheckBox->setChecked(settings.value("modelshow", true).toBool());
        ui.separationViewCheck->setChecked(settings.value("separationview",false).toBool());




        //Files Previously Opened
        bgpath= settings.value("bgpath1",nopath).toString();
        ui.backgroundFile->setText("..."+bgpath.right(50));

        maskpath= settings.value("maskpath1",nopath).toString();
        ui.maskFile->setText("..."+maskpath.right(50));

        videopath= settings.value("videopath1",nopath).toString();
        ui.videoFile->setText("..."+videopath.right(50));

        modelfolder= settings.value("modelpath1",nopath).toString();
        ui.modelFile->setText("..."+modelfolder.right(50));

        projectSaveDirectory= settings.value("projpath1","").toString();
        if(projectSaveDirectory==""){
            ui.projectDirectoryButton->setText("...default_directory/");
        }
        else{
            ui.projectDirectoryButton->setText("..."+projectSaveDirectory.right(50)+"/");
        }


        lastpath=videopath;
        loadNewTracker();
        settings.sync();
    }
}

//Save all values to a single file for easy loading and reloading

void Multitrack::saveSettings(){
    if(isPlaying){
        toggleStopButton();
    }



    QString savesetpath = QFileDialog::getSaveFileName (this, tr("Save UI Settings"),ui.projectsavename->text()+".ini",tr("Settings files (*.ini"));
    if ( savesetpath.isNull() == false )  {

        QSettings settings(savesetpath, QSettings::IniFormat);
        //SpinBoxes
        settings.setValue("bgsubthresh", ui.bgSubThresholdSpinBox->value());
        settings.setValue("blobbirth", ui.blobBirthAreaThresholdSpinBox->value());
        settings.setValue("resolutionAnal", ui.resolutionSpinBox->value());
        settings.setValue("trackdist", ui.trackdistanceSpinBox->value());
        settings.setValue("trackdeath", ui.trackdeathSpinBox->value());
        settings.setValue("separation", ui.separationSpinBox->value());

        settings.setValue("icpmaxiterations",ui.ICP_MaxIterspinBox->value());
        settings.setValue("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value());
        settings.setValue("icpepsilon", ui.ICP_TransEpsilondoubleSpinBox->value());


        //Previously opened files
        settings.setValue("bgpath1",bgpath);
        settings.setValue("videopath1",videopath);
        settings.setValue("maskpath1",maskpath);
        settings.setValue("modelpath1",modelfolder);

        settings.setValue("projpath1",projectSaveDirectory);

        //Toggle Boxes
        settings.setValue("bgsubcheck",ui.subtractioncheckBox->isChecked()); // store a bool
        settings.setValue("contourtracking",ui.contourTrackingcheckBox->isChecked()); // store a bool
        settings.setValue("modelshow",ui.modelViewcheckBox->isChecked()); // store a bool
        settings.setValue("separationview",ui.separationViewCheck->isChecked()); // store a bool

        settings.sync();
    }
}


//Save all settings to Operating system on close
//TODO make it work again
void Multitrack::closeEvent(QCloseEvent *event)
{

    writeSettings();

}

void Multitrack::writeSettings()
{
    QSettings settings("Biotracking", "ManyTrack");

    //SpinBoxes
    settings.setValue("bgsubthresh", ui.bgSubThresholdSpinBox->value());
    settings.setValue("blobbirth", ui.blobBirthAreaThresholdSpinBox->value());
    settings.setValue("resolutionAnal", ui.resolutionSpinBox->value());
    settings.setValue("trackdist", ui.trackdistanceSpinBox->value());
    settings.setValue("trackdeath", ui.trackdeathSpinBox->value());
    settings.setValue("separation", ui.separationSpinBox->value());

    settings.setValue("icpmaxiterations",ui.ICP_MaxIterspinBox->value());
    settings.setValue("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value());
    settings.setValue("icpepsilon", ui.ICP_TransEpsilondoubleSpinBox->value());


    //Previously opened files
    settings.setValue("bgpath1",bgpath);
    settings.setValue("videopath1",videopath);
    settings.setValue("maskpath1",maskpath);
    settings.setValue("modelpath1",modelfolder);

    settings.setValue("projpath1",projectSaveDirectory);

    //Toggle Boxes
    settings.setValue("bgsubcheck",ui.subtractioncheckBox->isChecked()); // store a bool
    settings.setValue("contourtracking",ui.contourTrackingcheckBox->isChecked()); // store a bool
    settings.setValue("modelshow",ui.modelViewcheckBox->isChecked()); // store a bool
    settings.setValue("separationview",ui.separationViewCheck->isChecked()); // store a bool




}

void Multitrack::readSettings()
{
    QSettings settings("Biotracking", "ManyTrack");

    //settings.beginGroup("MainWindow");
    //SpinBoxes
    ui.bgSubThresholdSpinBox->setValue(settings.value("bgsubthresh",ui.bgSubThresholdSpinBox->value()).toInt());
    ui.blobBirthAreaThresholdSpinBox->setValue(settings.value("blobbirth", ui.blobBirthAreaThresholdSpinBox->value()).toInt());
    ui.resolutionSpinBox->setValue(settings.value("resolutionAnal", ui.resolutionSpinBox->value()).toInt());
    ui.trackdistanceSpinBox->setValue(settings.value("trackdist", ui.trackdistanceSpinBox->value()).toInt());
    ui.trackdeathSpinBox->setValue(settings.value("trackdeath",ui.trackdeathSpinBox->value()).toInt());
    ui.separationSpinBox->setValue(settings.value("separation",ui.separationSpinBox->value()).toInt());

    ui.ICP_MaxIterspinBox->setValue(settings.value("icpmaxiterations",ui.ICP_MaxIterspinBox->value()).toInt());
    ui.ICP_EuclideanDistdoubleSpinBox->setValue(settings.value("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value()).toDouble());
    ui.ICP_TransEpsilondoubleSpinBox->setValue(settings.value("icpepsilon",ui.ICP_TransEpsilondoubleSpinBox->value()).toDouble());

    //Check Boxes
    ui.subtractioncheckBox->setChecked(settings.value("bgsubcheck",false).toBool()); // again, the "false" value will be used in case of problem with QSettings
    ui.contourTrackingcheckBox->setChecked(settings.value("contourtracking", false).toBool());
    ui.modelViewcheckBox->setChecked(settings.value("modelshow", true).toBool());
    ui.separationViewCheck->setChecked(settings.value("separationview",false).toBool());




    //Files Previously Opened
    bgpath= settings.value("bgpath1",nopath).toString();
    ui.backgroundFile->setText("..."+bgpath.right(50));

    maskpath= settings.value("maskpath1",nopath).toString();
    ui.maskFile->setText("..."+maskpath.right(50));

    videopath= settings.value("videopath1",nopath).toString();
    ui.videoFile->setText("..."+videopath.right(50));

    modelfolder= settings.value("modelpath1",nopath).toString();
    ui.modelFile->setText("..."+modelfolder.right(50));

    projectSaveDirectory= settings.value("projpath1","").toString();
    if(projectSaveDirectory==""){
        ui.projectDirectoryButton->setText("...default_directory/");
    }
    else{
        ui.projectDirectoryButton->setText("..."+projectSaveDirectory.right(50)+"/");
    }

    lastpath=videopath;

}








void Multitrack::icpReset(){

    capture.release();
    delete imageData;
    delete icpTracker;
    loadNewTracker();

}
void Multitrack::loadNewTracker(){

completedTracking=false;
    //Open all Image Assets and check to see if they are OK before creating a new ICPTRACKER object
    if(trackerCheck()){
        //All items checked out OK continue!
        icpTracker = new ICPTracker(vidFPS,bgpath,modelfolder,maskpath, ui);

        icpTracker->showModel=ui.modelViewcheckBox->isChecked();
        icpTracker->showRemovalRadii=ui.separationViewCheck->isChecked();
        icpTracker->showSearchRadius= ui.trackDistanceViewCheck->isChecked();
        icpTracker->showTrails = ui.showTrailscheckBox->isChecked();
        icpTracker->showBox= ui.showBoxcheckBox->isChecked();


        //Load UI settings into Tracker
        icpTracker->setResolutionFraction(ui.resolutionSpinBox->value());
        icpTracker->setTrackDeathThreshold(ui.trackdeathSpinBox->value());
        icpTracker->setMatchDistanceThreshold(ui.trackdistanceSpinBox->value());
        icpTracker->setSeparationThreshold(ui.separationSpinBox->value());

        icpTracker->setContourTracking(ui.contourTrackingcheckBox->isChecked());

        icpTracker->setBgSubThreshold(ui.bgSubThresholdSpinBox->value());
        icpTracker->setTrackBirthAreaThreshold(ui.blobBirthAreaThresholdSpinBox->value());

        icpTracker->Ticp_maxIter=ui.ICP_MaxIterspinBox->value();

        icpTracker->Ticp_transformationEpsilon=ui.ICP_TransEpsilondoubleSpinBox->value();

        icpTracker->Ticp_euclideanDistance=ui.ICP_EuclideanDistdoubleSpinBox->value();

        pHour =0;
        pMin=0;
        pSec=0;
        pMsec=0;
        pausedMillis=0;



        ui.resetButton->setEnabled(false);
        ui.stopButton->setEnabled(true);
        ui.stopButton->setText(tr("PLAY"));
        toggleBlobsView();
        ui.toolbartabWidget->setCurrentWidget(ui.controlsTabWidget); // if they are all set with the files, switch automatically to controls.
    }
    else{     ui.stopButton->setEnabled(false);
    }
}

bool Multitrack::trackerCheck(){
    bool everythingok=true;

    QString error="ERROR:  ";
    QString colour="red"; // you can use also QColor

    QString fonttemplate = "<font color='%1'>%2</font>";


    QString notprepared="<b>Not Ready!</b>  <br><br><i>Please choose a...</i> <br>";
    if (videopath==nopath)
        notprepared= notprepared+"   Video Source<br>";
    if (bgpath==nopath)
        notprepared= notprepared+"   Background Image <br>";
    if (modelfolder==nopath)
        notprepared= notprepared+"   Model Image <br>";


    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath){




        colour="green"; // you can use also QColor
        QString text="<b>Ready to Track!</b> <br> <br> Press Play Button to Begin";
        fonttemplate = tr("<font color='%1'>%2</font>");
        ui.imageLabel->setText(fonttemplate.arg( colour, text ));

    }
    else{
        //ui.blobsButton->setEnabled(false);

        colour="red"; // you can use also QColor
        QString text=notprepared;
        fonttemplate = tr("<font color='%1'>%2</font>");
        ui.imageLabel->setText(fonttemplate.arg( colour, text ));
        everythingok=false;
    }

    colour="red";
    //Video Capture
    capture.open(videopath.toStdString());
    //Find Properties of the Video File
    vidFPS = capture.get(CV_CAP_PROP_FPS);
    cout << "Frame rate   " <<vidFPS;


    //Load background image
    bgImage = imread(bgpath.toStdString());
    if(bgImage.empty()){
        error=error+"BG File is missing/empty or broken <br>";

        ui.imageLabel->setText(fonttemplate.arg( colour, error ));
        everythingok=false;
    }


    //make mask conditional
    if(maskpath.isNull() || maskpath=="(none selected)"){
        //Don't load the mask or care about it!
    }
    else{
        //the user chose a mask for us to use!
        Mat maskImage;
        //maskImage =bgImage.clone();
        maskImage = imread(maskpath.toStdString()); //Can put flag if want to include transparency, else strips Alpha channel

        if(maskImage.empty()){
            error=error+"MASK File is missing/empty or broken<br>";

            ui.imageLabel->setText(fonttemplate.arg( colour, error ));
            everythingok=false;
        }

    }

    //TODO check model in better way
    QDir myDir( modelfolder);

    QStringList filters;
    filters<<"*.png";
    myDir.setNameFilters(filters);

    QStringList list = myDir.entryList(filters);
    qDebug()<<list;
    // load model image
    if(list.isEmpty()){

        error=error+"MODEL Path is not a directory<br>";

    }
    else{
    modelImage = imread(modelfolder.toStdString()+"/"+list.at(0).toStdString()); //zero means force grayscale load
    if(modelImage.empty()){
        error=error+"MODEL File is missing/empty or broken<br>";
        ui.imageLabel->setText(fonttemplate.arg( colour, error ));
        everythingok=false;
    }
}


    //check that BG.size==video.size==mask.size
    Mat img;
    capture.retrieve(img);
    capture.read(img);
    cout<<"capture image size "<<img.size<<endl;
    cout<<"bg image size "<<bgImage.size<<endl;
    if(bgImage.size.operator !=(img.size)){
        error=error+"Video and BG do not match! <br>";
        ui.imageLabel->setText(fonttemplate.arg( colour, error ));
        cout<<"Sizes no match! "<<bgImage.size<<endl;
        everythingok=false;
    }

    trackerChecked=everythingok;
    return everythingok;
}



/*   Change Viewing preferences for display screen */

void Multitrack::on_displaycomboBox_currentIndexChanged(int index)
{        displayWidth = ui.scrollArea->width() -20;
         displayHeight = ui.scrollArea->height()-20;

              if(!bgImage.empty()){

                  if(index==1){ // 100% view
                      displayWidth = bgImage.cols;
                      displayHeight= bgImage.rows;

                      //     qimage = qimage.scaled(qimage.width(), qimage.height());// TODO make this display adjustable
                      //  ui.scrollAreaWidgetContents->setGeometry(0,0,qimage.width(), qimage.height());
                  }
                  else if(index==2){ // 50% view
                      displayWidth = bgImage.cols/2;
                      displayHeight= bgImage.rows/2;

                      // qimage = qimage.scaled(qimage.width()/2, qimage.height()/2); // TODO make this display adjustable
                      // ui.scrollAreaWidgetContents->setGeometry(0,0,qimage.width(), qimage.height());

                  }
                  else if(index==0){ //Fit to Window mode
                      displayWidth = ui.scrollArea->width() -20;
                      displayHeight = ui.scrollArea->height()-20;
                      //   qimage = qimage.scaledToWidth(ui.scrollArea->width()); // TODO make this display adjustable
                      // ui.scrollAreaWidgetContents->setGeometry(0,0,ui.imageLabel->width(), ui.imageLabel->height());

                  }
              }



              imageLabel->resize(displayWidth,displayHeight);
                   ui.visualizationLabel->resize(ui.imageLabel->size());
                   if(trackerChecked){

                       updateImage(icpTracker->getTrackResultImage());
                   }
                   ui.scrollAreaWidgetContents->resize(displayWidth,displayHeight);

}

void Multitrack::on_trackDistanceViewCheck_toggled(bool checked)
{
    if(trackerChecked||isPlaying)
        icpTracker->showSearchRadius = checked;
}

void Multitrack::on_modelViewcheckBox_toggled(bool checked)
{
    if(trackerChecked||isPlaying)
        icpTracker->showModel = checked;
}

void Multitrack::on_showTrailscheckBox_toggled(bool checked)
{
    if(trackerChecked||isPlaying)
        icpTracker->showTrails= checked;
}

void Multitrack::on_showBoxcheckBox_toggled(bool checked)
{
    if(trackerChecked||isPlaying)
        icpTracker->showBox = checked;
}

void Multitrack::on_separationViewCheck_toggled(bool checked)
{
    if(trackerChecked||isPlaying)
        icpTracker->showRemovalRadii = checked;
}

void Multitrack::on_bgSubThresholdSpinBox_valueChanged(int arg1)
{

    ui.bgsubSlider->setValue(arg1);
}

void Multitrack::on_bgsubSlider_sliderMoved(int position)
{
    ui.bgSubThresholdSpinBox->setValue(position);
}

void Multitrack::on_blobBirthAreaThresholdSpinBox_valueChanged(int arg1)
{
    ui.matchSlider->setValue(arg1);
}

void Multitrack::on_matchSlider_sliderMoved(int position)
{
    ui.blobBirthAreaThresholdSpinBox->setValue(position);
}

void Multitrack::on_trackdeathSpinBox_valueChanged(int arg1)
{
    ui.deathSlider->setValue(arg1);
}

void Multitrack::on_deathSlider_sliderMoved(int position)
{
    ui.trackdeathSpinBox->setValue(position);
}

void Multitrack::on_separationSpinBox_valueChanged(int arg1)
{
    ui.separationSlider->setValue(arg1);
}
void Multitrack::on_separationSlider_sliderMoved(int position)
{
    ui.separationSpinBox->setValue(position);
}



void Multitrack::on_trackdistanceSpinBox_valueChanged(int arg1)
{
    ui.searchSlider->setValue(arg1);
}

void Multitrack::on_searchSlider_sliderMoved(int position)
{
    ui.trackdistanceSpinBox->setValue(position);
}

//ICP Spin Boxes

void Multitrack::on_ICP_MaxIterspinBox_valueChanged(int arg1)
{
    if(trackerChecked)
        icpTracker->Ticp_maxIter=arg1;
}

void Multitrack::on_ICP_TransEpsilondoubleSpinBox_valueChanged(double arg1)
{
    if(trackerChecked||isPlaying)
        icpTracker->Ticp_transformationEpsilon=arg1;
}

void Multitrack::on_ICP_EuclideanDistdoubleSpinBox_valueChanged(double arg1)
{
    if(trackerChecked||isPlaying)
        icpTracker->Ticp_euclideanDistance=arg1;
}

void Multitrack::on_visualizationcheckBox_toggled(bool checked)
{
    if (checked)
 ui.visualizationLabel->raise();
    else ui.visualizationLabel->lower();
}
