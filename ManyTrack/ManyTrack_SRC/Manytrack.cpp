#include "Manytrack.h"


Manytrack::Manytrack(QWidget *parent, Qt::WFlags flags)
    : QMainWindow(parent, flags)
{

//Global Settings to Apply the everytime the program is run first time

    trackerChecked=false;
    nopath="(none selected)";
    bgpath=nopath;
    modelfolder=nopath;
    maskpath=nopath;
    videopath=nopath;
    projectSaveDirectory="";
    lastpath="";

    ui.setupUi(this);

    setWindowIcon(QIcon("Manytrack.png"));

    imageData = NULL;
    icpTracker= NULL;
    imageLabel = ui.imageLabel;

imageLabel->setScaledContents(true);


    isTracking = false;
    isVideoShowing=true;
    completedTracking=false;

//Connect UI buttons to appropriate slots
    connectUI();

    ui.visualizationLabel->setAttribute(Qt::WA_TranslucentBackground);

    //Load the default settings from the last time the program was used
    readSettings();

    loadNewTracker();

    //Set the Normal zoom view (prevents users from getting lost)
    on_displaycomboBox_currentIndexChanged(0);

    //Start the Timer Running (This controls the speed of QT's main loop)
    startTimer(0);  // High speed-second timer, tries to go as fast as it can
    //startTimer(1000/300000000);  // example 0.1-second timer

}

Manytrack::~Manytrack()
{

    capture.release(); // Gives closing errors sometimes
    delete imageData;
    delete icpTracker;

}

void Manytrack::timerEvent(QTimerEvent*) {
    if (!isTracking){
        return;} //Don't run the timer



    else{
        //This is the main tracking loop. The program attempts to:
        // go through every frame of the video,
        // track each frame
        // and stop at the end, ready to repeat or let the user change parameters

     double   t = (double)getTickCount();

        currentFrameImg = updateFrame();

        t = ((double)getTickCount() - t)/getTickFrequency();
        cout << "::: Get updateFrame() " << t << endl;

        if(completedTracking==false){

            t = (double)getTickCount();

            icpTracker->processFrame(currentFrameImg, (int)capture.get(CV_CAP_PROP_POS_FRAMES));

            t = ((double)getTickCount() - t)/getTickFrequency();
            cout << "::: ICPTracker processFrame() " << t << endl;


            if(ui.display_pushButton->isChecked()){

                t = (double)getTickCount();

            updateVideoImage(currentFrameImg);

            t = ((double)getTickCount() - t)/getTickFrequency();
            cout << "::: updateVideoImage() " << t << endl;


            t = (double)getTickCount();
                      updateVisualization(icpTracker->getTrackResultImage());
                       t = ((double)getTickCount() - t)/getTickFrequency();
            cout << "::: updateVisualization ()  " << t << endl;
           }
        }
        updateStatusBar();
    }
}

Mat Manytrack::updateFrame()
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

            toggleTracking();
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
        toggleTracking();
        ui.stopButton->setEnabled(false);
        ui.resetButton->setEnabled(true);


        return img;
    }

}

void Manytrack::updateStatusBar()
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

void Manytrack::messageToStatusBar(QString message)
{
    statusBar()->showMessage(tr(message.toAscii()));
}

void Manytrack::nextFrame()
{
    Mat img = updateFrame();
    updateVideoImage(img);
    updateStatusBar();
}


void Manytrack::saveHTMLinteractions()
{
    messageToStatusBar("saving interaction report...");
    icpTracker->outputInteractionsReport();
    messageToStatusBar("done saving interaction report");
}

void Manytrack::saveBTF()
{
    messageToStatusBar("saving BTF report...");
    icpTracker->outputBTF(projectSaveDirectory,ui.projectsavename->text());
    QString messageo= "done saving BTF data for project: " +ui.projectsavename->text();
    messageToStatusBar(messageo.toAscii());
}

void Manytrack::bgThresholdSpinValueChanged(int value)
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath){
        ui.bgSubThresholdSpinBox->setValue(value);
        icpTracker->setBgSubThreshold(ui.bgSubThresholdSpinBox->value());
    }
}


void Manytrack::blobBirthAreaThresholdValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        if(ui.healthyPercentageThresholdSpinBox->value() > 0){
            icpTracker->setTrackBirthAreaThreshold(ui.healthyPercentageThresholdSpinBox->value());
        }
}

void Manytrack::resolutionFractionValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setResolutionFraction(ui.resolutionSpinBox->value());
}
void Manytrack::trackdistanceValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setMatchDistanceThreshold(ui.trackdistanceSpinBox->value());
}
void Manytrack::trackDeathValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setTrackDeathThreshold(ui.trackdeathSpinBox->value());
}
void Manytrack::separationValueChanged()
{
    if (bgpath!=nopath && videopath!=nopath && modelfolder!=nopath)
        icpTracker->setSeparationThreshold(ui.separationSpinBox->value());
}

void Manytrack::toggleTracking()
{
    if (isTracking)
    {
        //Turn on or off UI components

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

        ui.framesSlider->setEnabled(true);
        ui.framesspinBox->setEnabled(true);
        ui.previewtrackingButton->setEnabled(true);

        isTracking = !isTracking;

        //Pause the Timer
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

        ui.framesSlider->setEnabled(false);
        ui.framesspinBox->setEnabled(false);
        ui.previewtrackingButton->setEnabled(false);


        isTracking = !isTracking;

        //Calculate the Paused Duration
        myQTime.setHMS(0,0,0,0);
        myQTime.start();


    }

}


void Manytrack::updateVideoImage(Mat dataimage)
{

    //TEST FOR FILTERing
//    blur(dataimage,dataimage,cv::Size(ui.blurspinBox->value(),ui.blurspinBox->value()));

    qimage = QImage((const uchar*)dataimage.data, dataimage.cols, dataimage.rows, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();

//    qimage = qimage.scaled(displayWidth, displayHeight);

   // qimage = qimage.scaledToWidth(displayWidth);

    imageLabel->setPixmap(QPixmap::fromImage(qimage).scaledToWidth((int) displayWidth));


    return;
}

void Manytrack::updateVisualization(Mat qImgARGB)
{
    QImage qimage;
    qimage = QImage((const uchar*)qImgARGB.data, qImgARGB.cols, qImgARGB.rows,qImgARGB.step, QImage::Format_ARGB32);
    qimage = qimage.rgbSwapped();
//    qimage = qimage.scaled(displayWidth, displayHeight);

    ui.visualizationLabel->setPixmap(QPixmap::fromImage(qimage).scaledToWidth((int) displayWidth));
}


bool Manytrack::checkreadytoPlay()
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


void Manytrack::loadBackgroundFile()
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

void Manytrack::loadVideoFile()
{
    /* select a directory using file dialog */
    videopath = QFileDialog::getOpenFileName (this, tr("Open Video File"),lastpath, tr("Manytrack Video (*.avi *.mov *.mpg *.mpeg)"));
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

void Manytrack::loadModelFile()
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

void Manytrack::chooseMaskFile()
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

void Manytrack::toggleContourTracking()
{



}


void Manytrack::chooseProjectDirectory()
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



void Manytrack::loadDefaults(){
    ///Bring in nice defaults for people
    //SpinBoxes
    ui.bgSubThresholdSpinBox->setValue(55);
    ui.healthyPercentageThresholdSpinBox->setValue(20);
    ui.resolutionSpinBox->setValue(8);
    ui.trackdistanceSpinBox->setValue(50);
    ui.trackdeathSpinBox->setValue(17);
    ui.separationSpinBox->setValue(4);
    //Check Boxes
    ui.subtractioncheckBox->setChecked(false);

    ui.colorRegSpinBox->setValue(1.00);

}

void Manytrack::loadSettings(){

    if(isTracking){
        toggleTracking();
    }

    QString loadsetpath = QFileDialog::getOpenFileName (this, tr("Open Settings File (.ini)"),lastpath, tr("Settings files (*.ini)"));
    if ( loadsetpath.isNull() == false )
    {
        QSettings settings(loadsetpath,QSettings::IniFormat);
        //QSettings settings("Biotracking", "Manytrack");

        //SpinBoxes
        ui.bgSubThresholdSpinBox->setValue(settings.value("bgsubthresh",ui.bgSubThresholdSpinBox->value()).toInt());
        ui.healthyPercentageThresholdSpinBox->setValue(settings.value("blobbirth", ui.healthyPercentageThresholdSpinBox->value()).toInt());
        ui.resolutionSpinBox->setValue(settings.value("resolutionAnal", ui.resolutionSpinBox->value()).toInt());
        ui.trackdistanceSpinBox->setValue(settings.value("trackdist", ui.trackdistanceSpinBox->value()).toInt());
        ui.trackdeathSpinBox->setValue(settings.value("trackdeath",ui.trackdeathSpinBox->value()).toInt());
        ui.separationSpinBox->setValue(settings.value("separation",ui.separationSpinBox->value()).toInt());

        ui.ICP_MaxIterspinBox->setValue(settings.value("icpmaxiterations",ui.ICP_MaxIterspinBox->value()).toInt());
        ui.ICP_EuclideanDistdoubleSpinBox->setValue(settings.value("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value()).toDouble());
        ui.ICP_TransEpsilondoubleSpinBox->setValue(settings.value("icpepsilon",ui.ICP_TransEpsilondoubleSpinBox->value()).toDouble());

        ui.colorRegSpinBox->setValue(settings.value("colorRegspinbox",ui.colorRegSpinBox->value()).toDouble());

        ui.icp_maxWorstScore->setValue(settings.value("icpmaxworstscore",ui.icp_maxWorstScore->value()).toInt());


        //Check Boxes
        ui.subtractioncheckBox->setChecked(settings.value("bgsubcheck",false).toBool()); // again, the "false" value will be used in case of problem with QSettings
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

void Manytrack::saveSettings(){
    if(isTracking){
        toggleTracking();
    }


    QString savesetpath = QFileDialog::getSaveFileName (this, tr("Save UI Settings"),lastpath+ui.projectsavename->text()+".ini",tr("Settings files (*.ini)"));
    if ( savesetpath.isNull() == false )  {

        QSettings settings(savesetpath, QSettings::IniFormat);
        //SpinBoxes
        settings.setValue("bgsubthresh", ui.bgSubThresholdSpinBox->value());
        settings.setValue("blobbirth", ui.healthyPercentageThresholdSpinBox->value());
        settings.setValue("resolutionAnal", ui.resolutionSpinBox->value());
        settings.setValue("trackdist", ui.trackdistanceSpinBox->value());
        settings.setValue("trackdeath", ui.trackdeathSpinBox->value());
        settings.setValue("separation", ui.separationSpinBox->value());

        settings.setValue("icpmaxiterations",ui.ICP_MaxIterspinBox->value());
        settings.setValue("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value());
        settings.setValue("icpepsilon", ui.ICP_TransEpsilondoubleSpinBox->value());

 settings.setValue("colorRegspinbox",ui.colorRegSpinBox->value());

 settings.setValue("icpmaxworstscore",ui.ICP_MaxIterspinBox->value());




        //Previously opened files
        settings.setValue("bgpath1",bgpath);
        settings.setValue("videopath1",videopath);
        settings.setValue("maskpath1",maskpath);
        settings.setValue("modelpath1",modelfolder);

        settings.setValue("projpath1",projectSaveDirectory);

        //Toggle Boxes
        settings.setValue("bgsubcheck",ui.subtractioncheckBox->isChecked()); // store a bool
        settings.setValue("modelshow",ui.modelViewcheckBox->isChecked()); // store a bool
        settings.setValue("separationview",ui.separationViewCheck->isChecked()); // store a bool

        settings.sync();
    }
}


//Save all settings to Operating system on close
//TODO make it work again
void Manytrack::closeEvent(QCloseEvent *event)
{

    writeSettings();

}

void Manytrack::writeSettings()
{
    QSettings settings("Biotracking", "ManyTrack");

    //SpinBoxes
    settings.setValue("bgsubthresh", ui.bgSubThresholdSpinBox->value());
    settings.setValue("blobbirth", ui.healthyPercentageThresholdSpinBox->value());
    settings.setValue("resolutionAnal", ui.resolutionSpinBox->value());
    settings.setValue("trackdist", ui.trackdistanceSpinBox->value());
    settings.setValue("trackdeath", ui.trackdeathSpinBox->value());
    settings.setValue("separation", ui.separationSpinBox->value());

    settings.setValue("icpmaxiterations",ui.ICP_MaxIterspinBox->value());
    settings.setValue("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value());
    settings.setValue("icpepsilon", ui.ICP_TransEpsilondoubleSpinBox->value());

    settings.setValue("colorRegspinbox",ui.colorRegSpinBox->value());
    settings.setValue("icpmaxworstscore",ui.icp_maxWorstScore->value());




    //Previously opened files
    settings.setValue("bgpath1",bgpath);
    settings.setValue("videopath1",videopath);
    settings.setValue("maskpath1",maskpath);
    settings.setValue("modelpath1",modelfolder);

    settings.setValue("projpath1",projectSaveDirectory);

    //Toggle Boxes
    settings.setValue("bgsubcheck",ui.subtractioncheckBox->isChecked()); // store a bool
    settings.setValue("modelshow",ui.modelViewcheckBox->isChecked()); // store a bool
    settings.setValue("separationview",ui.separationViewCheck->isChecked()); // store a bool




}

void Manytrack::readSettings()
{
    QSettings settings("Biotracking", "ManyTrack");

    //settings.beginGroup("MainWindow");
    //SpinBoxes
    ui.bgSubThresholdSpinBox->setValue(settings.value("bgsubthresh",ui.bgSubThresholdSpinBox->value()).toInt());
    ui.healthyPercentageThresholdSpinBox->setValue(settings.value("blobbirth", ui.healthyPercentageThresholdSpinBox->value()).toInt());
    ui.resolutionSpinBox->setValue(settings.value("resolutionAnal", ui.resolutionSpinBox->value()).toInt());
    ui.trackdistanceSpinBox->setValue(settings.value("trackdist", ui.trackdistanceSpinBox->value()).toInt());
    ui.trackdeathSpinBox->setValue(settings.value("trackdeath",ui.trackdeathSpinBox->value()).toInt());
    ui.separationSpinBox->setValue(settings.value("separation",ui.separationSpinBox->value()).toInt());

    ui.ICP_MaxIterspinBox->setValue(settings.value("icpmaxiterations",ui.ICP_MaxIterspinBox->value()).toInt());
    ui.ICP_EuclideanDistdoubleSpinBox->setValue(settings.value("icpeuclidean",ui.ICP_EuclideanDistdoubleSpinBox->value()).toDouble());
    ui.ICP_TransEpsilondoubleSpinBox->setValue(settings.value("icpepsilon",ui.ICP_TransEpsilondoubleSpinBox->value()).toDouble());

    ui.colorRegSpinBox->setValue(settings.value("colorRegspinbox",ui.colorRegSpinBox->value()).toDouble());
    ui.icp_maxWorstScore->setValue(settings.value("icpmaxworstscore",ui.icp_maxWorstScore->value()).toInt());



    //Check Boxes
    ui.subtractioncheckBox->setChecked(settings.value("bgsubcheck",false).toBool()); // again, the "false" value will be used in case of problem with QSettings
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








void Manytrack::icpReset(){

    capture.release();
    delete imageData;
    delete icpTracker;
    loadNewTracker();

}
void Manytrack::loadNewTracker(){

 //Set Tracking to unfinished
completedTracking=false;

    //Open all Image Assets and check to see if they are OK before creating a new ICPTRACKER object
    if(trackerCheck()){
        //All items checked out OK continue!
        icpTracker = new ICPTracker(vidFPS,bgpath,modelfolder,maskpath, ui);



        pHour =0;
        pMin=0;
        pSec=0;
        pMsec=0;
        pausedMillis=0;



        ui.resetButton->setEnabled(false);
        ui.stopButton->setEnabled(true);
        ui.stopButton->setText(tr("PLAY"));
        ui.toolbartabWidget->setCurrentWidget(ui.controlsTabWidget); // if they are all set with the files, switch automatically to controls.
    }
    else{     ui.stopButton->setEnabled(false);
    }
}

bool Manytrack::trackerCheck(){
    bool everythingok=true;

    //TODO Check for MASK matches to BG and Video

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
    capture.release();
    capturepreview.release();
    capture.open(videopath.toStdString());
    capturepreview.open(videopath.toStdString()); //TODO, this probably isn't right
    ui.framesSlider->setMaximum(capturepreview.get(CV_CAP_PROP_FRAME_COUNT)-1);
    ui.framesspinBox->setMaximum(capturepreview.get(CV_CAP_PROP_FRAME_COUNT)-1);

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

void Manytrack::on_displaycomboBox_currentIndexChanged(int index)
{        displayWidth = ui.scrollArea->width() -20;
         displayHeight = ui.scrollArea->height()-20;

              if(!bgImage.empty()){

                  if(index==1){ // 100% view
                      displayWidth = bgImage.cols;
                      displayHeight= bgImage.rows;

                      //     qimage = qimage.scaled(qimage.width(), qimage.height());
                      //  ui.scrollAreaWidgetContents->setGeometry(0,0,qimage.width(), qimage.height());
                  }
                  else if(index==2){ // 50% view
                      displayWidth = bgImage.cols/2;
                      displayHeight= bgImage.rows/2;

                      // qimage = qimage.scaled(qimage.width()/2, qimage.height()/2);
                      // ui.scrollAreaWidgetContents->setGeometry(0,0,qimage.width(), qimage.height());

                  }
                  else if(index==0){ //Fit to Window mode
                      displayWidth = ui.scrollArea->width() -20;
                             displayHeight = ui.scrollArea->height()-20;
                     // displayWidth = ui.scrollArea->width();
                      //displayHeight = (int)(((double)bgImage.rows/ (double)bgImage.cols)*(double)displayWidth);
                      //   qimage = qimage.scaledToWidth(ui.scrollArea->width());
                      // ui.scrollAreaWidgetContents->setGeometry(0,0,ui.imageLabel->width(), ui.imageLabel->height());
//                      qDebug()<<
//                                 displayHeight <<"DISPLAY HEIGHT";

                  }
              }

              imageLabel->resize(displayWidth,displayHeight);
                   ui.visualizationLabel->resize(ui.imageLabel->size());
                   if(trackerChecked){

                       updateVideoImage(currentFrameImg);
                       updateVisualization(icpTracker->getTrackResultImage());

                   }
                   ui.scrollAreaWidgetContents->resize(displayWidth,displayHeight);

}

/*
* Connect UI buttons to slots
*/
void Manytrack::connectUI()
{

    ui.stopButton->setEnabled(false);
    ui.saveBTFButton->setEnabled(false);
    //ui.blobsButton->setEnabled(false);
    //connect(ui.blobsButton, SIGNAL(clicked()), this, SLOT(toggleBlobsView()));
    ui.subtractioncheckBox->setChecked(false);



    ui.resetButton->setEnabled(false);
    //ui.resolutionSpinBox->setValue(resFractionMultiplier);
    connect(ui.stopButton, SIGNAL(clicked()), this, SLOT(toggleTracking()));
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

    connect(ui.healthyPercentageThresholdSpinBox, SIGNAL(valueChanged(int)), this, SLOT(blobBirthAreaThresholdValueChanged()));
    connect(ui.resolutionSpinBox, SIGNAL(valueChanged(int)), this, SLOT(resolutionFractionValueChanged()));
    connect(ui.trackdistanceSpinBox, SIGNAL(valueChanged(int)), this, SLOT(trackdistanceValueChanged()));

    connect(ui.trackdeathSpinBox, SIGNAL(valueChanged(int)), this, SLOT(trackDeathValueChanged()));
    connect(ui.separationSpinBox, SIGNAL(valueChanged(int)), this, SLOT(separationValueChanged()));

    connect(ui.actionLoad_Settings, SIGNAL(triggered()), this, SLOT(loadSettings()));
    connect(ui.actionLoad_Defaults, SIGNAL(triggered()), this, SLOT(loadDefaults()));
    connect(ui.actionSave_All, SIGNAL(triggered()), this, SLOT(saveSettings()));

}

void Manytrack::on_trackDistanceViewCheck_toggled(bool checked)
{
    if(trackerChecked||isTracking)
        icpTracker->showSearchRadius = checked;
}

void Manytrack::on_modelViewcheckBox_toggled(bool checked)
{
    if(trackerChecked||isTracking)
        icpTracker->showModel = checked;
}

void Manytrack::on_showTrailscheckBox_toggled(bool checked)
{
    if(trackerChecked||isTracking)
        icpTracker->showTrails= checked;
}

void Manytrack::on_showBoxcheckBox_toggled(bool checked)
{
    if(trackerChecked||isTracking)
        icpTracker->showBox = checked;
}

void Manytrack::on_separationViewCheck_toggled(bool checked)
{
    if(trackerChecked||isTracking)
        icpTracker->showRemovalRadii = checked;
}

void Manytrack::on_bgSubThresholdSpinBox_valueChanged(int arg1)
{

    ui.bgsubSlider->setValue(arg1);
}

void Manytrack::on_bgsubSlider_sliderMoved(int position)
{
    ui.bgSubThresholdSpinBox->setValue(position);
}

void Manytrack::on_blobBirthAreaThresholdSpinBox_valueChanged(int arg1)
{
    ui.matchSlider->setValue(arg1);
}

void Manytrack::on_matchSlider_sliderMoved(int position)
{
    ui.healthyPercentageThresholdSpinBox->setValue(position);
}

void Manytrack::on_healthyPercentageThresholdSpinBox_valueChanged(int arg1)
{
    ui.matchSlider->setValue(arg1);
}


void Manytrack::on_trackdeathSpinBox_valueChanged(int arg1)
{
    ui.deathSlider->setValue(arg1);
}

void Manytrack::on_deathSlider_sliderMoved(int position)
{
    ui.trackdeathSpinBox->setValue(position);
}

void Manytrack::on_separationSpinBox_valueChanged(int arg1)
{
    ui.separationSlider->setValue(arg1);
}
void Manytrack::on_separationSlider_sliderMoved(int position)
{
    ui.separationSpinBox->setValue(position);
}



void Manytrack::on_trackdistanceSpinBox_valueChanged(int arg1)
{
    ui.searchSlider->setValue(arg1);
}

void Manytrack::on_searchSlider_sliderMoved(int position)
{
    ui.trackdistanceSpinBox->setValue(position);
}

//ICP Spin Boxes

void Manytrack::on_ICP_MaxIterspinBox_valueChanged(int arg1)
{
    if(trackerChecked)
        icpTracker->Ticp_maxIter=arg1;
}

void Manytrack::on_ICP_TransEpsilondoubleSpinBox_valueChanged(double arg1)
{
    if(trackerChecked||isTracking)
        icpTracker->Ticp_transformationEpsilon=arg1;
}

void Manytrack::on_ICP_EuclideanDistdoubleSpinBox_valueChanged(double arg1)
{
    if(trackerChecked||isTracking)
        icpTracker->Ticp_euclideanDistance=arg1;
}

void Manytrack::on_visualizationcheckBox_toggled(bool checked)
{
    if (checked)
 ui.visualizationLabel->raise();
    else ui.visualizationLabel->lower();
}
void Manytrack::on_framesspinBox_valueChanged(int arg1)
{
    ui.framesSlider->setValue(arg1);
}
void Manytrack::on_framesSlider_sliderMoved(int position)
{
    ui.framesspinBox->setValue(position);
}


void Manytrack::on_actionLive_Preview_triggered()
{

}

void Manytrack::on_previewtrackingButton_clicked()
{ //Perform new tracking on a single frame help aid the quality of the fit in multiple situations

    Mat img;


    capturepreview.set(CV_CAP_PROP_POS_FRAMES,ui.framesspinBox->value());

    ui.framesSlider->setMaximum(capturepreview.get(CV_CAP_PROP_FRAME_COUNT)-1);
    ui.framesspinBox->setMaximum(capturepreview.get(CV_CAP_PROP_FRAME_COUNT)-1);

    capturepreview.grab();
    if( capturepreview.retrieve(img)){ //capture.read is just grab+retreive, BUT it gives faults for frames <10 WTF?

        //Load New preview Tracker
        icpTrackerpreview = new ICPTracker(vidFPS,bgpath,modelfolder,maskpath, ui);


  icpTrackerpreview->processFrame(img, ui.framesSlider->value());


        updateVideoImage(img);
        updateVisualization(icpTrackerpreview->getTrackResultImage());
        delete icpTrackerpreview;

    }
    else{

        //WAHT
    }

}
