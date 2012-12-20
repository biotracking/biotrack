#include "backgrounder.h"
#include "ui_backgrounder.h"
#include "BackgroundCalculatorAverage.h"
#include "BackgroundCalculatorMode.h"

#define BACKGROUND_METHOD_AVERAGE 0
#define BACKGROUND_METHOD_MODE 1
#define BACKGROUND_METHOD_MEDIAN 2

Backgrounder::Backgrounder(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Backgrounder)
{
    nopath="(none selected)";
    bgpath=nopath;
    lastpath="";

    ui->setupUi(this);

    ui->saveBGButton->setEnabled(false);
    ui->stopButton->setEnabled(false);

    backgroundCalculator = NULL;
    isRunning=false;

    startTimer(0);  // High speed-second timer

}




Backgrounder::~Backgrounder()
{
    delete ui;
}

void Backgrounder::timerEvent(QTimerEvent*) {
    if(    ui->stopButton->isChecked()){
        //Process the background
        if(isRunning)
        {
            backgroundCalculator->step();
            ui->progressBar->setValue(backgroundCalculator->getProgress());
            updatedisplay_bgout();
            updatedisplay_videoin(backgroundCalculator->lastFrame);

        }



        if ( backgroundCalculator->isFinished() )
        {
            cout << "done" << endl;
            isRunning = false;
            ui->stopButton->setChecked(false);
        }

    }
    else{

        return;
    }

}



void Backgrounder::on_loadVideoButton_clicked()
{

    /* select a directory using file dialog */
    videopath = QFileDialog::getOpenFileName (this, tr("Open Video File"),lastpath, tr("Antennate Video (*.avi *.mov *.mpg *.mpeg)"));
    if ( videopath.isNull() == false )
    {
        lastpath=videopath;
        ui->videoFileLabel->setText("..."+videopath.right(50));
        loadVideo();
        qDebug()<<"Load new file";
    }
    else{
        videopath=nopath;
        ui->videoFileLabel->setText(videopath.right(50));
    }
}


void Backgrounder::loadVideo(){

    qDebug()<<"--In load video-";


    //Open all Image Assets and check to see if they are OK before creating a new ICPTRACKER object
    if(checkVideo()){
        qDebug()<<"--Check Video OK!-";
        capture.release();
        capture.open(videopath.toStdString());

        //All items checked out OK continue!
        updatedisplay_videoin(0);
        TOTALVIDEOFRAMES=capture.get(CV_CAP_PROP_FRAME_COUNT)-1;


        ui->startframeSlider->setMaximum(TOTALVIDEOFRAMES);
        ui->endframeSlider->setMaximum(TOTALVIDEOFRAMES);
        ui->inFramespinBox->setMaximum(TOTALVIDEOFRAMES);
        ui->outFramespinBox->setMaximum(TOTALVIDEOFRAMES);
        ui->outFramespinBox->setValue(TOTALVIDEOFRAMES);
        ui->endframeSlider->setValue(TOTALVIDEOFRAMES);


        ui->stopButton->setEnabled(true);
        ui->stopButton->setText("Start");


    }
    else{
        qDebug()<<"--Check Video FALSE-";

    }
}

void Backgrounder::updatedisplay_videoin(int framenum){

    capture.set(CV_CAP_PROP_POS_FRAMES,framenum);
    capture.retrieve(currentFrameViz);
    capture.read(currentFrameViz);

    QImage qimage = QImage((const uchar*)currentFrameViz.data, currentFrameViz.cols, currentFrameViz.rows, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();


    qimage = qimage.scaled(ui->imageLabel->width(), ui->imageLabel->height());
    ui->imageLabel->setPixmap(QPixmap::fromImage(qimage));

    return;
}

void Backgrounder::updatedisplay_bgout(){

    Mat bgupdated;
    (backgroundCalculator->currentBackground).copyTo(bgupdated);
    QImage qimage = QImage((const uchar*)bgupdated.data, bgupdated.cols, bgupdated.rows, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();


    qimage = qimage.scaled(ui->imageLabel_processed->width(), ui->imageLabel_processed->height());
    ui->imageLabel_processed->setPixmap(QPixmap::fromImage(qimage));

    char statusMessage[100];
    sprintf(statusMessage,"frame: %d ",backgroundCalculator->lastFrame);

    statusBar()->showMessage(statusMessage);
    return;


}



bool Backgrounder::checkVideo(){
    bool everythingok=true;

    QString error="ERROR:  ";
    QString colour="red"; // you can use also QColor

    QString fonttemplate = "<font color='%1'>%2</font>";


    QString notprepared="<b>Not Ready!</b>  <br><br><i>Please choose a...</i> <br>";
    if (videopath==nopath){
        notprepared= notprepared+"   Video Source<br>";
        everythingok = false;
        return everythingok;
    }

    if (videopath!=nopath){




        colour="green"; // you can use also QColor
        QString text="<b>Ready to Obtain Background!</b> <br> <br> Press Play Button to Begin";
        fonttemplate = tr("<font color='%1'>%2</font>");
        ui->imageLabel->setText(fonttemplate.arg( colour, text ));


        colour="red";
        //Video Capture
        if(capture.open(videopath.toStdString())){
            return everythingok;
            qDebug()<<"Everything is OK";
        }
        else{

            return false;
            qDebug()<<"Everything is BAD could not capture.open";

        }
    }
}



void Backgrounder::on_startframeSlider_sliderMoved(int position)
{

    if(position>ui->endframeSlider->value()){
        ui->startframeSlider->setValue(ui->endframeSlider->value()-1);

    }

    ui->inFramespinBox->setValue(ui->startframeSlider->value());

    updatedisplay_videoin(ui->startframeSlider->value());
}



void Backgrounder::on_endframeSlider_sliderMoved(int position)
{
    if(position<ui->startframeSlider->value()){
        ui->endframeSlider->setValue(ui->startframeSlider->value()+1);

    }

    ui->outFramespinBox->setValue(ui->endframeSlider->value());
    updatedisplay_videoin(ui->endframeSlider->value());
}






void Backgrounder::on_inFramespinBox_editingFinished()
{
    ui->startframeSlider->setValue(ui->inFramespinBox->value());

    if(ui->startframeSlider->value()  >  ui->endframeSlider->value()){
        ui->startframeSlider->setValue(ui->endframeSlider->value()-1);
        ui->inFramespinBox->setValue(ui->endframeSlider->value()-1);

    }
    updatedisplay_videoin(ui->inFramespinBox->value());

}

void Backgrounder::on_outFramespinBox_editingFinished()
{
    ui->endframeSlider->setValue(ui->outFramespinBox->value());

    if(ui->endframeSlider->value()  <  ui->startframeSlider->value()){
        ui->endframeSlider->setValue(ui->startframeSlider->value()+1);
        ui->outFramespinBox->setValue(ui->startframeSlider->value()+1);

    }
    updatedisplay_videoin(ui->outFramespinBox->value());

}

void Backgrounder::on_stopButton_toggled(bool checked)
{
    if(checked){ //Run the subtractor
        qDebug()<<"Starting backgrounding";
        ui->stopButton->setText("Pause");
        if(backgroundCalculator != NULL)
            delete(backgroundCalculator);

        if(ui->methodcomboBox->currentIndex() == BACKGROUND_METHOD_AVERAGE)
            backgroundCalculator = new BackgroundCalculatorAverage(&capture, ui->startframeSlider->value(), ui->endframeSlider->value());
        if(ui->methodcomboBox->currentIndex()  == BACKGROUND_METHOD_MODE)
            //            qDebug()<<"MODE";
            backgroundCalculator = new BackgroundCalculatorMode(&capture, ui->startframeSlider->value(), ui->endframeSlider->value(), Mode);
        if(ui->methodcomboBox->currentIndex()  == BACKGROUND_METHOD_MEDIAN)
            //              qDebug()<<"MEDIAN";
            backgroundCalculator = new BackgroundCalculatorMode(&capture, ui->startframeSlider->value(), ui->endframeSlider->value(), Median);

        isRunning=true;

        ui->saveBGButton->setEnabled(true);
    }
    if(!checked){//Stop the subtractor
        qDebug()<<"Pause backgrounding";
        ui->stopButton->setText("Start");

    }
}

void Backgrounder::on_saveBGButton_clicked()
{
    QString saveseed = lastpath;
    saveseed.chop(4);
    savelocation = QFileDialog::getSaveFileName(this,tr("Select Background File as..."), saveseed);

    if ( !savelocation.isNull() )
    {
        lastpath=savelocation;
        savelocation=savelocation+".png";
        cv::imwrite(savelocation.toStdString(), backgroundCalculator->currentBackground);


    }
    else{
        savelocation="";
        ui->statusBar->showMessage("Save Cancelled");
    }
}
