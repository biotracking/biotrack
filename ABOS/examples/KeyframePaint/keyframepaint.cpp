#include "keyframepaint.h"
#include "ui_mainwindow.h"
#include "QtGui"

#include <iostream>
#include <typeinfo>
 #include "paintcanvas.h"
#include "QButtonGroup"


MainWindow::MainWindow(	AbosPool *image_buf,
						QVector<AbosThread*> modules,
						QVector<AbosPool*> pools,
						int framerate, QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow)
{
   ui->setupUi(this);
   //Setup Button group so buttons visibly toggle
   buttongroup = new QButtonGroup(this);
   buttongroup->addButton(ui->targetpaint);
   buttongroup->addButton(ui->backgroundmaskbutton);
   buttongroup->addButton(ui->nottargetpaint);
   buttongroup->addButton(ui->markHeadbutton);
   buttongroup->addButton(ui->markTailbutton);



   //Shortcuts
   ui->targetpaint->setShortcut(Qt::Key_Q);
   ui->backgroundmaskbutton->setShortcut(Qt::Key_A);
   ui->saveFrameButton->setShortcut(Qt::Key_F2);

   ui->clearButton->setShortcut(Qt::Key_QuoteLeft);

   ui->previousFrameButton->setShortcut(Qt::Key_Left);
   ui->nextFrameButton->setShortcut(Qt::Key_Right);

   ui->playButton->setShortcut(Qt::Key_Space);
   ui->pauseButton->setShortcut(Qt::Key_Space+Qt::CTRL);





   zoomsize=360;

   paintCanvas=ui->paintwidget;
   fps = framerate;
   //startTimer(100 / fps);  // timer
   thread_vector = modules;
   pool_vector = pools;
   image_pool = image_buf;

   ImageCapture* capture = (ImageCapture*)thread_vector.at(0);
   ui->FramespinBox->setMaximum(capture->getFrameCount());
   ui->currentTimeBar->setMaximum(capture->getFrameCount());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event) {
	bool done = false;
	std::cout << "Close" << std::endl;
	QVectorIterator<AbosThread*> thread_iter(thread_vector);
	while(thread_iter.hasNext()){
		AbosThread* thread = thread_iter.next();
		thread->stop();
	}
	QVectorIterator<AbosPool*> pool_iter(pool_vector);
	while(pool_iter.hasNext()){
		AbosPool* pool = pool_iter.next();
		pool->stop();
	}
	std::cout<<"Waiting for threads";
	while(!done){
		done = true;
		QVectorIterator<AbosThread*> thread_iter(thread_vector);
		while(thread_iter.hasNext()){
			AbosThread* thread = thread_iter.next();
			done = (done && thread->wait(1000));
		}
		QVectorIterator<AbosPool*> pool_iter(pool_vector);
		while(pool_iter.hasNext()){
			AbosPool* pool = pool_iter.next();
			done = (done && pool->wait(1000));
		}
		std::cout<<".";
		std::cout.flush();
	}
	std::cout<<"done."<<std::endl;
	QMainWindow::closeEvent(event);

}

void MainWindow::timerEvent(QTimerEvent*){
    updateUI();
}

void MainWindow::updateUI() {
    // read a frame
   PoolFrame *id = NULL;
   if( (id = readFrame()) == NULL ) {
        //std::cout << "A gap detected between frames" << std::endl;
      return;
   }

   if ( id != NULL ) {
      this->produceSharedQImage(*(id->getImage()));

      ui->currentTimeBar->setValue(id->getFrameNumber());
      ui->FramespinBox->setValue(id->getFrameNumber());

      image_pool->releaseFrame(id);
    }
}

/**
  read a frame from capture or from image_pool.
  if we are using the threaded model,
  it should read from the image_pool which is global beyond threads.
  */
PoolFrame* MainWindow::readFrame(){
    if(image_pool->getSize() == 0)
        return NULL;

    // read the oldest frame from the buffer and erase from it
    PoolFrame *frame = image_pool->getRecentFrame();

    return frame;
}

/**
  read a frame from capture or from image_pool.
  if we are using the threaded model,
  it should read from the image_pool which is global beyond threads.
  */
const cv::Mat* MainWindow::readIDImage(){
    if(image_pool->getSize() == 0)
        return NULL;

    // read from bgsub threadfrom the buffer and erase from it
        const cv::Mat* tmpImage = image_pool->getRecentFrame()->getImage();

    return tmpImage;
}

void MainWindow::produceSharedQImage(cv::Mat cvimage){
   qimage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);
   qimage = qimage.rgbSwapped();

        //QImage image = qimage.scaledToWidth(ui->imageLabel->width());
   QImage image = qimage.scaled(ui->imageLabel->width(), ui->imageLabel->height());
   ui->imageLabel->setPixmap(QPixmap::fromImage(image));
}


void MainWindow::startThreads(bool start) {

    QVectorIterator<AbosThread*> iter(thread_vector);
    while( iter.hasNext() ){
        AbosThread *thread = iter.next();

        if(start){
            thread->start();
            timer_id = startTimer(100 / fps);
        }
        else{
            thread->stop();
            if (timer_id != 0) {
                killTimer(timer_id);
                timer_id = 0;
            }
        }
    }
}

void MainWindow::on_playButton_clicked()
{
   ui->FramespinBox->setEnabled(false);
   ui->nextFrameButton->setEnabled(false);
   ui->previousFrameButton->setEnabled(false);
   ui->currentTimeBar->setEnabled(false);
   startThreads(true);
}

void MainWindow::on_pauseButton_clicked()
{
    ImageCapture* capture = (ImageCapture*)thread_vector.at(0);
    capture->pause();
    if (timer_id != 0) {
        killTimer(timer_id);
        timer_id = 0;
    }
    ui->FramespinBox->setEnabled(true);
    ui->nextFrameButton->setEnabled(true);
    ui->previousFrameButton->setEnabled(true);
    ui->currentTimeBar->setEnabled(true);
}

void MainWindow::on_stopButton_clicked()
{
   startThreads(false);
   updateUI();
   ui->FramespinBox->setEnabled(true);
   ui->nextFrameButton->setEnabled(true);
   ui->previousFrameButton->setEnabled(true);
   ui->currentTimeBar->setEnabled(true);
}

void MainWindow::on_FramespinBox_editingFinished()
{
   ImageCapture* capture = (ImageCapture*)thread_vector.at(0);
   capture->goToFrame(ui->FramespinBox->value());
   updateUI();
}

void MainWindow::on_nextFrameButton_clicked()
{
    ImageCapture* capture = (ImageCapture*)thread_vector.at(0);
    capture->goToFrame(ui->FramespinBox->value() + 1);
    updateUI();
}

void MainWindow::on_previousFrameButton_clicked()
{
    ImageCapture* capture = (ImageCapture*)thread_vector.at(0);
    capture->goToFrame(ui->FramespinBox->value() - 1);
    updateUI();
}

void MainWindow::on_currentTimeBar_sliderMoved(int position)
{
    ImageCapture* capture = (ImageCapture*)thread_vector.at(0);
    capture->goToFrame(position);
    updateUI();
}



void MainWindow::on_targetpaint_clicked()
{//Set target Pen Mode
    paintCanvas->targetpen=true;
    paintCanvas->backgroundpen=false;
    paintCanvas->backgroundmaskpen=false;
    paintCanvas->markHead=false;
    paintCanvas->markTail=false;
    QColor newColor =QColor(0,255,0,50);

    if (newColor.isValid())
    paintCanvas->setPenColor(newColor);
    paintCanvas->setPenWidth(8);
    std::cout<<"Ontarget penwidth=  "<<paintCanvas->penWidth()<<std::endl;
    ui->statusBar->showMessage(QString::fromStdString("Paint regions with definite target green"),0);

}

void MainWindow::on_nottargetpaint_clicked()
{//Set background Pen Mode
    paintCanvas->targetpen=false;
    paintCanvas->backgroundpen=true;
    paintCanvas->backgroundmaskpen=false;
    paintCanvas->markHead=false;
    paintCanvas->markTail=false;

    QColor newColor = (QColor(255,0,0,255));
    paintCanvas->setPenColor(newColor);
    int newWidth=10;
    paintCanvas->setPenWidth(newWidth);
    std::cout<<"OnBackground penwidth=  "<<paintCanvas->penWidth()<<std::endl;
    ui->statusBar->showMessage(QString::fromStdString("Paint areas without target red"),0);




}



void MainWindow::on_backgroundmaskbutton_clicked(){
    std::cout<<"Background Masking: "<<std::endl;
    paintCanvas->targetpen=false;
    paintCanvas->backgroundpen=false;
    paintCanvas->backgroundmaskpen=true;
    paintCanvas->markHead=false;
    paintCanvas->markTail=false;

    QColor newColor = (QColor(255,0,0,255));
    paintCanvas->setPenColor(newColor);
    paintCanvas->setBrushColor(newColor);
    int newWidth=3;
    paintCanvas->setPenWidth(newWidth);
    std::cout<<"OnBackgroundmask penwidth=  "<<paintCanvas->penWidth()<<std::endl;
    ui->statusBar->showMessage(QString::fromStdString("Left click to add mask points | Right click to close masking shape"),0);


}

void MainWindow::on_markHeadbutton_clicked()
{
    //mark the location of the target's head

    paintCanvas->targetpen=false;
    paintCanvas->backgroundpen=false;
    paintCanvas->backgroundmaskpen=false;
    paintCanvas->markHead=true;
    paintCanvas->markTail=false;

    QColor newColor = (QColor(0,0,255,255));//Heads read as blue 255
    paintCanvas->setPenColor(newColor);
    int newWidth=2;
    paintCanvas->setPenWidth(newWidth);
    std::cout<<"HEAD MARKING penwidth=  "<<paintCanvas->penWidth()<<std::endl;
    ui->statusBar->showMessage(QString::fromStdString("Click once to mark Head location"),0);

}

void MainWindow::on_markTailbutton_clicked()
{
    //mark the location of the target's tail

    paintCanvas->targetpen=false;
    paintCanvas->backgroundpen=false;
    paintCanvas->backgroundmaskpen=false;
    paintCanvas->markHead=false;
    paintCanvas->markTail=true;

    QColor newColor = (QColor(253,0,253,255));//Heads read as blue 253 and red 253
    paintCanvas->setPenColor(newColor);
    int newWidth=2;
    paintCanvas->setPenWidth(newWidth);
    std::cout<<"TAIL MARKING penwidth=  "<<paintCanvas->penWidth()<<std::endl;
    ui->statusBar->showMessage(QString::fromStdString("Click once to mark Tail location"),0);


}





void MainWindow::on_clearButton_clicked()
{
    paintCanvas->clearImage();
    std::cout<<"Cleared  "<<std::endl;
    ui->statusBar->showMessage(QString::fromStdString("-Painting Cleared-"),0);


}

void MainWindow::on_saveFrameButton_clicked()
{//Save the current Sketched Keyframe
    //Copy over the drawing area
    QImage canvasImage(paintCanvas->image.size(), QImage::Format_ARGB32);
    canvasImage= paintCanvas->image.copy();

    //Copy over the original video frame
    cv::Mat cvimage=*(image_pool->getRecentFrame()->getImage());
    QImage videoImage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);
    videoImage=videoImage.rgbSwapped();
    //Make sure they are the same scale
    canvasImage=canvasImage.scaled(videoImage.width(),videoImage.height());



    //Convert to graded Alpha values
    int width=canvasImage.width();
    int height=canvasImage.height();
QRgb colorvalue;
 QColor canvasColor;
 QColor videoColor;
    for(int i=0;i<width;i++)
    {
        for(int j=0;j<height;j++){
            bool pixchanged=false;
            colorvalue = canvasImage.pixel(i,j);
            canvasColor = canvasImage.pixel(i,j);
            videoColor=videoImage.pixel(i,j);



            //Def Target
            if(canvasColor.green()>5&&canvasColor.red()<5&&canvasColor.blue()<5&&canvasColor.alpha()>5&&!pixchanged){
            //std::cout<<"SwitchedGreen: "<<colorvalue<<" Pixel i= "<<i<<" Pixel j= "<<j<<std::endl;

            canvasImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.green(),videoColor.blue(),255));//Target Dots are saved as 255
            pixchanged=true;
}

            //Head mark //Heads are marked as Blue 255
            if(canvasColor.green()<5&&canvasColor.red()<5&&canvasColor.blue()>5&&canvasColor.alpha()>5&&!pixchanged){
            canvasImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.green(),videoColor.blue(),254));//Head dots are saved as 254
            pixchanged=true;
            std::cout<<"Changed a head "<<std::endl;


}

            //Tail mark //Tails are marked as blue 253 Red 253
            if(canvasColor.green()<5&&canvasColor.red()>5&&canvasColor.blue()>5&&canvasColor.alpha()>5&&!pixchanged){

            canvasImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.green(),videoColor.blue(),253));//Target Dots are saved as 253
            pixchanged=true;
            std::cout<<"Changed a tail "<<std::endl;



}


            //Def Background
            if(canvasColor.red()>5&&canvasColor.green()<5&&canvasColor.blue()<5&&canvasColor.alpha()>5){
                            //std::cout<<"Switched Back: "<<colorvalue<<" Pixel i= "<<i<<" Pixel j= "<<j<<std::endl;

            canvasImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.green(),videoColor.blue(),30));

            pixchanged=true;
}

            //noMark
            if(!pixchanged){
                            //std::cout<<"Switched Blank: "<<qcolor.alpha()<<" Pixel i= "<<i<<" Pixel j= "<<j<<std::endl;

            canvasImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.green(),videoColor.blue(),140));
            pixchanged=true;
}
        }
    }


        QString filename = ui->targetIDbox->text()+"_"+ui->FramespinBox->text()+".png";
        QFile file(filename);
        file.open(QIODevice::WriteOnly);
        canvasImage.save(&file,"PNG");
        std::cout<<"Successfully Saved: "<<filename.toStdString()<<std::endl;
        std::cout<<"Dimensions Canvas: h "<<canvasImage.height()<<" w= "<<canvasImage.width()<<" Dimensions video h"<<videoImage.height()<<" w"<<videoImage.width()<<std::endl;
        ui->statusBar->showMessage(QString::fromStdString("Successfully Saved: "+filename.toStdString()),0 );

}

void MainWindow::on_zoomButton_clicked()
{//Try out magnifying the images and paintbox!
//    ui->imagescrollArea->widgetResizable(true);
//    ui->imagescrollArea->setWidgetResizable(true);
    if(zoomsize!=1080){
    zoomsize=1080;
    paintCanvas->resize(1920,1080);

   paintCanvas->image.scaled(1920,1080,Qt::IgnoreAspectRatio);

   paintCanvas->resizeImage(&paintCanvas->image, QSize(1920, 1080));


    ui->imageLabel->resize(1920,1080);
    ui->scrollAreaWidgetContents->setGeometry(0,0,1920,1080);
    ui->statusBar->showMessage(QString::fromStdString("Zoomed In"),0);

//    QSize size = imageLabel->pixmap()->size();
//             size.scale(rect.size(), Qt::KeepAspectRatio);
   }
    else if(zoomsize==1080){
    zoomsize=360;
    paintCanvas->resize(690,360);

paintCanvas->image.scaled(690,360,Qt::IgnoreAspectRatio);

paintCanvas->resizeImage(&paintCanvas->image, QSize(690, 360));

    ui->imageLabel->resize(690,360);
    ui->scrollAreaWidgetContents->setGeometry(0,0,690,360);
    ui->statusBar->showMessage(QString::fromStdString("Zoomed Out"),0);

   }
    updateUI();

}

void MainWindow::on_pensizespinBox_valueChanged(int arg1)
{
    paintCanvas->setPenWidth(arg1);
        std::ostringstream o;
        o<<"New Pen Size = "<<arg1;

    ui->statusBar->showMessage(QString::fromStdString(o.str()),0);


}

