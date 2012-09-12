#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <typeinfo>


MainWindow::MainWindow(	AbosPool *image_buf,
						QVector<AbosThread*> modules,
						QVector<AbosPool*> pools,
						int framerate, QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	fps = framerate;
    startTimer(100 / fps);  // timer
	thread_vector = modules;
	pool_vector = pools;
	image_pool=image_buf;
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
    // read a frame
    //IplImage* img = NULL;
	PoolFrame *id = NULL;
    if( (id = readFrame()) == NULL ) {
		//std::cout << "A gap detected between frames" << std::endl;
        return;
	}

/*  DEBUG - Video output
    static cv::VideoWriter writer;
    if( writer.isOpened() == false ){
        writer.open("result.avi", 0, 60, id->getImage()->size(), true);
    }
*/

    if ( id != NULL ) {
        this->produceSharedQImage(*(id->getImage()));
/*  DEBUG - Video output
        if( writer.isOpened() == true ){
            writer << *(id->getImage());
        }
*/
        //this->produceSharedQImage(id->edge_image);
        //this->produceSharedQImage(id->image);
        /*
		cvReleaseImage(&id->orig_image);
        cvReleaseImage(&id->edge_image);
        cvReleaseImage(&id->image);
		*/
		//id->release();
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
    //share_cvimage = cvCreateImageHeader(cvSize(qimage.width(), qimage.height()), 8, 3);
    //share_cvimage->imageData = (char*)qimage.bits();    // share buffers btw/ qimage and share_cvimage
    //qimage = QImage((const uchar*)cvimage->imageData, cvimage->width, cvimage->height, QImage::Format_RGB888);
	qimage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);
	qimage = qimage.rgbSwapped();
/*
    if(cvimage->origin == IPL_ORIGIN_TL)
        cvCopy(cvimage, share_cvimage, 0);  // don't use the original
    else
        cvFlip(cvimage, share_cvimage, 0);


    // cvimage (which comes from cvQueryFrame) using BGR format while QT image using RGB format
    // format should be convert so that we can keep consistency
    cvCvtColor(share_cvimage, share_cvimage, CV_BGR2RGB);
*/
        QImage image = qimage.scaledToWidth(ui->imageLabel->width());
    ui->imageLabel->setPixmap(QPixmap::fromImage(image));

	//cvReleaseImage (&share_cvimage);
}


void MainWindow::startThreads(bool start) {

    QVectorIterator<AbosThread*> iter(thread_vector);
    while( iter.hasNext() ){
        AbosThread *thread = iter.next();

        if(start){
            thread->start();
         }
        else{
            //FIXME:
            thread->terminate();
        }
    }
}



void MainWindow::on_actionExit_triggered()
{
    this->close();
}

