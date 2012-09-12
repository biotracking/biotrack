#include "keyframepaint.h"
#include "ui_mainwindow.h"
#include "QtGui"

#include <iostream>
#include <typeinfo>
 #include "ascribblearea.h"
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

zoomsize=360;

    ascribbleArea=ui->paintwidget;



	fps = framerate;
    startTimer(100 / fps);  // timer
	thread_vector = modules;
	pool_vector = pools;
	image_pool=image_buf;

         //ui connections"
        //connect(ui->stopButton,SIGNAL(clicked()),this,SLOT(on_stopButton_clicked()));

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

    if ( id != NULL ) {
        this->produceSharedQImage(*(id->getImage()));
//        qimage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);

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
    qimage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();

        QImage image = qimage.scaledToWidth(ui->imageLabel->width());
    ui->imageLabel->setPixmap(QPixmap::fromImage(image));

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


void MainWindow::on_stopButton_clicked()
{
startThreads(false);
}

void MainWindow::on_playButton_clicked()
{
    startThreads(true);
}

void MainWindow::on_targetpaint_clicked()
{//Set target Pen Mode
    ascribbleArea->targetpen=true;
    ascribbleArea->backgroundpen=false;
    ascribbleArea->backgroundmaskpen=false;
    QColor newColor =Qt::green;
    if (newColor.isValid())
    ascribbleArea->setPenColor(newColor);
    int newWidth=15;
    ascribbleArea->setPenWidth(15);
    std::cout<<"Ontarget penwidth=  "<<ascribbleArea->penWidth()<<std::endl;



}

void MainWindow::on_nottargetpaint_clicked()
{//Set background Pen Mode
    ascribbleArea->targetpen=false;
    ascribbleArea->backgroundpen=true;
    ascribbleArea->backgroundmaskpen=false;

    QColor newColor = (QColor(255,0,0,255));
    ascribbleArea->setPenColor(newColor);
    int newWidth=20;
    ascribbleArea->setPenWidth(newWidth);
    std::cout<<"OnBackground penwidth=  "<<ascribbleArea->penWidth()<<std::endl;



}



void MainWindow::on_backgroundmaskbutton_clicked(){
    std::cout<<"Background Masking: "<<std::endl;
    ascribbleArea->targetpen=false;
    ascribbleArea->backgroundpen=false;
    ascribbleArea->backgroundmaskpen=true;

    QColor newColor = (QColor(255,0,0,255));
    ascribbleArea->setPenColor(newColor);
    ascribbleArea->setBrushColor(newColor);
    int newWidth=3;
    ascribbleArea->setPenWidth(newWidth);
    std::cout<<"OnBackgroundmask penwidth=  "<<ascribbleArea->penWidth()<<std::endl;

}


void MainWindow::on_clearButton_clicked()
{
    ascribbleArea->clearImage();
    std::cout<<"Cleared  "<<std::endl;

}

void MainWindow::on_saveFrameButton_clicked()
{//Save the current Sketched Keyframe
    //Copy over the drawing area
    QImage newImage(ascribbleArea->image.size(), QImage::Format_ARGB32);
    newImage= ascribbleArea->image.copy();

    //Copy over the original video frame
//    QImage videoImage=ui->imageLabel->pixmap()->toImage();
    cv::Mat cvimage=*(image_pool->getRecentFrame()->getImage());
    QImage videoImage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);
    videoImage.convertToFormat(QImage::Format_ARGB32);
//    videoImage=videoImage.rgbSwapped();
    //Make sure they are the same scale
    newImage=newImage.scaled(videoImage.width(),videoImage.height());



    //Convert to graded Alpha values
    int width=newImage.width();
    int height=newImage.height();
QRgb colorvalue;
 QColor qcolor;
 QColor videoColor;
    for(int i=0;i<width;i++)
    {
        for(int j=0;j<height;j++){
            bool pixchanged=false;
            colorvalue = newImage.pixel(i,j);
            qcolor = newImage.pixel(i,j);
            videoColor=videoImage.pixel(i,j);



            //Def Target
            if(qcolor.green()>5&&qcolor.red()<5&&qcolor.blue()<5&&qcolor.alpha()>5&&!pixchanged){
            std::cout<<"SwitchedGreen: "<<colorvalue<<" Pixel i= "<<i<<" Pixel j= "<<j<<std::endl;

            newImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.blue(),videoColor.green(),255));
            pixchanged=true;
}
            //Def Background
            if(qcolor.red()>5&&qcolor.green()<5&&qcolor.blue()<5&&qcolor.alpha()>5){
                            std::cout<<"Switched Back: "<<colorvalue<<" Pixel i= "<<i<<" Pixel j= "<<j<<std::endl;

            newImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.blue(),videoColor.green(),180));

            pixchanged=true;
}

            //noMark
            if(!pixchanged){
                            std::cout<<"Switched Blank: "<<qcolor.alpha()<<" Pixel i= "<<i<<" Pixel j= "<<j<<std::endl;

            newImage.setPixel(i,j,qRgba(videoColor.red(),videoColor.blue(),videoColor.green(),210));
            pixchanged=true;
}
        }
    }




        QPixmap pixmap;

//        pixmap.convertFromImage(ascribbleArea->image);

pixmap.convertFromImage(newImage);

        QString filename= ui->targetIDbox->text()+".png";
        QFile file(filename);
        file.open(QIODevice::WriteOnly);
        pixmap.save(&file,"PNG");
        std::cout<<"Successfully Saved: "<<filename.toStdString()<<std::endl;
        std::cout<<"Dimensions Canvas: h "<<newImage.height()<<" w= "<<newImage.width()<<" Dimensions video h"<<videoImage.height()<<" w"<<videoImage.width()<<std::endl;




}

bool MainWindow::saveFile(const QByteArray &fileFormat)
{
//    QPixmap pixmap()
//    QFile file("adsfdsaf");
//    file.open(QIODevice::WriteOnly);


//    QString initialPath = QDir::currentPath() + "/untitled." + fileFormat;

//    QString fileName = "test.png";
//    if (fileName.isEmpty()) {
//        std::cout<<"Filename is empty"<<std::endl;

//        return false;
//    } else {
//        std::cout<<"About to save in area"<<std::endl;

//        return ascribbleArea->saveImage(fileName, fileFormat);
//    }
}

void MainWindow::on_zoomButton_clicked()
{//Try out magnifying the images and paintbox!
//    ui->imagescrollArea->widgetResizable(true);
//    ui->imagescrollArea->setWidgetResizable(true);
    if(zoomsize!=1080){
    zoomsize=1080;
    ascribbleArea->image.scaled(1920,1080,Qt::IgnoreAspectRatio);

    ui->imageLabel->resize(1920,1080);
    ui->paintwidget->resize(1920,1080);
    ui->scrollAreaWidgetContents->setGeometry(0,0,1920,1080);
//    QSize size = imageLabel->pixmap()->size();
//             size.scale(rect.size(), Qt::KeepAspectRatio);
}
    else if(zoomsize==1080){
    zoomsize=360;
    ascribbleArea->image.scaled(690,360,Qt::IgnoreAspectRatio);

    ui->imageLabel->resize(690,360);
    ui->paintwidget->resize(690,360);
    ui->scrollAreaWidgetContents->setGeometry(0,0,690,360);
}

//    QRect rect= ui->paintwidget->geometry().getRect(0,0,1920,1080);

//    QRect rect= new QRect(0,0,1920,1080);
//    ui->imagescrollArea->contentsRect(rect);


}
