#include <QtGui/QApplication>
#include <QFile>
#include <string>
#include <iostream>

#include "mainwindow.h"
#include "AbosThread.h"
#include "abospool.h"
#include "imagecapture.h"
#include "ofthread.h"


int main(int argc, char *argv[])
{
    if(argc < 2){
        std::cout << "please specify an input video file." << std::endl;
        exit(0);
    }
	int fps = 30;
	// get ready the data pools
    AbosPool capture_pool, optflow_pool;
    capture_pool.clear();   // to make sure it is empty at first
	optflow_pool.clear();
    // image capture
    ImageCapture image_capture( fps );
    image_capture.setDeviceFile(argv[1]);   // read
    image_capture.setImagePool(&capture_pool);    // write
	// Lucas-Kanade 
	OFThread optflowThread( &capture_pool, &optflow_pool, fps );
	// configure main window (QT)
    QVector<AbosThread*> thread_vec;
	QVector<AbosPool*> pool_vec;
	thread_vec.append(&image_capture);
	thread_vec.append(&optflowThread);
	pool_vec.append(&capture_pool);
	pool_vec.append(&optflow_pool);
    QApplication a(argc, argv);
    MainWindow w(&optflow_pool, thread_vec, pool_vec, fps);
	w.startThreads(true);
    // start each threads
    w.show();
    // run main event loop of QT
    int ret = a.exec();
    // wait until every thread finish
    while ( image_capture.isRunning() || optflowThread.isRunning() ) { }
	printf("Successfully terminated.\n");
    return ret;
}
