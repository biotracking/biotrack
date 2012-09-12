#include <QtGui/QApplication>
#include <QFile>
#include <QVariant>

#include <string>
#include <iostream>

#include "mainwindow.h"
#include "AbosThread.h"
#include "abospool.h"
#include "imagecapture.h"
#include "prosilicacapture.h"
#include "ofthread.h"
#include "sliders_thread.h"
#include "BlobPairing.h"


int main(int argc, char *argv[])
{

    QApplication a(argc, argv);

    QStringList arguments = QCoreApplication::arguments();

    //qDebug(arguments);

    // TODO use flags and defaults instead of this CLI
    if(arguments.length() < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <input_video_file> <slider_output_file> <Realtime|Offline> [Centroid|CentroidAndSize|Overlap]" << endl;
        exit(0);
    }

    QString device_filename = arguments[1];
    QString sliders_out_filename = arguments[2];
    QString processing_mode_str = arguments[3];
    QString processingMode = processing_mode_str == QString("Offline") ? "Offline" : "Realtime";
    qApp->setProperty("processingMode", QVariant(processingMode));

    BlobPairing::setDistanceMetric(BlobPairing::CentroidDistanceMetric);
    if(arguments.length() > 4)
    {
        if(arguments[4] == QString("Centroid"))
            BlobPairing::setDistanceMetric(BlobPairing::CentroidDistanceMetric);
        else if(arguments[4] == QString("CentroidAndSize"))
            BlobPairing::setDistanceMetric(BlobPairing::CentroidAndSizeDistanceMetric);
        else if(arguments[4] == QString("Overlap"))
            BlobPairing::setDistanceMetric(BlobPairing::OverlapDistanceMetric);
    }

    int fps = 30;
    // prepare the data pools
    AbosPool capture_pool, optflow_pool_flow, optflow_pool_mask, sliders_pool;
    capture_pool.clear();   // to make sure it is empty at first
    optflow_pool_flow.clear();
    optflow_pool_mask.clear();
    sliders_pool.clear();

    // image capture
    ImageCapture image_capture( fps );
    image_capture.setDeviceFile(device_filename.toStdString());   // read
    image_capture.setImagePool(&capture_pool);    // write
    
    ProsilicaCapture image_capture_prosilica( fps );
    image_capture_prosilica.setImagePool(&capture_pool);

    // a shared list to hold output values
    list<Blobs> sliders_vec;

    OFThread optflowThread( &capture_pool, &optflow_pool_flow, &optflow_pool_mask, fps );
    SlidersThread slidersThread(&optflow_pool_flow, &optflow_pool_mask, &sliders_pool, &capture_pool, &sliders_vec, fps);

    QVector<ImageCapture*> capture_vec;
    QVector<AbosThread*> thread_vec;
    QVector<AbosPool*> pool_vec;

    capture_vec.append(&image_capture);
    capture_vec.append(&image_capture_prosilica);

    thread_vec.append(&optflowThread);
    thread_vec.append(&slidersThread);

    pool_vec.append(&capture_pool);
    pool_vec.append(&optflow_pool_flow);
    pool_vec.append(&optflow_pool_mask);
    pool_vec.append(&sliders_pool);

    // configure main window (QT)

    MainWindow w(&sliders_pool, thread_vec, capture_vec, pool_vec, &sliders_vec, fps, sliders_out_filename);
    
    // start each threads
    w.show();
    // run main event loop of QT
    int ret = a.exec();
    // wait until every thread finishes
    while ( image_capture.isRunning()) { }
    std::cerr << "Successfully terminated." << std::endl;
    return ret;
}
