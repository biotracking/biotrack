#include <QtGui/QApplication>

#include "mainwindow.h"
//#include "prosilicacapture.h"
#include "imagecapture.h"
#include "background.h"
//#include "bgsubtract.h"
#include <iostream>
// #include <fstream>
// #include <string>

int main(int argc, char *argv[])
{
  int fps = 30;
  AbosPool capture_pool, bg_pool;

  char *outFolder =  "/home/pipeihuang/ABOS/tools/file2XY/output(video and timestamp)";
  // image capture
  ImageCapture image_capture( fps );
  image_capture.setDeviceFile("/home/pipeihuang/ABOS/tools/file2XY/output(video and timestamp)/out.avi");   // read
  image_capture.setImagePool(&capture_pool);    // write


  //BG averaging
  Background bg(&capture_pool, &bg_pool, outFolder, 0.025,fps);
 
  // configure main window (QT)
  QVector<AbosThread*> thread_vec;
  QVector<AbosPool*> pool_vec;
  thread_vec.append(&bg);
  //thread_vec.append(&bgSub);
  thread_vec.append(&image_capture);


  pool_vec.append(&capture_pool);
  pool_vec.append(&bg_pool);
  //pool_vec.append(&output_pool);
  QApplication a(argc, argv);
  MainWindow w(&bg_pool, thread_vec, pool_vec, fps);

  w.startThreads(true);

  // start each threads
  w.show();

  // run main event loop of QT
  return a.exec();
}
