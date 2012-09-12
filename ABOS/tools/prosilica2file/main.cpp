#include <QtGui/QApplication>

#include "mainwindow.h"
#include "prosilicacapture.h"
//#include "imagecapture.h"
#include "background.h"
//#include "bgsubtract.h"
#include <iostream>
// #include <fstream>
// #include <string>

int main(int argc, char *argv[])
{
  int fps = 30;
  AbosPool capture_pool, bg_pool;//, output_pool;

 //CvVideoWriter *writer = 0;
//int isColor = 1;
//int fps     = 30;  // or 30
// int frameW  = 640; // 744 for firewire cameras
// int frameH  = 480; // 480 for firewire cameras

  char *outFolder =  "/home/pipeihuang/ABOS/tools/prosilica2file/output(video and timestamp)";
// string str_outVideo(outFolder);
//  str_outVideo+="/out.avi";
//  writer = cvCreateVideoWriter(str_outVideo.c_str(),CV_FOURCC('P','I','M','1'),
//                            fps,cvSize(frameW,frameH),isColor);
 // string str_outTimeFile(outFolder);
 // str_outTimeFile+="/timestamp.txt";
 // fstream outTimeFile(str_outTimeFile.c_str());

  // image capture
  ProsilicaCapture prosilica_capture( fps );
  //ImageCapture prosilica_capture( fps );
  //image_capture.setDeviceFile("/dev/video0");   // read
//  prosilica_capture.readFromCamera(true);
  prosilica_capture.setImagePool(&capture_pool);    // write

  //BG averaging
  Background bg(&capture_pool, &bg_pool, outFolder, 0.025,fps);
  //BG subtraction and thresholding
 // BGSubtract bgSub(&capture_pool, &bg_pool, &output_pool, 20,fps);
  // configure main window (QT)
  QVector<AbosThread*> thread_vec;
  QVector<AbosPool*> pool_vec;
  thread_vec.append(&bg);
  //thread_vec.append(&bgSub);
  thread_vec.append(&prosilica_capture);


  pool_vec.append(&capture_pool);
  pool_vec.append(&bg_pool);
  //pool_vec.append(&output_pool);
  QApplication a(argc, argv);
  MainWindow w(&bg_pool, thread_vec, pool_vec, fps);

  w.startThreads(true);

  // start each threads
  w.show();

  //cvReleaseVideoWriter(&writer);
  // run main event loop of QT
  return a.exec();
}
