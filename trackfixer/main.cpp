#include <QtGui/QApplication>
#include <opencv2/opencv.hpp>

#include <mainwindow.h>

int main(int argc, char** argv){
    QApplication a(argc,argv);
    QStringList args = QCoreApplication::arguments();
    std::string videoFile, btfDir;
    int tmp = args.indexOf("--video");
    double sf = 1.0;
    if(tmp>=0){
        videoFile = args.at(tmp+1).toStdString();
    }
    tmp = args.indexOf("--btf");
    if(tmp>=0){
        btfDir = args.at(tmp+1).toStdString();
    }
    tmp = args.indexOf("--scale");
    if(tmp>=0){
        sf = args.at(tmp+1).toDouble();
    }
    MainWindow mw;
    if(!videoFile.empty()) mw.loadVideo(videoFile);
    if(!btfDir.empty()) mw.loadBTF(btfDir);
    mw.setScaleFactor(sf);
    mw.show();
    return a.exec();
}
