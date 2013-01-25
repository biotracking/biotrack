#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <fstream>
#include <typeinfo>
#include <algorithm>

void TrackletRectItem::mousePressEvent(QGraphicsSceneMouseEvent *event){
    //std::cout<<"Pressed!"<<std::endl;
    if(event != NULL) event = event; //get rid of that stupid squiggly
    QBrush tmp = this->brush();
    if(selected){
        tmp.setColor(color);
        this->setBrush(tmp);
        selected = false;
    } else {
        tmp.setColor(color.lighter(150));
        this->setBrush(tmp);
        selected = true;
    }
}


MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->statusBar->setSizeGripEnabled(true);
	fps = 30.0;
    timerID=-1;
    paused = true;
    timestamp_idx = x_idx = y_idx = t_idx = id_idx = -1;
    scaleFactor = 1.0;
    frame_count=0;
    ui->imageLabel->setScaledContents(true);
    ui->graphicsView->setScene(&scene);
    curFrameLine = NULL;
    frame_tracklets.clear();
}


MainWindow::~MainWindow()
{
    clearBTFData();
    delete ui;
}

void MainWindow::clearBTFData(){

    //for(unsigned int i=0; i<btf_data.size();i++){
    //	btf_data[i].clear();
    //}
    //btf_data.clear();
    for(unsigned int i=0;i<frame_tracklets.size();i++){
        frame_tracklets.at(i).clear();
    }
    frame_tracklets.clear();
    //for(unsigned int i=0;i<frame_data.size();i++){
    //	frame_data[i].clear();
    //}
    //frame_data.clear();
    //flip_data.clear();
	btf_names.clear();
    for(int i=0; i<ui->graphicsView->scene()->items().count();i++){
        ui->graphicsView->scene()->removeItem(ui->graphicsView->scene()->items().at(i));
    }
    for(unsigned int i=0;i<tracklets.size();i++){
        TrackletRectItem* tmp= tracklets.at(i);
        delete tmp;
    }
    tracklets.clear();
}

void MainWindow::loadVideo(std::string fname){
    cap.open(fname);
    fps = cap.get(CV_CAP_PROP_FPS);
    std::cout<<"Video FPS:"<<fps<<std::endl;
    frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);
    std::cout<<"Frames:"<<frame_count<<std::endl;
    cv::Mat in,out;
    paused = true;
    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setMaximum(frame_count);
    ui->horizontalSlider->setValue(0);
    cap.read(in);
    out = in.clone();
//    cv::cvtColor(in,out,CV_BGR2RGB);
    QImage img = Mat2QImage(out);
    ui->imageLabel->setPixmap(QPixmap::fromImage(img));
}

void MainWindow::loadBTF(std::string dname){
	clearBTFData();
    std::cout<<"Parsing BTF from ["<<dname<<"]"<<std::endl;
    QStringList filters, files;
    filters<<"*.btf";
    QDir dir(dname.c_str());
    files = dir.entryList(filters);
    std::vector<std::vector<std::string> > btf_data;
    std::vector<std::vector<int> > frame_data;
    //for(unsigned int i=0;i<btf_data.size();i++){
    //    btf_data.at(i).clear();
    //}
    //btf_data.clear();
    btf_names.clear();
    for(int i=0;i<files.size();i++){
        btf_names.push_back(files.at(i).toStdString());
        if(files.at(i).toStdString().compare("timestamp.btf")==0){
            timestamp_idx = i;
        } else if (files.at(i).toStdString().compare("id.btf")==0){
            id_idx = i;
        } else if (files.at(i).toStdString().compare("ximage.btf")==0){
            x_idx = i;
        } else if (files.at(i).toStdString().compare("yimage.btf")==0){
            y_idx = i;
        } else if (files.at(i).toStdString().compare("timage.btf")==0){
            t_idx = i;
        }
        std::cout<<"\tLoading ["<<dname+"/"+(files.at(i).toStdString())<<"]"<<std::endl;
        std::ifstream inf;
        inf.open((dname+"/"+(files.at(i).toStdString())).c_str());
        std::vector<std::string> fdata;
        while(inf.good()){
            char buff[512];
            inf>>buff;
            fdata.push_back(std::string(buff));
        }
        inf.close();
        btf_data.push_back(fdata);
        std::cout<<"\t"<<fdata.size()<<" lines loaded"<<std::endl;
    }
    std::string lastTS = "33";
    std::vector<int> linenos;
    for(unsigned int i=0;i<btf_data.at(timestamp_idx).size();i++){
        if(btf_data.at(timestamp_idx).at(i).compare(lastTS)!=0){
            frame_data.push_back(linenos);
            linenos.clear();
            lastTS=btf_data.at(timestamp_idx).at(i);
        }
        linenos.push_back(i);
    }
    std::cout<<"BTF data for ["<<frame_data.size()<<"] frames"<<std::endl;
    //load up all the ID's through the video
    std::vector<std::pair<std::string,int> > idsAndStarts, scratch;
    double maxID = 0.0;
    for(unsigned int i=0;i<frame_data.size();i++){
        scratch.clear();
        bool *marked = new bool[idsAndStarts.size()];
        for(unsigned int flarfle = 0;flarfle<idsAndStarts.size();flarfle++) marked[flarfle]=false;
        for(unsigned int j=0;j<frame_data.at(i).size();j++){
            bool found = false;
            for(unsigned int k=0;k<idsAndStarts.size();k++){
                if(idsAndStarts.at(k).first.compare(btf_data.at(id_idx).at(frame_data.at(i).at(j))) == 0){
                    marked[k] = true;
                    found = true;
                    break;
                }
            }
            if(!found){
                std::pair<std::string,int> tmpPair;
                tmpPair.first = btf_data.at(id_idx).at(frame_data.at(i).at(j)); //ant ID
                tmpPair.second = i; //frame number
                scratch.push_back(tmpPair);
            }
        }
        for(unsigned int k=0;k<idsAndStarts.size();k++){
            if(marked[k]){
                scratch.push_back(idsAndStarts.at(k));
            } else {
                int x,y,width,height;
                x = idsAndStarts.at(k).second;
                y = atoi(idsAndStarts.at(k).first.c_str())*20;
                width = i-x;
                height = 19;
                if(y+height > maxID) maxID = y+height;
                TrackletRectItem *tr = new TrackletRectItem();
                tr->ximage.clear();
                tr->yimage.clear();
                tr->timage.clear();
                tr->setRect(x,y,width,height);
                tr->startFrame = idsAndStarts.at(k).second;
                tr->endFrame = i;
                tr->color = Qt::yellow;
                tr->flipped = tr->nuked = tr->selected = false;
                tr->antID = atoi(idsAndStarts.at(k).first.c_str());
                tr->setBrush(QBrush(Qt::yellow));
                tracklets.push_back(tr);
                ui->graphicsView->scene()->addItem(tr);
            }
        }
        delete [] marked;
        idsAndStarts = scratch;
    }
    for(unsigned int i=0;i<idsAndStarts.size();i++){
        int x,y,width,height;
        x = idsAndStarts.at(i).second;
        y = atoi(idsAndStarts.at(i).first.c_str())*20;
        width = frame_data.size()-x;
        height = 19;
        if(y+height > maxID) maxID = y+height;
        TrackletRectItem *tr = new TrackletRectItem();
        tr->ximage.clear();
        tr->yimage.clear();
        tr->timage.clear();
        tr->setRect(x,y,width,height);
        tr->startFrame = idsAndStarts.at(i).second;
        tr->endFrame = frame_data.size();
        tr->color = Qt::yellow;
        tr->flipped = tr->nuked = tr->selected = false;
        tr->antID = atoi(idsAndStarts.at(i).first.c_str());
        tr->setBrush(QBrush(Qt::yellow));
        tracklets.push_back(tr);
        ui->graphicsView->scene()->addItem(tr);
    }
    curFrameLine = ui->graphicsView->scene()->addLine(0,0,0,maxID,QPen(Qt::red));
    for(int i=0;i<=(maxID-19)/20;i++){
        std::stringstream thisisdumb;
        thisisdumb<<i;
        ui->graphicsView->scene()->addText(thisisdumb.str().c_str())->setPos(0,i*20);
    }
    curFrameLine->setZValue(1.0);
    std::vector<TrackletRectItem*> trackletsInFrame;
    for(unsigned int i=0;i<frame_data.size();i++){
        trackletsInFrame.clear();
        for(unsigned int j=0;j<frame_data.at(i).size();j++){
            int curID = atoi(btf_data.at(id_idx).at(frame_data.at(i).at(j)).c_str());
            for(unsigned int k=0;k<tracklets.size();k++){
                if(tracklets.at(k)->antID == curID && tracklets.at(k)->startFrame <= (int)i && tracklets.at(k)->endFrame > (int)i){
                    tracklets.at(k)->ximage.push_back(atof(btf_data.at(x_idx).at(frame_data.at(i).at(j)).c_str()));
                    tracklets.at(k)->yimage.push_back(atof(btf_data.at(y_idx).at(frame_data.at(i).at(j)).c_str()));
                    tracklets.at(k)->timage.push_back(atof(btf_data.at(t_idx).at(frame_data.at(i).at(j)).c_str()));
                    tracklets.at(k)->timestamp.push_back(atof(btf_data.at(timestamp_idx).at(frame_data.at(i).at(j)).c_str()));
                    trackletsInFrame.push_back(tracklets.at(k));
                }
            }
        }
        frame_tracklets.push_back(trackletsInFrame);
    }

}

void MainWindow::closeEvent(QCloseEvent *event) {
    //loadedVideo.clear();
    //cap.close();
	std::cout << "Close" << std::endl;
	QMainWindow::closeEvent(event);

}

QImage MainWindow::Mat2QImage(const cv::Mat3b &src) {
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const cv::Vec3b *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        destrow[x] = qRgba(srcrow[x][2], srcrow[x][1], srcrow[x][0], 255);
                }
        }
        return dest;
}

void MainWindow::timerEvent(QTimerEvent*){
    int curFrame = ui->horizontalSlider->value();

    if(paused) return;
    if(curFrame+1 >= frame_count){
        paused = true;
    } else {
        ui->horizontalSlider->setValue(curFrame+1);
    }
}


void MainWindow::on_actionExit_triggered()
{
    this->close();
}


void MainWindow::on_pushButton_4_clicked()
{
    if(!paused){
        if(timerID!=-1){
            this->killTimer(timerID);
        }
        paused = true;
    } else {
        timerID = this->startTimer((int)(1000.0/fps));
        paused = false;
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    ui->horizontalSlider->setValue(0);
}

void MainWindow::on_pushButton_clicked()
{
    ui->horizontalSlider->setValue(frame_count-1);
}

void MainWindow::on_pushButton_2_clicked()
{
    if(ui->horizontalSlider->value() > 0)
        ui->horizontalSlider->setValue(ui->horizontalSlider->value()-1);
}

void MainWindow::on_pushButton_5_clicked()
{
    if(ui->horizontalSlider->value()<frame_count-1)
        ui->horizontalSlider->setValue(ui->horizontalSlider->value()+1);
}

void MainWindow::on_actionLoad_video_file_triggered()
{
    ui->statusBar->showMessage("Loading video...");
    QString vidName = QFileDialog::getOpenFileName(this);
    if(vidName.isNull()){
        ui->statusBar->showMessage("Cancled loading video");
        return;
    } else {
        loadVideo(vidName.toStdString());
        ui->statusBar->showMessage("Done!");
    }
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    if(value > frame_count-1 || value < 0) return;
    cv::Mat in,out;
    int curFrame = value;
    std::stringstream thisisdumb;
    thisisdumb<<"Frame "<<curFrame;
    ui->statusBar->showMessage(QString(thisisdumb.str().c_str()));
    
    cap.set(CV_CAP_PROP_POS_FRAMES,curFrame);
    cap.read(in);
    
    out = in.clone();
    cv::cvtColor(in,out,CV_BGR2RGB);
    if(value<((int)frame_tracklets.size())){
        for(unsigned int j=0;j<frame_tracklets.at(value).size();j++){
            if(frame_tracklets.at(value).at(j)->nuked) continue;
            thisisdumb.str("");
            thisisdumb.clear();
            thisisdumb<<frame_tracklets.at(value).at(j)->antID;
            std::string trackId = thisisdumb.str();
            int trackletStart = frame_tracklets.at(value).at(j)->startFrame;
            double scaledX = (scaleFactor*frame_tracklets.at(value).at(j)->ximage.at(value-trackletStart));
            double scaledY = (scaleFactor*frame_tracklets.at(value).at(j)->yimage.at(value-trackletStart));
            bool flip_coeff = frame_tracklets.at(value).at(j)->flipped;
            double theta = ((flip_coeff)?M_PI:0)+frame_tracklets.at(value).at(j)->timage.at(value-trackletStart);
            int radius = 10;
            int lineWidth = 2;
            cv::Scalar btfColor(255,0,0);
            cv::circle(out,cv::Point(scaledX,scaledY),radius,btfColor,lineWidth);
            double endPtX = (radius*cos(theta))+scaledX;
            double endPtY = (radius*sin(theta))+scaledY;
            cv::line(out,cv::Point(scaledX,scaledY),cv::Point(endPtX,endPtY),btfColor,lineWidth);
            cv::putText(out,trackId,cv::Point(scaledX,scaledY),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(0,0,255),2);
        }
    }
    QImage img = Mat2QImage(out);
    ui->imageLabel->setPixmap(QPixmap::fromImage(img).scaledToWidth((int)(out.cols*ui->doubleSpinBox->value())));
    if(curFrameLine != NULL) curFrameLine->setPos(value,0);
}

void MainWindow::on_actionLoad_BTF_triggered()
{
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::Directory);
    dialog.setOption(QFileDialog::ShowDirsOnly,true);
    if(dialog.exec()){
        loadBTF(dialog.selectedFiles().at(0).toStdString());
    }

}

void MainWindow::on_pushButton_6_clicked(){
    for(unsigned int i=0;i<tracklets.size();i++){
        if(tracklets.at(i)->selected){
            tracklets.at(i)->flipped = !(tracklets.at(i)->flipped);
            QBrush tmp = tracklets.at(i)->brush();
            if(tracklets.at(i)->flipped){
                tracklets.at(i)->color = Qt::green;
            } else {
                tracklets.at(i)->color = Qt::yellow;
            }
            tmp.setColor(tracklets.at(i)->color.lighter(150));
            tracklets.at(i)->setBrush(tmp);
        }
    }
}

void MainWindow::on_pushButton_7_clicked()
{
    //QString file_name = QFileDialog::getSaveFileName(this,"Save BTF as","timage.btf");
    QString dir_name = QFileDialog::getExistingDirectory(this,"BTF Save Directory");
    dir_name = dir_name+"/";
    std::cout<<"Writting BTF to ["<<dir_name.toStdString()<<"]"<<std::endl;
    std::cout<<"Ignored tracks: [ ";
    for(unsigned int i=0;i<tracklets.size();i++){
        if(tracklets.at(i)->nuked){
            std::cout<<tracklets.at(i)->antID<<"@("<<tracklets.at(i)->startFrame<<","<<tracklets.at(i)->endFrame<<") ";
        }
    }
    std::cout<<"]"<<std::endl;
    std::cout<<"Track flips: [ ";
    for(unsigned int i=0;i<tracklets.size();i++){
        if(tracklets.at(i)->flipped){
            std::cout<<tracklets.at(i)->antID<<"@("<<tracklets.at(i)->startFrame<<","<<tracklets.at(i)->endFrame<<") ";
        }
    }
    std::cout<<"]"<<std::endl;
    std::vector<std::fstream* > files;
    for(unsigned int i=0;i<btf_names.size();i++){
        std::fstream *a_file = new std::fstream();
        a_file->open((dir_name+btf_names.at(i).c_str()).toStdString().c_str(),std::fstream::out);
        files.push_back(a_file);
    }
    for(unsigned int i=0;i<frame_tracklets.size();i++){
        for(unsigned int j=0;j<frame_tracklets.at(i).size();j++){
            //if nuked, ignore this tracklet
            if(frame_tracklets.at(i).at(j)->nuked)
                continue;
            //otherwise, write out the x and y
            int trackletStart = frame_tracklets.at(i).at(j)->startFrame;
            (*(files.at(x_idx)))<<frame_tracklets.at(i).at(j)->ximage.at(i-trackletStart)<<std::endl;
            (*(files.at(y_idx)))<<frame_tracklets.at(i).at(j)->yimage.at(i-trackletStart)<<std::endl;
            //flip theta?
            (*(files.at(t_idx)))<<((frame_tracklets.at(i).at(j)->flipped)?M_PI:0)+frame_tracklets.at(i).at(j)->timage.at(i-trackletStart)<<std::endl;
            (*(files.at(timestamp_idx)))<<frame_tracklets.at(i).at(j)->timestamp.at(i-trackletStart)<<std::endl;
            (*(files.at(id_idx)))<<frame_tracklets.at(i).at(j)->antID<<std::endl;
        }
    }
    for(unsigned int i=0;i<files.size();i++){
        files.at(i)->close();
        delete files.at(i);
    }
    std::cout<<"Done!"<<std::endl;
}

void MainWindow::on_pushButton_8_clicked()
{
    for(unsigned int i=0;i<tracklets.size();i++){
        if(tracklets.at(i)->selected){
            tracklets.at(i)->nuked = !(tracklets.at(i)->nuked);
            QBrush tmp = tracklets.at(i)->brush();
            if(tracklets.at(i)->nuked){
                tmp.setStyle(Qt::DiagCrossPattern);
            } else {
                tmp.setStyle(Qt::SolidPattern);
            }
            tracklets.at(i)->setBrush(tmp);
        }
    }
}

void MainWindow::on_pushButton_9_clicked()
{
    //split button
    int currentFrame = ui->horizontalSlider->value();
    if(currentFrame >=0 && currentFrame < (int)frame_tracklets.size()){
        for(unsigned int i=0;i<tracklets.size();i++){
            if(tracklets.at(i)->selected){
                int x,y,width,height;
                x = currentFrame;
                y = tracklets.at(i)->antID*20;
                width = (tracklets.at(i)->endFrame)-currentFrame;
                height = 19;
                TrackletRectItem *tr = new TrackletRectItem();
                tr->ximage.clear();
                tr->yimage.clear();
                tr->timage.clear();
                tr->setRect(x,y,width,height);
                tr->startFrame = currentFrame;
                tr->endFrame = tracklets.at(i)->endFrame;
                tr->color = Qt::yellow;
                tr->flipped = tr->nuked = tr->selected = false;
                tr->antID = tracklets.at(i)->antID;
                //find the split point in the relative tracks
                int newEnd = currentFrame-tracklets.at(i)->startFrame;
                //remove tracklets.at(i) from frame_tracklets at all
                //the frames between currentFrame and endFrame
                //and add tr to frame_tracklets at all the frames
                //between currentFrame and endFrame
                for(int j=currentFrame;j<tr->endFrame;j++){
                    for(unsigned int k=0;k<frame_tracklets.at(j).size();k++){
                        if(frame_tracklets.at(j).at(k)->antID == tr->antID){
                            //std::cout<<"Removing Ant "<<tr->antID<<" from frame "<<j<<std::endl;
                            frame_tracklets.at(j).erase(frame_tracklets.at(j).begin()+k);
                        }
                    }
                    frame_tracklets.at(j).push_back(tr);
                }
                //now add the track data to the new tracklet
                //X pos
                tr->ximage.reserve(width);
                for(int j=newEnd;j<(int)tracklets.at(i)->ximage.size();j++){
                    tr->ximage.push_back(tracklets.at(i)->ximage.at(j));
                }
                //Y pos
                tr->yimage.reserve(width);
                for(int j=newEnd;j<(int)tracklets.at(i)->yimage.size();j++){
                    tr->yimage.push_back(tracklets.at(i)->yimage.at(j));
                }
                //Theta
                tr->timage.reserve(width);
                for(int j=newEnd;j<(int)tracklets.at(i)->timage.size();j++){
                    tr->timage.push_back(tracklets.at(i)->timage.at(j));
                }
                //Timestamp
                tr->timestamp.reserve(width);
                for(int j=newEnd;j<(int)tracklets.at(i)->timestamp.size();j++){
                    tr->timestamp.push_back(tracklets.at(i)->timestamp.at(j));
                }
                tr->setBrush(QBrush(Qt::yellow));
                tracklets.push_back(tr);
                x = tracklets.at(i)->startFrame;
                y = tracklets.at(i)->antID*20;
                width = currentFrame - tracklets.at(i)->startFrame;
                height = 19;
                tracklets.at(i)->endFrame = currentFrame;
                //now remove the split data from the old tracklet
                tracklets.at(i)->ximage.erase(tracklets.at(i)->ximage.begin()+(newEnd),
                                              tracklets.at(i)->ximage.end());
                tracklets.at(i)->yimage.erase(tracklets.at(i)->yimage.begin()+(newEnd),
                                              tracklets.at(i)->yimage.end());
                tracklets.at(i)->timage.erase(tracklets.at(i)->timage.begin()+(newEnd),
                                              tracklets.at(i)->timage.end());
                tracklets.at(i)->timestamp.erase(tracklets.at(i)->timestamp.begin()+(newEnd),
                                              tracklets.at(i)->timestamp.end());
                                             /* */
                tracklets.at(i)->setRect(x,y,width,height);
                ui->graphicsView->scene()->addItem(tr);
            }
        }
    }
}

void MainWindow::on_pushButton_10_clicked()
{
    int currentFrame = ui->horizontalSlider->value();
    if(currentFrame >= 0 && currentFrame < (int)frame_tracklets.size()){
        int maxID = -1;
        for(unsigned int i=0;i<tracklets.size();i++){
            if(maxID < tracklets.at(i)->antID) maxID = tracklets.at(i)->antID;
        }
        for(unsigned int i=0;i<tracklets.size();i++){
            if(tracklets.at(i)->selected){
                std::stringstream thisisdumb;
                thisisdumb<<"New track ID for "<<tracklets.at(i)->antID<<"@("<<tracklets.at(i)->startFrame<<", "<<tracklets.at(i)->endFrame<<")";
                bool isOk;
                int newID = QInputDialog::getInt(this,"New trackID",thisisdumb.str().c_str(),tracklets.at(i)->antID,0,2147483647,1,&isOk);
                if(!isOk) continue;
                int x,y,width,height;
                x= tracklets.at(i)->startFrame;
                y = newID*20;
                width = tracklets.at(i)->endFrame - tracklets.at(i)->startFrame;
                height = 19;
                tracklets.at(i)->setRect(x,y,width,height);
                tracklets.at(i)->antID = newID;
                if(newID > maxID){
                    for(int j=maxID+1;j<=newID;j++){
                        thisisdumb.str("");
                        thisisdumb.clear();
                        thisisdumb<<j;
                        ui->graphicsView->scene()->addText(thisisdumb.str().c_str())->setPos(0,j*20);
                    }
                    maxID = newID;
                    if(curFrameLine!=NULL){
                        curFrameLine->setLine(curFrameLine->line().x1(),curFrameLine->line().y1(),curFrameLine->line().x2(),(double)(maxID*20)+19);
                    }
                }
            }
        }
    }
}
