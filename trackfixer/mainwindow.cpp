#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <fstream>
#include <typeinfo>


MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->statusBar->setSizeGripEnabled(true);
	fps = 30.0;
    //timerID=startTimer(100 / fps);  // timer
    timerID=-1;
    paused = true;
    timestamp_idx = x_idx = y_idx = t_idx = id_idx = -1;
    scaleFactor = 1.0;
    frame_count=0;
    ui->imageLabel->setScaledContents(true);
    ui->graphicsView->setScene(&scene);
    curFrameLine = NULL;
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::clearBTFData(){
	for(unsigned int i=0; i<btf_data.size();i++){
		btf_data[i].clear();
	}
	btf_data.clear();
	for(unsigned int i=0;i<frame_data.size();i++){
		frame_data[i].clear();
	}
	frame_data.clear();
	flip_data.clear();
	btf_names.clear();
}

void MainWindow::loadVideo(std::string fname){
    //loadedVideo.clear();
    //cv::VideoCapture cap;
    cap.open(fname);
    fps = cap.get(CV_CAP_PROP_FPS);
    std::cout<<"Video FPS:"<<fps<<std::endl;
    frame_count = cap.get(CV_CAP_PROP_FRAME_COUNT);
    std::cout<<"Frames:"<<frame_count<<std::endl;
    cv::Mat in,out;
    /*
    std::cout<<"Loading video ["<<fname<<"]";
    std::cout.flush();
    int ctr = 0;
    while(cap.read(in)){
        loadedVideo.push_back(in.clone());
        ctr++;
        if(ctr%(int)(fps*10)==0){
            std::cout<<".";
            std::cout.flush();
        }
        if(in.cols == 0 || in.rows==0){
            std::cout<<"Whoops!"<<std::endl;
        }
    }
    std::cout<<" Done! ["<<loadedVideo.size()<<" frames loaded]"<<std::endl;
    */
    paused = true;
    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setMaximum(frame_count);
    ui->horizontalSlider->setValue(0);
    cap.read(in);
    //in = loadedVideo.at(0);
    //ui->imageLabel->setMaximumSize(in.cols,in.rows);
    out = in.clone();
    cv::cvtColor(in,out,CV_BGR2RGB);
    QImage img = Mat2QImage(out);
    ui->imageLabel->setPixmap(QPixmap::fromImage(img));
    //updateGeometry();
    //ui->imageLabel->resize(800,600);
}

void MainWindow::loadBTF(std::string dname){
	clearBTFData();
    std::cout<<"Parsing BTF from ["<<dname<<"]"<<std::endl;
    QStringList filters, files;
    filters<<"*.btf";
    QDir dir(dname.c_str());
    files = dir.entryList(filters);
    for(unsigned int i=0;i<btf_data.size();i++){
        btf_data.at(i).clear();
    }
    btf_data.clear();
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
        for(int flarfle = 0;flarfle<idsAndStarts.size();flarfle++) marked[flarfle]=false;
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
                ui->graphicsView->scene()->addRect(x,y,width,height,QPen(),QBrush(Qt::yellow));
            }
        }
        delete [] marked;
        //std::string id = btf_data.at(id_idx).at(i);
        //if(ui->listWidget->findItems(QString(id.c_str()),Qt::MatchExactly).size()<=0){
        //    ui->listWidget->addItem(QString(id.c_str()));
        //}
        /*
        std::cout<<"scratch [";
        for(unsigned int flarfle=0;flarfle<scratch.size();flarfle++) std::cout<<" "<<scratch.at(flarfle).first;
        std::cout<<"]"<<std::endl;
        std::cout<<"idsAndStarts [";
        for(unsigned int flarfle=0;flarfle<idsAndStarts.size();flarfle++) std::cout<<" "<<idsAndStarts.at(flarfle).first;
        std::cout<<"]"<<std::endl;
        */
        idsAndStarts = scratch;
    }
    for(unsigned int i=0;i<idsAndStarts.size();i++){
        int x,y,width,height;
        x = idsAndStarts.at(i).second;
        y = atoi(idsAndStarts.at(i).first.c_str())*20;
        width = frame_data.size()-x;
        height = 19;
        if(y+height > maxID) maxID = y+height;
        ui->graphicsView->scene()->addRect(x,y,width,height,QPen(),QBrush(Qt::yellow));
    }
    curFrameLine = ui->graphicsView->scene()->addLine(0,0,0,maxID,QPen(Qt::red));
    //ui->graphicsView->scene()->addRect(0,0,frame_data.size(),18,QPen(),QBrush(Qt::red));
    //ui->listWidget->sortItems();
    /*
    std::cout<<"Frame 684 contains lines: [";
    for(unsigned int i=0;i<frame_data.at(684).size();i++){
        std::cout<<frame_data.at(684).at(i)<<" ";
    }
    std::cout<<"]"<<std::endl;
    */
    /*
    std::cout<<"Done!"<<std::endl;
    for(unsigned int i=0;i<btf_names.size();i++){
        if(btf_names.at(i).compare(std::string("id.btf")) == 0){
            for(unsigned int j=0;j<btf_data.at(i).size();j++){
                std::string id = btf_data.at(i).at(j);
                if(ui->listWidget->findItems(QString(id.c_str()),Qt::MatchExactly).size()<=0){
                    ui->listWidget->addItem(QString(id.c_str()));
                }
            }
        }
        if(btf_names.at(i).compare(std::string("timestamp.btf")) == 0){
            int count=0;
            std::string lastTS = "33";
            for(unsigned int j=0;j<btf_data.at(i).size();j++){
                if(lastTS.compare(btf_data.at(i).at(j))!=0){
                    count++;
                    lastTS=btf_data.at(i).at(j);
                }
                std::stringstream thisisdumb;
                thisisdumb<<count;
                btf_frameno.push_back(thisisdumb.str());
            }
        }
    }
    */
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
    loadVideo( QFileDialog::getOpenFileName(this).toStdString() );
    ui->statusBar->showMessage("Done!");
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
    //in = loadedVideo[curFrame];
    
    out = in.clone();
    cv::cvtColor(in,out,CV_BGR2RGB);
    //for(int i=0;i<ui->listWidget->count();i++){
        bool found = false;
        //std::string trackId = ui->listWidget->item(i)->text().toStdString();
        if(value<((int)frame_data.size())){
            for(unsigned int j=0;j<frame_data.at(value).size();j++){
                std::string trackId = btf_data.at(id_idx).at(frame_data.at(value).at(j));
                //if(trackId.compare(btf_data.at(id_idx).at(frame_data.at(value).at(j)))==0){
                    double scaledX = (scaleFactor*atof(btf_data.at(x_idx).at(frame_data.at(value).at(j)).c_str()));
                    double scaledY = (scaleFactor*atof(btf_data.at(y_idx).at(frame_data.at(value).at(j)).c_str()));
                    bool flip_coeff = false;
                    for(unsigned int flip_idx=0;flip_idx<flip_data.size();flip_idx++){
                        if(flip_data.at(flip_idx).second<value){
                            if(trackId.compare(flip_data.at(flip_idx).first)==0){
                                flip_coeff = !flip_coeff;
                                //std::cout<<"FLIPPED"<<std::endl;
                            }
                        }
                    }
                    double theta = ((flip_coeff)?M_PI:0)+atof(btf_data.at(t_idx).at(frame_data.at(value).at(j)).c_str());
                    int radius = 10;
                    int lineWidth = 2;
                    //std::cout<<"(";
                    //std::cout<<(int)(scaleFactor*atof(btf_data.at(x_idx).at(frame_data.at(value).at(j)).c_str()))<<", ";
                    //std::cout<<(int)(scaleFactor*atof(btf_data.at(y_idx).at(frame_data.at(value).at(j)).c_str()))<<") ";
                    //std::cout<<"("<<in.cols<<", "<<in.rows<<")"<<std::endl;
                    cv::Scalar btfColor(255,0,0);
                    cv::circle(out,cv::Point(scaledX,scaledY),radius,btfColor,lineWidth);
                    double endPtX = (radius*cos(theta))+scaledX;
                    double endPtY = (radius*sin(theta))+scaledY;
                    cv::line(out,cv::Point(scaledX,scaledY),cv::Point(endPtX,endPtY),btfColor,lineWidth);
                    cv::putText(out,trackId,cv::Point(scaledX,scaledY),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(0,0,255),2);
                    //found = true;
                    //break;
                //}
            }
        }
        if(found){
            //ui->listWidget->item(i)->setBackground(QBrush(QColor::fromRgb(255,255,255)));
        } else {
            //ui->listWidget->item(i)->setBackground(QBrush(QColor::fromRgb(200,200,200)));
        }
    //}
    QImage img = Mat2QImage(out);
    //img.scaledToHeight(ui->imageLabel->height());
    //ui->imageLabel->setPixmap(QPixmap::fromImage(img));
    ui->imageLabel->setPixmap(QPixmap::fromImage(img).scaledToWidth((int)(out.cols*ui->doubleSpinBox->value())));
    curFrameLine->setPos(value,0);
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
    /*
    for(int i=0;i<ui->listWidget->selectedItems().size();i++){
        std::pair<std::string,int> tmpPair;
        tmpPair.first = ui->listWidget->selectedItems().at(i)->text().toStdString();
        tmpPair.second = ui->horizontalSlider->value();
        flip_data.push_back(tmpPair);
        //std::cout<<"Added flip:["<<tmpPair.first<<", "<<tmpPair.second<<"]"<<std::endl;
    }
    */
}

void MainWindow::on_pushButton_7_clicked()
{
    //QString file_name = QFileDialog::getSaveFileName(this,"Save BTF as","timage.btf");
    QString dir_name = QFileDialog::getExistingDirectory(this,"BTF Save Directory");
    dir_name = dir_name+"/";
    std::cout<<"Writting BTF to ["<<dir_name.toStdString()<<"]"<<std::endl;
    std::cout<<"Ignored tracks: [ ";
    for(unsigned int i=0;i<removed_tracks.size();i++){
        std::cout<<removed_tracks[i]<<" ";
    }
    std::cout<<"]"<<std::endl;
    std::cout<<"Track flips: [ ";
    for(unsigned int i=0;i<flip_data.size();i++){
        std::cout<<"("<<flip_data.at(i).first<<", "<<flip_data.at(i).second<<") ";
    }
    std::cout<<"]"<<std::endl;
    std::vector<std::fstream* > files;
    for(unsigned int i=0;i<btf_names.size();i++){
        std::fstream *a_file = new std::fstream();
        a_file->open((dir_name+btf_names.at(i).c_str()).toStdString().c_str(),std::fstream::out);
        files.push_back(a_file);
    }

    unsigned int frameNo = 0;
    for(unsigned int i=0;i<btf_data.at(id_idx).size()-1;i++){
        //figure out which frame this line belongs to
        for(;frameNo<frame_data.size();frameNo++){
            bool foundIt = false;
            for(unsigned int j=0;j<frame_data.at(frameNo).size();j++){
                if(frame_data.at(frameNo).at(j)==i){
                    foundIt = true;
                    break;
                }
            }
            if(foundIt) break;
        }
        //figure out which ID this belongs to
        std::string trackId = btf_data.at(id_idx).at(i);
        bool ignored = false;
        for(unsigned int j=0;j<removed_tracks.size();j++){
            if(removed_tracks.at(j)==atoi(trackId.c_str())){
                ignored=true;
                break;
            }
        }
        if(ignored) continue;
        //figure out what un-corrected theta is
        //std::stringstream alsodumb(btf_data.at(t_idx).at(i));
        double theta;
        theta = atof(btf_data.at(t_idx).at(i).c_str());
        //count how many times this ID's theta has flipped between start and this frame
        bool flip_coeff = false;
        for(unsigned int j=0;j<flip_data.size();j++){
            if(flip_data.at(j).second<frameNo){
                if(trackId.compare(flip_data.at(j).first)==0){
                    flip_coeff = !flip_coeff;
                }
            }
        }
        //write out correct theta
        theta = ((flip_coeff)?M_PI:0)+theta;
        if(theta>(2*M_PI)) theta=theta-(2*M_PI);
        for(unsigned int j=0;j<files.size();j++){
            if(j==t_idx){
                files.at(j)->precision(15);
                (*files.at(j))<<theta<<std::endl;
            } else {
                (*files.at(j))<<btf_data.at(j).at(i)<<std::endl;
            }
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
    /*
    for(int i=0;i<ui->listWidget->selectedItems().size();i++){
        int trackNo = ui->listWidget->selectedItems().at(i)->text().toInt();
        bool found = false;
        for(std::vector<int>::iterator j=removed_tracks.begin(); j<removed_tracks.end(); j++){
            if(*j == trackNo){
                removed_tracks.erase(j);
                found=true;
            }
        }
        if(!found){
            removed_tracks.push_back(ui->listWidget->selectedItems().at(i)->text().toInt());
        }
    }
    */
}
