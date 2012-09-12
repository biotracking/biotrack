#include <QFile>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <typeinfo>
#include <fstream>

using namespace std;

MainWindow::MainWindow(	AbosPool *image_buf,
                       QVector<AbosThread*> modules,
                       QVector<ImageCapture*> captures,
                       QVector<AbosPool*> pools,
                       list<Blobs> *blobs,
                       int framerate, QString slider_output_file, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
    //, csvSliderOutput(slider_output_file)
{
    lowPassInited = false;

    ui->setupUi(this);

    fps = framerate;
    timer_id = 0;
    thread_vector = modules;
    capture_vector = captures;
    pool_vector = pools;
    image_pool = image_buf;
    blobs_vec = blobs;

    capture = NULL;

    isCaptureConfigured = false;
    isStart = false;
    p_status = v_status = false;

    // used for normalizing the slider values
    areaMax = 8000;
    velMax = 3.5;
    pixelsMax = 22000;

    // read the actual slider values
    read_doc = xmlReadFile("sliders.xml", "UTF-8", 0);
    if (read_doc != NULL) {
        read_root_node = xmlDocGetRootElement(read_doc);
        read_node = read_root_node->children;
        while (read_node->type == XML_TEXT_NODE)
            read_node = read_node->next;
    }
    else
        cerr<<"XML read error\n";

    write_doc = xmlNewDoc(BAD_CAST "1.0");
    write_root_node = xmlNewNode(NULL, BAD_CAST "blobs");
    xmlDocSetRootElement(write_doc, write_root_node);

    initParameters();

    if(SAVE_TO_XML)
        csvSliderOutput.open(slider_output_file.toStdString().c_str());

    currentFrameNumber = 1;
    gotFirstTwoFish = false;

    cerr<<"SAVE_TO_XML "<<SAVE_TO_XML<<"\n";
}

MainWindow::~MainWindow()
{

    csvSliderOutput.close();

    delete ui;
}

/*
  Initializing the parameters
  */
void MainWindow::initParameters() {

    char tmp[80];

    std::ifstream fileReadStream;
    fileReadStream.open("conf/areaMax.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;
        areaMax = atoi(tmp);  // assign the value
    }
    fileReadStream.close();

    fileReadStream.open("conf/velMax.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;
        velMax = atof(tmp);  // assign the value
    }
    fileReadStream.close();

    fileReadStream.open("conf/pixelsMax.conf");
    if( fileReadStream.is_open() ){
        fileReadStream >> tmp;
        pixelsMax = atoi(tmp);  // assign the value
    }
    fileReadStream.close();
}

void MainWindow::closeEvent(QCloseEvent *event) {
    bool done = false;
    std::cerr << "Close" << std::endl;
    QVectorIterator<AbosThread*> thread_iter(thread_vector);
    while(thread_iter.hasNext()){
        AbosThread* thread = thread_iter.next();
        thread->stop();
    }
    QVectorIterator<ImageCapture*> capture_iter(capture_vector);
    while(capture_iter.hasNext()){
        ImageCapture* capture = capture_iter.next();
        capture->stop();
    }
    QVectorIterator<AbosPool*> pool_iter(pool_vector);
    while(pool_iter.hasNext()){
        AbosPool* pool = pool_iter.next();
        pool->stop();
    }
    std::cerr<<"Waiting for threads";
    while(!done){
        done = true;
        QVectorIterator<AbosThread*> thread_iter(thread_vector);
        while(thread_iter.hasNext()){
            AbosThread* thread = thread_iter.next();
            done = (done && thread->wait(1000));
        }
        QVectorIterator<ImageCapture*> capture_iter(capture_vector);
        while(capture_iter.hasNext()){
            ImageCapture* capture = capture_iter.next();
            done = (done && capture->wait(1000));
        }
        QVectorIterator<AbosPool*> pool_iter(pool_vector);
        while(pool_iter.hasNext()){
            AbosPool* pool = pool_iter.next();
            done = (done && pool->wait(1000));
        }
        std::cerr<<".";
        std::cerr.flush();
    }

    xmlFreeDoc(read_doc);
    if (SAVE_TO_XML) {
        xmlSaveFormatFileEnc("observed_sliders.xml", write_doc, "UTF-8", 1);
    }
    xmlFreeDoc(write_doc);

    std::cerr<<"done."<<std::endl;
    QMainWindow::closeEvent(event);
}

void MainWindow::timerEvent(QTimerEvent*){

    updateUI();
}

/*
  Update all the controls on the UI
  */
void MainWindow::updateUI() {

    static unsigned long long int prevFrameNo = 9999;
    PoolFrame *id = NULL;
    if( (id = readFrame()) == NULL ) {
        //std::cerr << "A gap detected between frames" << std::endl;
        return;
    }

    if ( id != NULL ) {
        unsigned long long int frameNo = id->getFrameNumber();
        if (prevFrameNo == frameNo)
            return;
        prevFrameNo = frameNo;
        this->produceSharedQImage(*(id->getImage()));
        image_pool->releaseFrame(id);
        ui->frameSlider->setValue(frameNo);

        if (blobs_vec->size() > 0) {
            Blobs blobs = blobs_vec->back();
            updateRawSliders(blobs);
        }
        updateExpectedSliders(frameNo);

        if (frameNo >= capture->getFrameCount() - 1)
            startThreads(false);
    }
}

/*
  Updates the raw blobs with the calculated values
  */
void MainWindow::updateRawSliders(Blobs blobs) {

    double avgVel = blobs.computeAverageVelocity();
    int numPixels = blobs.computeTotalArea();
    //cerr << "numPixels: " << numPixels << endl;


    Blob* fish1 = NULL;
    Blob* fish2 = NULL;

    set< int > usedLabels;

    // if labels from previous frame are still here, use them
    Blob* blobMatchedWithPreviousFish1 = blobs.getBlobByLabel(previousFish1Label);
    if(gotFirstTwoFish && blobMatchedWithPreviousFish1 != NULL)
    {
        fish1 = blobMatchedWithPreviousFish1;
        usedLabels.insert(fish1->label);
        //cerr << "took previous fish111111111111111" << endl;
    }
    Blob* blobMatchedWithPreviousFish2 = blobs.getBlobByLabel(previousFish2Label);
    if(gotFirstTwoFish && blobMatchedWithPreviousFish2 != NULL)
    {
        fish2 = blobMatchedWithPreviousFish2;
        usedLabels.insert(fish2->label);
        //cerr << "took previous fish222222222222222" << endl;
    }

    // if we don't have blobs from previous frame, use biggest new blobs
    //cerr << "total blobs: " << blobs.num_blobs << ", usedLabels.size(): " << usedLabels.size() << endl;
    vector< Blob* > twoLargestUsableBlobs = blobs.getTwoLargestBlobsWithLabelsNotIn(usedLabels);
    if(fish1 == NULL && twoLargestUsableBlobs.size() > 0)
    {
        fish1 = twoLargestUsableBlobs.back();
        twoLargestUsableBlobs.pop_back();
    }
    if(fish2 == NULL && twoLargestUsableBlobs.size() > 0)
        fish2 = twoLargestUsableBlobs.back();


    if(fish1 != NULL)
    {
        previousFish1Label = fish1->label;
        //cerr << "fish1->label: " << fish1->label;
    }
    if(fish2 != NULL)
    {
        //cerr << ", fish2->label: " << fish2->label;
        previousFish2Label = fish2->label;
    }
    //cerr << endl << endl;

    if(fish1 != NULL && fish2 != NULL)
        gotFirstTwoFish = true;

    assert(
        !gotFirstTwoFish
        ||
        blobs.num_blobs < 2
        ||
        (fish1 != fish2 && fish1->label != fish2->label)
    );

    // normalize the values to 127
    float size1 = -1, x1 = -1, y1 = -1, size2 = -1, x2 = -1, y2 = -1;
    if(fish1 != NULL)
    {
        size1 = fish1->size * ((double)127 / areaMax);
        x1 = fish1->centroid.x * ((double)127 / input_width);
        y1 = fish1->centroid.y * ((double)127 / input_height);
    }
    if(fish2 != NULL)
    {
        size2 = fish2->size * ((double)127 / areaMax);
        x2 = fish2->centroid.x * ((double)127 / input_width);
        y2 = fish2->centroid.y * ((double)127 / input_height);
    }

    double activity = avgVel * (double)127 / velMax;
    int density = numPixels * (double)127 / pixelsMax;

    if (size1 <= 0)
        size1 = -1;
    else if (size1 > 127)
        size1 = 127;

    if (size2 <= 0)
        size2 = -1;
    else if (size2 > 127)
        size2 = 127;

    if (activity > 127)
        activity = 127;

    if (density > 127)
        density = 127;


    // low pass filter size1, x1, y1, size2, x2, y2, activity, density
    if(!lowPassInited)
    {
        size1LowPass = size1;
        size2LowPass = size2;
        x1LowPass = x1;
        x2LowPass = x2;
        y1LowPass = y1;
        y2LowPass = y2;
        activityLowPass = activity;
        densityLowPass = density;

        lowPassInited = true;
    }
    else
    {
        size1LowPass = LOW_PASS_POWER * size1LowPass    + (1.0 - LOW_PASS_POWER) * size1;
        size2LowPass = LOW_PASS_POWER * size2LowPass    + (1.0 - LOW_PASS_POWER) * size2;
        x1LowPass = LOW_PASS_POWER * x1LowPass       + (1.0 - LOW_PASS_POWER) * x1;
        x2LowPass = LOW_PASS_POWER * x2LowPass       + (1.0 - LOW_PASS_POWER) * x2;
        y1LowPass = LOW_PASS_POWER * y1LowPass       + (1.0 - LOW_PASS_POWER) * y1;
        y2LowPass = LOW_PASS_POWER * y2LowPass       + (1.0 - LOW_PASS_POWER) * y2;
        activityLowPass = LOW_PASS_POWER * activityLowPass + (1.0 - LOW_PASS_POWER) * activity;
        densityLowPass = LOW_PASS_POWER * densityLowPass  + (1.0 - LOW_PASS_POWER) * density;

        size1    =    size1LowPass;
        size2    =    size2LowPass;
        x1       =       x1LowPass;
        x2       =       x2LowPass;
        y1       =       y1LowPass;
        y2       =       y2LowPass;
        activity = activityLowPass;
        density  =  densityLowPass;
    }



    // Update raw blobs (right side box)
    ui->size1->setValue(size1);
    ui->x1->setValue(x1);
    ui->y1->setValue(y1);

    ui->size2->setValue(size2);
    ui->x2->setValue(x2);
    ui->y2->setValue(y2);

    ui->avgVel->setValue(activity);
    ui->numPixels->setValue(density);

    // Update mapped blobs
    // TODO: move this into another function which will be the
    // mapping function
    ui->o_fish1_size->setValue(size1);
    ui->o_fish1_xPos->setValue(x1);
    ui->o_fish1_yPos->setValue(y1);
    ui->o_fish2_size->setValue(size2);
    ui->o_fish2_xPos->setValue(x2);
    ui->o_fish2_yPos->setValue(y2);
    ui->o_activity->setValue(activity);
    ui->o_density->setValue(density);

    if (SAVE_TO_XML) {
        //while (prevFrameNo != blobs.frameNum) {
            addToXml(blobs.frameNum, size1, x1, abs(127 - y1), size2, x2, abs(127 - y2), activity, density, 0);
            //prevFrameNo++;
        //}
    }

    // draw triangle to mark the two fishes
    QPainter painter(&pixmap);
    painter.setBrush(Qt::red);

    QPoint point1, point2;
    if(fish1 != NULL)
        point1 = QPoint(fish1->centroid.x * ((double)ui->imageLabel->width() / input_width), fish1->centroid.y * ((double)ui->imageLabel->height() / input_height));
    if(fish2 != NULL)
        point2 = QPoint(fish2->centroid.x * ((double)ui->imageLabel->width() / input_width), fish2->centroid.y * ((double)ui->imageLabel->height() / input_height));

    QPoint point11(point1.x(), point1.y() - 6);
    QPoint point12(point1.x() + 6, point1.y() + 6);
    QPoint point13(point1.x() - 6, point1.y() + 6);

    QPoint point21(point2.x(), point2.y() - 6);
    QPoint point22(point2.x() + 6, point2.y() + 6);
    QPoint point23(point2.x() - 6, point2.y() + 6);

    QPoint points[3];
    points[0] = point11;
    points[1] = point12;
    points[2] = point13;

    painter.drawPolygon(points, 3);

    painter.setBrush(Qt::green);

    points[0] = point21;
    points[1] = point22;
    points[2] = point23;

    painter.drawPolygon(points, 3);


    // Update the x-axis and y-axis blobs for both fishes
    /*ui->fish1_XPos->setValue();
   ui->fish1_YPos->setValue();
   ui->fish2_XPos->setValue();
   ui->fish2_YPos->setValue();*/

    //   cerr<<"Calculated: "<<blobs.frameNum<<" "<<size1<<" "<<x1<<" "<<y1<<" "<<size2<<" "<<x2<<" "<<y2<<"\n";
}

/*
  Updates the expected blobs with values read from xml
  */
void MainWindow::updateExpectedSliders(unsigned long long int frameNo) {

    if (read_doc) {
        long long int xmlFrameNo = -1;
        if (read_node) {
            xmlChar* xmlString = xmlGetProp(read_node, BAD_CAST "id");
            xmlFrameNo = atoi((char*)xmlString);
            while (xmlFrameNo < (signed long long)frameNo && read_node) {
                read_node = read_node->next;
                if (read_node->type == XML_ELEMENT_NODE) {
                    xmlString = xmlGetProp(read_node, BAD_CAST "id");
                    xmlFrameNo = atoi((char*)xmlString);
                }
            }
            xmlNodePtr child1 = read_node->children;
            while (xmlStrcmp(child1->name, BAD_CAST "FISH1"))
                child1 = child1->next;
            xmlString = xmlGetProp(child1, BAD_CAST "size");
            int size1 = atoi((char*)xmlString);
            xmlString = xmlGetProp(child1, BAD_CAST "x");
            int x1 = atoi((char*)xmlString);
            xmlString = xmlGetProp(child1, BAD_CAST "y");
            int y1 = atoi((char*)xmlString);
            y1 = abs(127 - y1);

            xmlNodePtr child2 = child1->next;
            while (xmlStrcmp(child2->name, BAD_CAST "FISH2"))
                child2 = child2->next;
            xmlString = xmlGetProp(child2, BAD_CAST "size");
            int size2 = atoi((char*)xmlString);
            xmlString = xmlGetProp(child2, BAD_CAST "x");
            int x2 = atoi((char*)xmlString);
            xmlString = xmlGetProp(child2, BAD_CAST "y");
            int y2 = atoi((char*)xmlString);
            y2 = abs(127 - y2);

            xmlNodePtr child3 = child2->next;
            while (xmlStrcmp(child3->name, BAD_CAST "OVERALL"))
                child3 = child3->next;
            xmlString = xmlGetProp(child3, BAD_CAST "activity");
            int activity = atoi((char*)xmlString);
            xmlString = xmlGetProp(child3, BAD_CAST "density");
            int density = atoi((char*)xmlString);
            xmlString = xmlGetProp(child3, BAD_CAST "excitement");
            int excitement = atoi((char*)xmlString);


            if (size1 == -1) {
                x1 = -1;
                y1 = -1;
            }

            if (size2 == -1) {
                x2 = -1;
                y2 = -1;
            }

            ui->e_fish1_size->setValue(size1);
            ui->e_fish1_xPos->setValue(x1);
            ui->e_fish1_yPos->setValue(y1);
            ui->e_fish2_size->setValue(size2);
            ui->e_fish2_xPos->setValue(x2);
            ui->e_fish2_yPos->setValue(y2);
            ui->e_activity->setValue(activity);
            ui->e_density->setValue(density);
            ui->e_excitement->setValue(excitement);

            // draw circle to mark the two fishes
            QPainter painter(&pixmap);
            painter.setBrush(Qt::red);
            QPoint point1(x1 * (ui->imageLabel->width() / 127.0), y1 * (ui->imageLabel->height() / 127.0));
            painter.drawEllipse(point1, 6, 6);
            painter.setBrush(Qt::green);
            QPoint point2(x2 * (ui->imageLabel->width() / 127.0), y2 * (ui->imageLabel->height() / 127.0));
            painter.drawEllipse(point2, 6, 6);
            painter.end();
            ui->imageLabel->setPixmap(pixmap);

            //       cerr<<"Expected: "<<xmlFrameNo<<" "<<size1<<" "<<x1<<" "<<y1<<" "<<size2<<" "<<x2<<" "<<y2<<" "<<activity<<" "<<excitement<<"\n\n";
            xmlFree(xmlString);
        }
    }
}

/**
  Write the observed blobs to the xml file
  */
void MainWindow::addToXml(int frameNo, int size1, int x1, int y1, int size2, int x2, int y2, int activity, int density, int excitement) {

    csvSliderOutput << frameNo << " " << size1 << " " << x1 << " " << y1 << " " << size2 << " " << x2 << " " << y2 << " " << activity << " " << density << " " << excitement << std::endl;


    xmlNodePtr child = xmlNewChild(write_root_node, NULL, BAD_CAST "FRAME", NULL);
    sprintf(buff, "%d", frameNo);
    xmlNewProp(child, BAD_CAST "id", BAD_CAST (buff));

    xmlNodePtr child1 = xmlNewChild(child, NULL, BAD_CAST "FISH1", NULL);
    sprintf(buff, "%d", size1);
    xmlNewProp(child1, BAD_CAST "size", BAD_CAST (buff));
    sprintf(buff, "%d", x1);
    xmlNewProp(child1, BAD_CAST "x", BAD_CAST (buff));
    sprintf(buff, "%d", y1);
    xmlNewProp(child1, BAD_CAST "y", BAD_CAST (buff));

    xmlNodePtr child2 = xmlNewChild(child, NULL, BAD_CAST "FISH2", NULL);
    sprintf(buff, "%d", size2);
    xmlNewProp(child2, BAD_CAST "size", BAD_CAST (buff));
    sprintf(buff, "%d", x2);
    xmlNewProp(child2, BAD_CAST "x", BAD_CAST (buff));
    sprintf(buff, "%d", y2);
    xmlNewProp(child2, BAD_CAST "y", BAD_CAST (buff));

    xmlNodePtr child3 = xmlNewChild(child, NULL, BAD_CAST "OVERALL", NULL);
    sprintf(buff, "%d", activity);
    xmlNewProp(child3, BAD_CAST "activity", BAD_CAST (buff));
    sprintf(buff, "%d", density);
    xmlNewProp(child3, BAD_CAST "density", BAD_CAST (buff));
    sprintf(buff, "%d", excitement);
    xmlNewProp(child3, BAD_CAST "excitement", BAD_CAST (buff));
}

/**
  read a frame from capture or from image_pool.
  if we are using the threaded model,
  it should read from the image_pool which is global beyond threads.
  */
PoolFrame* MainWindow::readFrame(){
    if(image_pool->getSize() == 0)
        return NULL;

    PoolFrame* frame = NULL;
    if(qApp->property("processingMode") == "Realtime")
    {
        frame = image_pool->getRecentFrame();
    }
    else
    {
//        frame = image_pool->getOldestFrame();
        frame = image_pool->getFrameByNumber(currentFrameNumber);
        if(frame != NULL)
            currentFrameNumber++;
    }

    return frame;
}

/*
  Converts the cv::Mat to a Qt image
  */
void MainWindow::produceSharedQImage(cv::Mat cvimage){

    //TODO: do a static assignment
    input_width = cvimage.cols;
    input_height = cvimage.rows;
    qimage = QImage(cvimage.data,cvimage.size().width,cvimage.size().height, QImage::Format_RGB888);
    qimage = qimage.rgbSwapped();
    QImage image = qimage.scaledToWidth(ui->imageLabel->width());
    pixmap = QPixmap::fromImage(image);
    //ui->imageLabel->setPixmap(pixmap);
}

/*
  Start or stop the threads
  */
void MainWindow::startThreads(bool start) {

    if( start ) {
        capture->start();
        timer_id = startTimer(100 / fps);
        isStart = true;
    }
    else{
        capture->stop();
        if (timer_id != 0) {
            killTimer(timer_id);
            timer_id = 0;
        }
        isStart = false;
        read_node = read_root_node->children;
        while (read_node->type == XML_TEXT_NODE)
            read_node = read_node->next;
    }
    QVectorIterator<AbosThread*> iter(thread_vector);
    while( iter.hasNext() ){
        AbosThread *thread = iter.next();

        if(start){
            thread->start();
        }
        else{
            thread->stop();
        }
    }
    isStart = start;
}

void MainWindow::on_actionStart_Tracking_triggered()
{
    if(capture == NULL)
        return;

    ui->frameSlider->setEnabled(false);
    this->startThreads(true);
}

void MainWindow::on_actionStop_Tracking_triggered()
{
    this->startThreads(false);
    ui->frameSlider->setEnabled(true);
}

void MainWindow::on_actionExit_triggered()
{
    this->close();
}

void MainWindow::on_actionProsilica_Camera_triggered()
{
    bool prev_isStart = isStart;
    if(capture != NULL) this->startThreads(false);

    if(!p_status){
        capture = this->capture_vector.at(1);

        if( capture->isSet() ){
            p_status = true;
            v_status = false;
            ui->actionVideo_File->setChecked(false);
            isCaptureConfigured = true;

            ui->frameSlider->setMaximum(0);
            ui->frameSlider->setEnabled(false);

            if( prev_isStart ) this->startThreads(true);
        }
        else{
            p_status = false;
            ui->actionProsilica_Camera->setChecked(false);
            // dialog display
            QMessageBox msgBox;
            msgBox.setText("Cannot find Prosilica camera(s) installed or unexpected error(s) occurred while trying to open");
            msgBox.exec();
            if (ui->actionVideo_File->isChecked())
                capture = this->capture_vector.at(0);
        }
    }
    else{
        capture = NULL;
        p_status = false;
        ui->actionProsilica_Camera->setChecked(false);
        isCaptureConfigured = false;
    }
}

void MainWindow::on_actionVideo_File_triggered()
{
    bool prev_isStart = isStart;
    if(capture != NULL ) this->startThreads(false);

    if(!v_status){
        capture = this->capture_vector.at(0);

        v_status = true;
        p_status = false;
        ui->actionProsilica_Camera->setChecked(false);
        isCaptureConfigured = true;

        ui->frameSlider->setMaximum(capture->getFrameCount());

        if( prev_isStart ) this->startThreads(true);
    }
    else{
        capture = NULL;
        v_status = false;
        ui->actionVideo_File->setChecked(false);
        isCaptureConfigured = false;
    }
}
