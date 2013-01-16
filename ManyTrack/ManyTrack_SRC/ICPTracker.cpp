#include "ICPTracker.h"





void
viewerOneOff (pcl::visualization::PCLVisualizer& viewerT)
{
    //    pcl:: PointCloud<PointXYZRGB>::Ptr data_cloud_PTR (new pcl::PointCloud<PointXYZRGB> (data_cloud));

    viewerT.setBackgroundColor (1.0, 0.5, 1.0);
    //    viewerT->updatePointCloud(data_cloud_PTR,"datacloud");
    //viewerT.updatePointCloud()
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    //    viewerT.addSphere (o, 0.25, "sphere", 0);
    qDebug()<< "i only run once";

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    //This pops open a new cloud everytime
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 40, 0);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}


ICPTracker::ICPTracker(float fps, QString bgImagepath, QString modelFolderpath, QString maskImagepath, Ui::ManytrackClass uiPass)
{


    mFolderPath = modelFolderpath;
    uiICP=uiPass;
    thefps=fps;

    numOfTracks = 0;

    //Set some default values
    //Initialize values based on UI now!

    //TODO fix redundancies
    showSearchRadius=uiICP.trackDistanceViewCheck->isChecked();
    showRemovalRadii=uiICP.separationViewCheck->isChecked();
    showModel=uiICP.modelViewcheckBox->isChecked();
    showTrails = uiICP.showTrailscheckBox->isChecked();
    showBox =  uiICP.showBoxcheckBox->isChecked();

    resolutionFractionMultiplier = uiICP.resolutionSpinBox->value(); // 4 for 1/4 resolution, 2 for 1/2 resolution, 1 for full resolution

    bgsubstractionThreshold = uiICP.bgSubThresholdSpinBox->value();
    modelTOdataThreshold = uiICP.healthyPercentageThresholdSpinBox->value(); // percentage of model points to be considered healthy
    trackDeathThreshold=uiICP.trackdeathSpinBox->value();


//    isVideoShowing = true;
    separationThreshold=uiICP.separationSpinBox->value();
    icpMatchDistanceThreshold = uiICP.trackdistanceSpinBox->value();
    Ticp_maxIter=uiICP.ICP_MaxIterspinBox->value();
    Ticp_transformationEpsilon=uiICP.ICP_TransEpsilondoubleSpinBox->value();
    Ticp_euclideanDistance=uiICP.ICP_EuclideanDistdoubleSpinBox->value();
    colorRegScale=uiICP.colorRegSpinBox->value();




    /*
      * Load All canned data
      */
    //Load background image
    bgImage = imread(bgImagepath.toStdString());

    //Make visual overlay transparent
    QImage qimage = QImage((const uchar*)bgImage.data, bgImage.cols, bgImage.rows, QImage::Format_ARGB32);

    QPixmap transparent(qimage.size());
    // Do transparency
    transparent.fill(Qt::transparent);
    uiICP.visualizationLabel->setPixmap(transparent);


    //Create useful copies of other images
    bgSubImage = bgImage.clone();

    trackResultImage = bgImage.clone();
    cv::cvtColor(bgSubImage, bgSubImageGray, CV_BGR2GRAY);

    //NOTE, resize NEEEEEEDS floats in the scale factor or else it FAILS!
    cv::resize(bgSubImageGray,bgSubImageGraySmall,Size(),1./resolutionFractionMultiplier,1./resolutionFractionMultiplier);

    //make mask conditional
    if(maskImagepath.isNull() || maskImagepath=="(none selected)"){
        //Don't load the mask or care about it!
    }

    else{
        //the user chose a mask for us to use!
        Mat maskImage;
        maskImage =bgImage.clone();
        maskImage = imread(maskImagepath.toStdString()); //Can put flag if want to include transparency, else strips Alpha channel
        cv::cvtColor(maskImage, maskImageGray, CV_BGR2GRAY);

        maskImage.release();
    }

    //Load list of modelfiles, turn them into a paired vector of MATS

    //    model_clouds_orig = loadModelClouds(mFolderPath  );
    models.clear();
    models = loadModelClouds(mFolderPath);

    qDebug()<< "Number of Models loaded:" << models.size();


    // 3D Visualization Stuff
    /**  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer2->setBackgroundColor (0, 0, 0);
    //  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    //  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer2->addCoordinateSystem (1.0);
    viewer2->initCameraParameters ();
    viewer = viewer2;
    pcl:: PointCloud<PointXYZRGB>::Ptr data_cloud_PTR (new pcl::PointCloud<PointXYZRGB> (data_cloud));

    viewer->addPointCloud<pcl::PointXYZRGB> (data_cloud_PTR, "datacloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "datacloud");
    /**/
}

ICPTracker::~ICPTracker()
{

    if(!trackResultImage.empty()){
        trackResultImage.release();
    }
    if(!modelImage.empty()){
        modelImage.release();
    }
    bgImage.release();
    bgSubImage.release();
    bgSubImageGray.release();
    bgSubImageGraySmall.release();
    trackResultImage.release();


    if(maskImageGray.empty()){
        //Don't care about mask
    }
    else{
        maskImageGray.release();
    }

    activeTracks.clear();
    inActiveTracks.clear();
    data_cloud.clear();
}

void ICPTracker::MattoCloudDetections(Mat img){

    //Rescale from full size down to our resampled size
    cv::resize(bgSubImageGray, bgSubImageGraySmall, Size(), 1./resolutionFractionMultiplier, 1./resolutionFractionMultiplier);

    Mat imgsmall;
    cv::resize(img, imgsmall, Size(), 1./resolutionFractionMultiplier, 1./resolutionFractionMultiplier); //full color reduced image
    Mat imgsmallgray;
    cvtColor(imgsmall,imgsmallgray,CV_BGR2GRAY);
    Mat imgsmallHSV;
    cvtColor(imgsmall,imgsmallHSV,CV_BGR2HSV);


    ///////////////
    /// The Biggest Loop
    /////////////


    ///////////////
    /// Collect Detections into Cloud of Data Points
    /////////////
    // iterate over decimated segmentation and create an array of data points (detections)
    // for each foreground pixel (non black pixels are foreground)


    // cvtColor(bgSubImage,bgSubImageGray, CV_BGR2GRAY);// BGR -> gray
    //Get as many hard coded values as possible before we go through expensive looping!
    int grayrows=bgSubImageGraySmall.rows; // number of lines
    int graycols = bgSubImageGraySmall.cols; // number of columns
    int graystep = bgSubImageGraySmall.step;
    int grayelemsize= bgSubImageGraySmall.elemSize();

    //Color values
    int crows=imgsmall.rows; // number of lines
    int ccols = imgsmall.cols; // number of columns
    int cstep = imgsmall.step;
    int celemsize= imgsmall.elemSize();

    int gstep = imgsmallgray.step;
    int gelemsize= imgsmallgray.elemSize();

    int HSVstep = imgsmallHSV.step;
    int HSVelemsize= imgsmallHSV.elemSize();

    uchar pixval = 0;
    uchar pixvalcolorR = 0;
    uchar pixvalcolorG = 0;
    uchar pixvalcolorB = 0;
    uchar pixvalGray = 0;
    uchar pixvalHue = 0;







    //This is a quick define for fast pixel access
#define aPixel(type, dataStart,step,size,x,y,channel)*((type*)(dataStart+step*(y)+(x)*size+channel)) // This says it is fast (maybe a lie)

    pcl::PointCloud<pcl::PointXYZRGB> temp_data_cloud;
    pcl:: PointCloud<PointXYZRGB> senddata_cloud;// (new pcl::PointCloud<PointXYZRGB> (data_cloud));

    copyPointCloud(data_cloud, temp_data_cloud);


    //TODO Loop pixels in parallel
     double t = (double)getTickCount();


/** //Loop in parallel

const int SIZECOL = graycols -1;
    for (int j=0; j<grayrows; j++) {

//        const Parallel_Scan_Pix_row body(&bgSubImageGraySmall, &imgsmall, &imgsmallHSV,
//                                         &data_cloud, graystep,grayelemsize,cstep,celemsize,HSVstep,HSVelemsize,
//                                         j, colorRegScale, resolutionFractionMultiplier );
//        const cv::Range range(0, SIZECOL);

//        cv::parallel_for_(range, body);
        cv::parallel_for_(Range(0, SIZECOL), Parallel_Scan_Pix_row(&bgSubImageGraySmall, &imgsmall, &imgsmallHSV,
                                                                        &senddata_cloud, graystep,grayelemsize,cstep,celemsize,HSVstep,HSVelemsize,
                                                                   j, colorRegScale, resolutionFractionMultiplier));


    }
    t = ((double)getTickCount() - t)/getTickFrequency();
    qDebug() << "::: Parallel Scan time " << t << endl;
/**/

//Original
    /**/

   t = (double)getTickCount();

    for (int y=0; y < grayrows; y++)
    {
        for (int x=0; x < graycols; x++)
        {
            pixval=aPixel(uchar,bgSubImageGraySmall.data,graystep,grayelemsize,x,y,0);
            pixvalcolorB=aPixel(uchar,imgsmall.data,cstep,celemsize,x,y,0);
            pixvalcolorG=aPixel(uchar,imgsmall.data,cstep,celemsize,x,y,1);
            pixvalcolorR=aPixel(uchar,imgsmall.data,cstep,celemsize,x,y,2);
//            pixvalGray=aPixel(uchar, imgsmallgray.data,gstep,gelemsize,x,y,0);
            pixvalHue=aPixel(uchar, imgsmallHSV.data,HSVstep,HSVelemsize,x,y,0);



            // qDebug()<<"RGB  "<<pixvalcolorR <<"  "<<pixvalcolorG;



            //     pixval=aPixel(uchar,bgSubImageGraySmall.data,bgSubImageGraySmall.step,bgSubImageGraySmall.elemSize(),x,y,1);
            if (pixval > 2)
            {
                temp_data_cloud.push_back(pcl::PointXYZRGB(pixvalcolorR,pixvalcolorG,pixvalcolorB));
                temp_data_cloud.back().x=                                              x*resolutionFractionMultiplier;
                temp_data_cloud.back().y=                                              y*resolutionFractionMultiplier;
                temp_data_cloud.back().z=   pixvalHue*colorRegScale; //0;


            }
        }
    }


    data_cloud = temp_data_cloud;

    t = ((double)getTickCount() - t)/getTickFrequency();
    qDebug() << "::: Serial  Scan time " << t << endl;

    /**/

//    viewer->updatePointCloud(data_cloud_PTR,"datacloud");

    //Simple way to keep track of tracking progress within a frame
    numDetectionsinFrame =     data_cloud.size();



    /// End Big Loop
    qDebug()<<"!!!!!!!!!ICP update Tracks!!!!!!!!!!!!";
    qDebug()<<"Total data_cloud  pts "<<data_cloud.size()<<"  totalDatacloudpts "<<temp_data_cloud.size();
    qDebug()<<" data_cloud Before track removal "<<numDetectionsinFrame;


}


/*
  This is the ICPTracker's main action function
  it
  0) Checks for keyframes and adds tracks there, Removes nearby detections
  1) Updates pre-existing tracks, removes nearby detections
  2) Adds new tracks to remaining detections
  3) Deletes tracks that have died

  */

void ICPTracker::processFrame(Mat scene_img, int timeIndex)


{
    Track* track;
    vector<Model> currentTrackModelPoints;
    currentTrackModelPoints = models;


    frameIndex = timeIndex;

    ///////////////////////////
    /// Prep the current Frame
    //get current frame to track
    ////////////////////////////



    /////////////////
    // background subtraction
    ////////////////
    absdiff(scene_img, bgImage,bgSubImage);

    cvtColor(bgSubImage,bgSubImageGray, CV_BGR2GRAY);// BGR -> gray //NOTE!!!! Never do a cvtColor(img,img, CVBGR2GRAY). if src and dst are same you get ERRORS!
    cv::threshold(bgSubImageGray,bgSubImageGray,bgsubstractionThreshold,255,CV_THRESH_BINARY);


    /// Draw contours
    /// If we want to track just the outlines of the detections, let's give it a go!
//    if(isContourTracking){
//    //    bgSubImage = runContourDetection(bgSubImageGray);//TODO Weird i used to just have bgSubImage
//    }

    /*Mask  */
    if(maskImageGray.empty()){//Don't care about mask
    }    else{//Only mix in mask image if it exists
        cv::bitwise_and(bgSubImageGray,maskImageGray,bgSubImageGray); //One way of applying a mask (Modelmaker has an alternative)
    }

    ////////////////////////////////////////////////////
    ///TODO add user-controllable function for pre-filtering images (like blurring and stuff)
    /*additional Filters*/
    /////////////////////////////////////////////////

  //  Dilation(0,uiICP.dilatespinbox->value(),bgSubImageGray);
   // blur(scene_img,scene_img,cv::Size(uiICP.blurspinBox->value(),uiICP.blurspinBox->value()));


    /// examine our subtraction to make sure we are doing things correctly
    //    namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
    //    imshow( "Gray image", gray_image );

   // end bg subtraction

    //Load the "detection" pixels into a point cloud
    MattoCloudDetections(scene_img);

    ////////////////
    /// 0) TODO check for keyframes for this frame
    //////////////////

    /////////////////////////////////////////////////////////////////
    // 1) Use ICP to update current tracks
    //      precondition: data_cloud (data points) vector is populated for the current frame
    //      postcondition: tracks vector is updated for the current frame
    /////////////////////////////////////////////////////////////////


    // for each track, update its transform and remove closest data points from detection
    for (uint i=0; i < activeTracks.size(); i++)
    {

        currentTrackModelPoints = models;

        data_cloud=  activeTracks[i]->updatePosition(data_cloud,currentTrackModelPoints, modelTOdataThreshold, separationThreshold,icpMatchDistanceThreshold, Ticp_maxIter, Ticp_transformationEpsilon, Ticp_euclideanDistance, uiICP.icp_maxWorstScore->value());

    }

    qDebug()<<" data cloud Pts left over after existing tracks have removed data "<<data_cloud.size();



    //  viewer->spinOnce(100);



    /////////////////////////
    // 2) add new tracks
    /////////////////////////

    //  dpoints associated with tracks were removed in previous update step
    //  Now we will add new tracks for remaining points
    //  Each point gets assigned a track
    for (uint i=0; i < data_cloud.size()-1 -(fewestNumofModelPoints * .01 * modelTOdataThreshold) ; i++) //Assumption, can't have a model birthed if there are fewer points than deemed birthable by even the smallest model   //Assumption Won't have a model with a single point
    {
        if(data_cloud.size()<1) //This seems redundant, TODO remove
        {
            break;
        }

        currentTrackModelPoints = models;
        PointXYZRGB newcloudtargetpoint = data_cloud.points[i];
        newcloudtargetpoint.z = 0; // Keep our translations in the 2D spatial plane (z is reserved for color)
        track = new Track(timeIndex, newcloudtargetpoint, numOfTracks, icpMatchDistanceThreshold,separationThreshold, resolutionFractionMultiplier); // todo: add extra parameter for initial centroid,, MAGIC NUMBER! !! 2/3 is an arbitrary threshold!!!!!
        data_cloud=  track->updatePosition(data_cloud,currentTrackModelPoints, modelTOdataThreshold, separationThreshold, icpMatchDistanceThreshold,Ticp_maxIter, Ticp_transformationEpsilon, Ticp_euclideanDistance, uiICP.icp_maxWorstScore->value());
        if(track->wasBirthed())
        {
            activeTracks.push_back(track);
            numOfTracks++;
        }
        else
        {
            delete track;
        }
    }

    qDebug()<<" data_cloud after adding new tracks "<<data_cloud.size();


    /////////////////////////
    // 3) delete old tracks
    /////////////////////////
    // Iterate over tracks that were not updated this round and kill them
    // Why would a track not be updated? If the number of continuous zombie frames
    // is greater than a threshold.
    for (uint i=0; i < activeTracks.size(); i++)
    {
        if (activeTracks[i]->getNumberOfContinuousZombieFrames() > trackDeathThreshold)
        {
            activeTracks[i]->end();

            // push to inActiveTracks
            inActiveTracks.push_back(activeTracks[i]);

            // erase from activeTracks
            activeTracks.erase(activeTracks.begin() + i);
        }
    }
    /// End Delete old Tracks



    data_cloud.clear();

        //Show what the computer actually sees (at 1/resolution)
        //!!!! Change to have user control size of screen!)
        if (uiICP.display_pushButton->isChecked())
        {
            cvtColor(bgSubImageGray,bgSubImage,CV_GRAY2BGR);
        double t = (double)getTickCount();
            drawTrackResult(bgSubImage);
            t = ((double)getTickCount() - t)/getTickFrequency();
            qDebug() << "Draw Track Result time " << t << endl;

        }




    return;
}






/*

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}
  */



template<typename T>
void ICPTracker::alphaBlendBGRA(const Mat& src1, const Mat& src2, Mat& dst)
{
    const float alpha_scale = (float)std::numeric_limits<T>::max(),
            inv_scale = 1.f/alpha_scale;

    CV_Assert( src1.type() == src2.type() &&
               src1.type() == CV_MAKETYPE(DataType<T>::depth, 4) &&
               src1.size() == src2.size());
    Size size = src1.size();
    dst.create(size, src1.type());

    // here is the idiom: check the arrays for continuity and,
    // if this is the case,
    // treat the arrays as 1D vectors
    if( src1.isContinuous() && src2.isContinuous() && dst.isContinuous() )
    {
        size.width *= size.height;
        size.height = 1;
    }
    size.width *= 4;

    for( int i = 0; i < size.height; i++ )
    {
        // when the arrays are continuous,
        // the outer loop is executed only once
        const T* ptr1 = src1.ptr<T>(i);
        const T* ptr2 = src2.ptr<T>(i);
        T* dptr = dst.ptr<T>(i);

        for( int j = 0; j < size.width; j += 4 )
        {
            float alpha = ptr1[j+3]*inv_scale, beta = ptr2[j+3]*inv_scale;
            // float alpha = 1.0, beta =1.0;

            dptr[j] = saturate_cast<T>(ptr1[j]*alpha + ptr2[j]*beta);
            dptr[j+1] = saturate_cast<T>(ptr1[j+1]*alpha + ptr2[j+1]*beta);
            dptr[j+2] = saturate_cast<T>(ptr1[j+2]*alpha + ptr2[j+2]*beta);
            dptr[j+3] = saturate_cast<T>((1 - (1-alpha)*(1-beta))*alpha_scale);
        }
    }
}


/**
 * Visualization of the tracking result
 */
void ICPTracker::drawTrackResult(Mat img)
{

    int font = FONT_HERSHEY_PLAIN;
    double fontscale=1.5;
    //    int thickness = 40;
    char label[10];
    trackResultImage =img.clone();


    Mat trackResultAlpha;
    cvtColor(trackResultImage,trackResultAlpha,CV_BGR2BGRA);
    cvtColor(trackResultImage,trackResultImage,CV_BGR2BGRA);

    if(!uiICP.subtractioncheckBox->isChecked()){
trackResultImage.setTo(Scalar(0,0,255,0));
    }



    Model model_cloudtodraw;


    int trackLineLength = 300;
    int frameIndex;
    Point previousPosition;
    Point currentPosition;

    qDebug()<<"*****Amount of active Tracks  "<< activeTracks.size();


    ///////////////
    ///Loop Through All Tracks
    /////////////

    for (uint i=0; i < activeTracks.size(); i++)//Temp test
        // for (int i=0; i < 1; i++)//Temp test

    {
        //Start off with a fresh copy of the modelcloud
        model_cloudtodraw = models[activeTracks[i]->modelIndex];



        activeTracks[i]->getTemplatePoints(model_cloudtodraw.cloud); //translates the points

        //  qDebug()<<"ModelCloud Size for drawing after getTemplate"<<model_cloudtodraw.size();

        ///////////////
        ///Draw Track Trails
        /////////////
        if(showTrails){
            // draw trailing breadcrumbs using a red line;
            previousPosition.x = activeTracks[i]->getX();
            previousPosition.y = activeTracks[i]->getY();



            for (int k=1; k < trackLineLength; k++)
            {
                frameIndex = activeTracks[i]->getFrameIndex() - k;
                if (activeTracks[i]->getBirthFrameIndex()+1 > frameIndex)
                {
                    frameIndex = activeTracks[i]->getBirthFrameIndex()+1;
                }

                //frameIndex = max(activeTracks[i]->getBirthFrameIndex()+1,frameIndex);
                currentPosition.x = activeTracks[i]->getX(frameIndex);
                currentPosition.y = activeTracks[i]->getY(frameIndex);

                //Line shrinks over time, different colors are given to different ants
                cv::line(trackResultImage,currentPosition,previousPosition,Scalar(255+10*(-i),250/activeTracks.size()*i,300/activeTracks.size()*(2-i),240*(trackLineLength-k)/trackLineLength),3*(trackLineLength-k)/trackLineLength);


                previousPosition = currentPosition;

            }
        }

        ///////////////
        ///Draw Bounding Box
        //////////////
        if(showBox){
            RotatedRect boundRect( RotatedRect(Point2f(activeTracks[i]->getX(),activeTracks[i]->getY()),
                                               Size(models.at(activeTracks[i]->modelIndex).width,models.at(activeTracks[i]->modelIndex).height),
                                               activeTracks[i]->getRotationAngle() * 180 / 3.1415926

                                               ));

            //TODO :if rects are flipped 180, then draw the blue line on the other side

            Point2f rect_points[4]; boundRect.points( rect_points );
            for( int j = 0; j < 4; j++ )



                if(j==2){
                    //Right Side (head) of box
                    line( trackResultImage, rect_points[j], rect_points[(j+1)%4], Scalar(250,200,0,255),3 , 8 );

                }
                else{
                    line( trackResultImage, rect_points[j], rect_points[(j+1)%4], Scalar(0,200,200,255),3 , 8 );
                }


            //Draw Optional Bounding Ellipse
            //       cv::ellipse(trackResultImage, boundRect,
            //                     CV_BGR(0,255,255),3  );

        }
        ///////////////
        ///Draw Detection Cirlce
        /////////////
        if(showSearchRadius){
            cv::circle(trackResultImage,Point(activeTracks[i]->getX(),activeTracks[i]->getY()),(icpMatchDistanceThreshold),Scalar(255,255,255,150),3);
        }
        ///////////////
        ///Draw ICP Model Templates
        /////////////
        if(showModel){
            int green = 255;
            int red = 0;
            //Zombie flag  --  colorize models if they go into ZOMBIE mode
            if (activeTracks[i]->getNumberOfContinuousZombieFrames() > 2)
            {
                red = 40 * activeTracks[i]->getNumberOfContinuousZombieFrames();
                green = 255 - 40 * activeTracks[i]->getNumberOfContinuousZombieFrames();
            }
            int circleradius =1;
            if(isContourTracking){
                circleradius =3;
            }
            // draw template
            for (uint j=0; j < model_cloudtodraw.cloud.size(); j++)
            {
                //cvSet2D(trackResultImage, pts[j].y, pts[j].x, CV_BGR(255,255,255));

                ///////////////
                ///Draw Point Removal Area
                /////////////

                if(showRemovalRadii){
                    cv::circle(trackResultImage,Point(model_cloudtodraw.cloud.points[j].x, model_cloudtodraw.cloud.points[j].y),separationThreshold,Scalar(70,70,70,100),1);
                }
                //Made model colored green for better visibility
                cv::circle(trackResultImage,Point(model_cloudtodraw.cloud.points[j].x, model_cloudtodraw.cloud.points[j].y),circleradius,Scalar(red,green,2,200),-1);
            }
        }//End Draw ICP Template

        ///////////////
        ///Draw ID Label Text
        //////////////

        if(uiICP.textCheckBox->isChecked()){
        sprintf(label, "%d", activeTracks[i]->getID());
        //        cv::putText(trackResultImage, label, Point(activeTracks[i]->getX(),activeTracks[i]->getY()), font, fontscale, Scalar(255,0,0,255));

        //Write the name of the model used
        cv::putText(trackResultImage, models[activeTracks[i]->modelIndex].name.toStdString()+" ("+label+")", Point(activeTracks[i]->getX(),activeTracks[i]->getY()), font, fontscale+1, Scalar(255,0,255,255), 2);
}
    }


    Mat qImgARGB;
    cvtColor(trackResultImage,qImgARGB,CV_BGRA2RGBA);

    trackResultImage = qImgARGB;
    return;
}

/**
  Takes in a path to a folder, and loads all the png "model" files into Mats

  **/
vector<Model> ICPTracker::modelFilesToMAT(QString modelFolderPath){

    QDir myDir( modelFolderPath);
    QStringList filters;
    filters<<"*.png";
    myDir.setNameFilters(filters);

    vector<Model> output;


    //    int index=0;
    //    QRegExp rx("_.*.btf"); //Todo get just PNG files

    QStringList list = myDir.entryList(filters);
    for (QStringList::iterator it = list.begin(); it != list.end(); ++it) {
        Model imgAndPath;
        QString fullpath;
        QString current = *it;

        qDebug()<<current<<" current modelFile   "<<current;
        fullpath = modelFolderPath+"/"+current;
        Mat modelImg;
        modelImg =imread(fullpath.toStdString(),-1); //flags of zero means force grayscale load
        imgAndPath.img = modelImg;
        imgAndPath.name=current;
        imgAndPath.filepath=fullpath;
        imgAndPath.rotated = 0.0;

//        namedWindow("0");
//        imshow("0", modelImg);

        output.push_back(imgAndPath);


        //Now make a model version that is the same just flipped 180 degrees
        Model imgAndPathF;
        QString fullpathF;
        QString currentF = *it;

        qDebug()<<currentF<<" current modelFile Flipped   "<<current;
        fullpathF = modelFolderPath+"/"+current;
        Mat modelImgF;
        modelImgF =imread(fullpathF.toStdString(),-1); //flags of zero means force grayscale load
        //Rotate the whole thing 180Degrees
        modelImgF = rotateImage(modelImgF, 180.0);//Rotate full image about this center 180 degrees

//        namedWindow("180");
//        imshow("180", modelImgF);

        imgAndPathF.img = modelImgF;
        imgAndPathF.name=current+"_180";
        imgAndPathF.filepath=fullpathF;
        imgAndPathF.rotated=180.0;

        output.push_back(imgAndPathF);



    }

    return output;


}
/**
 * Given an alpha-segmented image for a model target,
 * extract and store the x,y coordinates in a pointcloud
 *
 */
Model ICPTracker::loadModelPoints(Model modelBGRAimg)
{

    /**
      Models contain spatial information coded in the Alpha channel
      right now they are set to
        //Set Head to alpha=240
        //Set Center to Alpha = 250
        //BG subtracted body mask = 255

        ...All spatial information should be >200


        //Everything inside rough mask == alpha 200
        //Everything outside rough mask alpha=100;



      **/


    Model outputModelWCloud = modelBGRAimg;
    Point coordinate;
    Mat imgBGRAsmall;
    cv::resize(modelBGRAimg.img,imgBGRAsmall,Size(),1./resolutionFractionMultiplier,1./resolutionFractionMultiplier, INTER_NEAREST);
    Point modelDimensions;
    modelDimensions.x = imgBGRAsmall.cols*resolutionFractionMultiplier;
    modelDimensions.y = imgBGRAsmall.rows*resolutionFractionMultiplier;
    Mat imgBGR;
    cvtColor(imgBGRAsmall, imgBGR,CV_BGRA2BGR); //Drop Alpha
    Mat imgHSV;

    cvtColor(imgBGR, imgHSV,CV_BGR2HSV); //Get HSV

    pcl::PointCloud<pcl::PointXYZ> loadmodel_cloud;// (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB> loadmodelXYZRGB_cloud;

    int minX=DBL_MAX;
    int maxX=0;
    int minY=DBL_MAX;
    int maxY=0;

    ///// New type of looping
    int irows=imgBGRAsmall.rows; // number of lines
    int icols = imgBGRAsmall.cols; // number of columns

    int BGRAistep = imgBGRAsmall.step;
    int BGRAielemsize = imgBGRAsmall.elemSize();

    int HSVistep = imgHSV.step;
    int HSVielemsize= imgHSV.elemSize();

    uchar pixvalAlpha = 0;
    uchar pixvalcolorR = 0;
    uchar pixvalcolorG = 0;
    uchar pixvalcolorB = 0;
    uchar pixvalGray = 0;
    uchar pixvalHue = 0;


    //Functions //Fancy Define
#define aPixel(type, dataStart,step,size,x,y,channel)*((type*)(dataStart+step*(y)+(x)*size+channel)) // This says it is fast (maybe a lie)

    colorRegScale=uiICP.colorRegSpinBox->value();
    pcl::PointCloud<pcl::PointXYZRGB> temp_model_cloud;

    for (int y=0; y < irows; y++)
    {
        for (int x=0; x < icols; x++)
        {
            //           pixvalAlpha = imgBGRAsmall.at<Vec4b>(y,x)[3];//Alpha Channel gives information about target location, everything else is just random

            pixvalAlpha=aPixel(uchar,imgBGRAsmall.data,BGRAistep,BGRAielemsize,x,y,3);
            pixvalcolorB=aPixel(uchar,imgBGRAsmall.data,BGRAistep,BGRAielemsize,x,y,0);
            pixvalcolorG=aPixel(uchar,imgBGRAsmall.data,BGRAistep,BGRAielemsize,x,y,1);
            pixvalcolorR=aPixel(uchar,imgBGRAsmall.data,BGRAistep,BGRAielemsize,x,y,2);
            //           pixvalGray=aPixel(uchar, imgsmallgray.data,gistep,gielemsize,x,y,1);
            pixvalHue=aPixel(uchar, imgHSV.data,HSVistep,HSVielemsize,x,y,0);



            // qDebug()<<"RGB  "<<pixvalcolorR <<"  "<<pixvalcolorG;



            //     pixval=aPixel(uchar,bgSubImageGraySmall.data,bgSubImageGraySmall.step,bgSubImageGraySmall.elemSize(),x,y,1);
            if (pixvalAlpha > 200)
            {

                coordinate.x = x*resolutionFractionMultiplier - modelDimensions.x/2; // center //scale back into full res coordinates
                coordinate.y = y*resolutionFractionMultiplier - modelDimensions.y/2; // center

                temp_model_cloud.push_back(pcl::PointXYZRGB(pixvalcolorR,pixvalcolorG,pixvalcolorB));
                temp_model_cloud.back().x=                                          x*resolutionFractionMultiplier - modelDimensions.x/2;
                temp_model_cloud.back().y=                                       y*resolutionFractionMultiplier - modelDimensions.y/2;
                temp_model_cloud.back().z=   pixvalHue*colorRegScale; //0;

                //Figure out dimensions
                if(coordinate.x<minX) minX=coordinate.x;
                if(coordinate.y<minY) minY=coordinate.y;
                if(coordinate.x>maxX) maxX=coordinate.x;
                if(coordinate.y>maxY) maxY=coordinate.y;


            }
        }
    }

    if(fewestNumofModelPoints>temp_model_cloud.size()) fewestNumofModelPoints=temp_model_cloud.size();

    if(largestNumofModelPoints<temp_model_cloud.size()) largestNumofModelPoints = temp_model_cloud.size();

    qDebug()<<"fewestNumofModel "<<fewestNumofModelPoints<< "  largest  "<<largestNumofModelPoints;
    outputModelWCloud.cloud=temp_model_cloud;



    //TODO, store model width and height with the super vector
    int trueModelwidth;
    int trueModelheight;
    trueModelwidth=maxX;
    if(maxX<abs(minX)) trueModelwidth=minX;

    trueModelheight=maxY;
    if(maxY<abs(minY)) trueModelheight=minY;

    trueModelwidth=abs(trueModelwidth*2);
    trueModelheight=abs(trueModelheight*2);
    qDebug()<<" modx width "<<trueModelwidth<<"   modHeight "<<trueModelheight;
    outputModelWCloud.width=trueModelwidth;
    outputModelWCloud.height=trueModelheight;


    int maxModelDimension;
    maxModelDimension = trueModelwidth > trueModelheight ? trueModelwidth : trueModelheight; //new method uses actual image image shape
    outputModelWCloud.maxDimension=maxModelDimension;



    return outputModelWCloud;

}

/**
 * Given a folder of BGRA images for a model target,
 * extract and store the x,y coordinates in a
 * vector of "model" structures.
 *
 */
std::vector<Model> ICPTracker::loadModelClouds(QString mPath)
{
    fewestNumofModelPoints=DBL_MAX;
    largestNumofModelPoints = 0;

    vector<Model> BGRA_Name_and_Paths;
    BGRA_Name_and_Paths =  modelFilesToMAT(mPath);


    if(isContourTracking){//todo fix! This program runs the loader before loading the settings or something, and can switch contourtracking on!
        // imgAlpha = runContourDetection(imgAlpha);
    }

    //Load the data for each model
    for(int i; i<BGRA_Name_and_Paths.size();i++){
        BGRA_Name_and_Paths.at(i) = loadModelPoints(BGRA_Name_and_Paths.at(i)); //Adds cloud and dimensional data to the Model
    }

    return BGRA_Name_and_Paths;
}


/**
 * Iterate over all tracks, active and inactive, and
 * output the TeamView format
 **/
void ICPTracker::outputBTF(QString projectdirectory,QString icpprojectname)
{


    QDir btfPath;
    btfPath.mkpath(projectdirectory+icpprojectname);

    Track* track;

    QString tsname=projectdirectory+icpprojectname+"/"+"timestamp.btf";
    QString tsname_f=projectdirectory+icpprojectname+"/"+"timestamp_frames.btf";

    QString modelname=projectdirectory+icpprojectname+"/"+"type.btf";

    QString idname=projectdirectory+icpprojectname+"/"+"id.btf";
    QString ximagename=projectdirectory+icpprojectname+"/"+"ximage.btf";
    QString yimagename=projectdirectory+icpprojectname+"/"+"yimage.btf";
    QString timagename=projectdirectory+icpprojectname+"/"+"timage.btf";


    FILE* timeStampFP = fopen(tsname.toAscii(),"w");
    FILE* timeStampFP_f = fopen(tsname_f.toAscii(),"w");

    FILE* xImageFP = fopen(ximagename.toAscii(),"w");
    FILE* yImageFP = fopen(yimagename.toAscii(),"w");
    FILE* angleFP = fopen(timagename.toAscii(),"w");
    FILE* idFP = fopen(idname.toAscii(),"w");
    FILE* typeFP = fopen(modelname.toAscii(),"w");

    int startFrame = 0;
    int endFrame = 0;
    typedef std::multimap<int, Track*> mapType;

    mapType framesToTracksMAP;

    // We can either iterate over each time step and gather the
    // tracks that were active during that frame, or we can iterate
    // over tracks and gather the state into time steps.
    //
    // If we do the latter, then we can create a new data structure
    // which slots state into each frame.  After constructing this,
    // we can iterate back over the frames and report state.  This
    // is a bit inefficient, however, as we are essentially doing
    // two passes when only one is necessary.
    //
    // What would be the best data structure to handle temporal
    // intervals like this?  Perhaps just a hash table of
    // lists of pointers to tracks.

    for (uint i=0; i < activeTracks.size()+inActiveTracks.size(); i++)
    {
        if (i >= inActiveTracks.size())
        {
            // active track
            track = activeTracks[i-inActiveTracks.size()];
        }
        else
        {
            // inactive track
            track = inActiveTracks[i];
        }

        if (track->getLength() < 10) continue; // ignore this as an outlier //TODO remove this magic Number
        // get frame indices in video clip's reference frame
        startFrame = track->getBirthFrameIndex();
        endFrame = track->getBirthFrameIndex() + track->getLength();

        for (uint j=startFrame; j < endFrame; j++)
        {
            framesToTracksMAP.insert(std::make_pair(j, track));

        } // loop over track's frames

    }

    // iterate over the frames to tracks map and write files

    int fIndex = 0;
    int x, y;
    float angle;
    int timestamp;
    int id;
    String type;
    for(mapType::const_iterator it = framesToTracksMAP.begin(); it != framesToTracksMAP.end(); ++it)
    {
        fIndex = (*it).first;
        track = (*it).second;

        //1000
        timestamp = (1000/(thefps))*fIndex;
        id = track->getID();
        fprintf(timeStampFP, "%d\n", timestamp);
        fprintf(timeStampFP_f, "%d\n", fIndex); //Fix this: determine if needs to be fIndex+1 or just fIndex

        type =  models[track->modelIndex].name.toStdString();
        fprintf(typeFP, "%s\n", type.c_str());

        fprintf(idFP, "%d\n", id);
        x = track->getX(fIndex+1);
        y = track->getY(fIndex+1);

        //Some have been flipped by 180degrees, if that is the case, unflip them!
//        if(models[track->modelIndex].rotated)
        angle = (float)(models[track->modelIndex].rotated-track->getRotationAngle(fIndex+1));
        fprintf(xImageFP, "%d\n", x);
        fprintf(yImageFP, "%d\n", y);
        fprintf(angleFP, "%f\n", angle);
    }

    // close files
    fclose(timeStampFP);
    fclose(timeStampFP_f);

    fclose(xImageFP);
    fclose(yImageFP);
    fclose(angleFP);
    fclose(typeFP);

    fclose(idFP);

    return;
}

/**
 * Iterate over all tracks, active and inactive, and
 * perform the following analysis tasks:
 * 1) Merge tracks which are highly correlated to overcome
 *    current bug where ICP drifts, loses target between merge
 *    and split, resulting in a new track.
 * 2) Eliminate tracks with a short life
 * 3) Output (idA, idB, startFrame, endFrame) in an HTML file to
 *    report proximity interactions.
 **/
void ICPTracker::outputInteractionsReport()
{
    Track* ta;
    Track* tb;
    FILE* fp = fopen("interaction_report.html","w");
    fprintf(fp,"<html><title>Manytrack Interactions Report</title>\n");
    fprintf(fp,"<body><font face=\"arial, helvetica\"><table border=\"1\" >\n");
    fprintf(fp,"<tr><td><b>Ant 1 ID</b></td><td><b>Ant 2 ID</b></td><td><b>Interaction start frame</b></td><td><b>Interaction end frame</b></td></tr>\n");

    // iterate over all tracks, both active and inactive
    for (uint i=0; i < activeTracks.size()+inActiveTracks.size(); i++)
    {
        // Inactive tracks will have ended already and active tracks
        // will be present at the last frame.  When a track ends, it
        // no longer is part of the activeTracks collection.
        if (i >= inActiveTracks.size())
        {
            // active track
            ta = activeTracks[i-inActiveTracks.size()];
        }
        else
        {
            // inactive track
            ta = inActiveTracks[i];
        }
        // iterate over upper triangle
        for (uint j=i+1; j < activeTracks.size()+inActiveTracks.size(); j++)
        {
            if (j >= inActiveTracks.size())
            {
                // active track
                tb = activeTracks[j-inActiveTracks.size()];
            }
            else
            {
                // inactive track
                tb = inActiveTracks[j];
            }

            // Process track pair if they overlap in time
            processInteractions(ta,tb,fp);

        }
    }

    fprintf(fp,"</table></font></body>\n");
    fprintf(fp,"</html>\n");
    fclose(fp);

    return;
}

/**
 * Given pointers to a pair of tracks, determine for which frames
 * they come into close proximity with each other.  Close proximity
 * counts as an interaction between the tracks.  If the interaction
 * time length is a very large percentage of the total track length,
 * then the track probably belongs to the same object as the other
 * track and it doesn't count as an interaction.  Also, if one of the
 * tracks is very short, do not count it at all.
 *
 * TODO: Output a thumbnail of the interaction
 */
void ICPTracker::processInteractions(Track* ta, Track* tb, FILE* fp)
{
    double distance;
    int interactionLength;
    int interactionStartFrame;
    Point aPos;
    Point bPos;
    int distanceThreshold = 30;
    int minInteractionLengthToBeValid = 10;
    int startFrameOfOverlap = 0;
    int endFrameOfOverlap = 0;

    if (ta->getLength() < 10) return;
    if (tb->getLength() < 10) return;

    // iterate over the overlap range between two tracks
    if (ta->getBirthFrameIndex() > tb->getBirthFrameIndex())
        startFrameOfOverlap = ta->getBirthFrameIndex();
    else
        startFrameOfOverlap = tb->getBirthFrameIndex();

    //startFrameOfOverlap = max(ta->getBirthFrameIndex(),tb->getBirthFrameIndex());
    if (ta->getBirthFrameIndex()+ta->getLength() < tb->getBirthFrameIndex()+tb->getLength())
        endFrameOfOverlap = ta->getBirthFrameIndex()+ta->getLength();
    else
        //changed something
        endFrameOfOverlap=		tb->getBirthFrameIndex() + tb->getLength();

    //endFrameOfOverlap = min(ta->getBirthFrameIndex()+ta->getLength(),tb->getBirthFrameIndex()+tb->getLength());
    if (endFrameOfOverlap-startFrameOfOverlap < minInteractionLengthToBeValid) return;

    interactionLength = 0;
    interactionStartFrame = 0;

    for (int i=startFrameOfOverlap; i < endFrameOfOverlap; i++)
    {
        aPos.x = ta->getX(i+1)*4;
        aPos.y = ta->getY(i+1)*4;

        bPos.x = tb->getX(i+1)*4;
        bPos.y = tb->getY(i+1)*4;

        // compute distance between targets
        distance = powf(aPos.x-bPos.x,2)+powf(aPos.y-bPos.y,2);
        if (distance < powf(distanceThreshold,2))
        {
            if (interactionStartFrame == 0)
            {
                interactionStartFrame = i+1;
            }
            interactionLength++;
        }
        else
        {
            // Report the interaction if time length is long enough.
            if (interactionLength > minInteractionLengthToBeValid)
            {
                fprintf(fp,"<tr><td>%d</td><td>%d</td><td>%d</td><td>%d</td></tr>\n", ta->getID(), tb->getID(), interactionStartFrame, interactionStartFrame+interactionLength);
                interactionLength = 0;
                interactionStartFrame = 0;
            }
        }
    } // loop over overlap range

    return;
}



Mat ICPTracker::runContourDetection(Mat img){

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;


    /// Detect edges using canny
    /// Find contours
    findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    Scalar color = Scalar( 255, 255, 255 );
    int thickness = 1; // FIX magic number
    /// Draw contours
    Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
    for( uint i = 0; i< contours.size(); i++ )
    {

        drawContours( drawing, contours, i, color, thickness, 8, hierarchy, 0, Point() );
    }


    //    /// Show in a window
    //    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    //    imshow( "Contours", drawing );

    return drawing;


}


//rotates an image about center
cv::Mat ICPTracker::rotateImage(const Mat& source, double angle)
{
//    qDebug()<<"Angle in rad"<<anglerad;
//    double angle  = ((anglerad*180)/CV_PI);
    qDebug()<<"Angle in deg"<<angle;
    Point2f src_center(source.cols/2.0F, source.rows/2.0F);
    Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
    Mat dst;
    warpAffine(source, dst, rot_mat, source.size());
    return dst;
}



/** @function Dilation */
void ICPTracker::Dilation( int dilation_elem, int dilation_size, Mat &src )
{

  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  erode( src, src, element );
//  imshow( "Dilation Demo", src );
}



