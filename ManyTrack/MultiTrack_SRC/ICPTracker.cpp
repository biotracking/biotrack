#include "ICPTracker.h"



void
viewerOneOff (pcl::visualization::PCLVisualizer& viewerT)
{
//    pcl:: PointCloud<PointXYZRGB>::Ptr data_cloud_PTR (new pcl::PointCloud<PointXYZRGB> (data_cloud));

    viewerT.setBackgroundColor (1.0, 0.5, 1.0);
//    viewerT->updatePointCloud(data_cloud_PTR,"datacloud");

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


ICPTracker::ICPTracker(float fps, QString bgImagepath, QString modelFolderpath, QString maskImagepath, Ui::MultitrackClass uiPass)
{


    mFolderPath = modelFolderpath;
    uiICP=uiPass;
    thefps=fps;

    //Set some default values
    showSearchRadius=true;
    showRemovalRadii=true;
    showModel=true;
    showTrails = true;

    bgsubstractionThreshold = 58;
    modelTOdataThreshold = 20; // percentage of model points to be considered healthy


    numOfTracks = 0;
    isVideoShowing = true;
    resolutionFractionMultiplier = 4; // 4 for 1/4 resolution, 2 for 1/2 resolution, 1 for full resolution
    separationThreshold=6;

    // TODO: move into load function
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

    model_clouds_orig = loadModelClouds(mFolderPath  );

    qDebug()<< "Number of Models loaded:" << model_clouds_orig.size();

    trackToBlobDistanceThreshold = 0;
    trackDeathThreshold=20;

    colorRegScale=uiICP.colorRegSpinBox->value();

  /**/  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));

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




void ICPTracker::track(Mat img, int timeIndex)
{



    Track* track;

   vector< pair<PointCloud<pcl::PointXYZRGB>, QString> > currentTrackModelPoints;
    currentTrackModelPoints = model_clouds_orig;



    /////////////////////////
    // update all tracks
    /////////////////////////
    frameIndex = timeIndex;

    ///////////////////////////
    /// Prep the current Frame
    //get current frame to track
    ////////////////////////////




    // bg subtraction

    absdiff(img, bgImage,bgSubImage);

    cvtColor(bgSubImage,bgSubImageGray, CV_BGR2GRAY);// BGR -> gray //NOTE!!!! Never do a cvtColor(img,img, CVBGR2GRAY). if src and dst are same you get ERRORS!
    cv::threshold(bgSubImageGray,bgSubImageGray,bgsubstractionThreshold,255,CV_THRESH_BINARY);


    /// Draw contours
    /// If we want to track just the outlines of the detections, let's give it a go!
    if(isContourTracking){
        bgSubImage = runContourDetection(bgSubImageGray);//TODO Weird i used to just have bgSubImage
    }

    /*Mask  */
    if(maskImageGray.empty()){//Don't care about mask
    }    else{//Only mix in mask image if it exists
        cv::bitwise_and(bgSubImageGray,maskImageGray,bgSubImageGray); //TODO check modeler for alternate method of applying mask
    }

    ////////////////////////////////////////////////////
    ///TODO add user-controllable function for pre-filtering images (like blurring and stuff)
    /*additional Filters*/
    // median filter bgSubImageGray
    //cvSmooth(bgSubImageGray, bgSubImageGray, CV_MEDIAN, 3);
    /////////////////////////////////////////////////

    //Rescale from full size down to our resampled size
    cv::resize(bgSubImageGray, bgSubImageGraySmall, Size(), 1./resolutionFractionMultiplier, 1./resolutionFractionMultiplier);

    Mat imgsmall;
    cv::resize(img, imgsmall, Size(), 1./resolutionFractionMultiplier, 1./resolutionFractionMultiplier); //full color reduced image
    Mat imgsmallgray;
    cvtColor(imgsmall,imgsmallgray,CV_BGR2GRAY);
    Mat imgsmallHSV;
    cvtColor(imgsmall,imgsmallHSV,CV_BGR2HSV);


    /// examine our subtraction to make sure we are doing things correctly
    //    namedWindow( "Gray image", CV_WINDOW_AUTOSIZE );
    //    imshow( "Gray image", gray_image );

    ///////////////
    /// The Big Loop
    /////////////


    ///////////////
    /// Collect Detections into Cloud of Data Points
    /////////////
    // iterate over decimated segmentation and create an array of data points (detections)
    // for each foreground pixel (non black pixels are foreground)


    // cvtColor(bgSubImage,bgSubImageGray, CV_BGR2GRAY);// BGR -> gray
    //Get as many hard coded values as possible before we go through expensive looping!
    int irows=bgSubImageGraySmall.rows; // number of lines
    int icols = bgSubImageGraySmall.cols; // number of columns
    int istep = bgSubImageGraySmall.step;
    int ielemsize= bgSubImageGraySmall.elemSize();

    //Color values
    int cirows=imgsmall.rows; // number of lines
    int cicols = imgsmall.cols; // number of columns
    int cistep = imgsmall.step;
    int cielemsize= imgsmall.elemSize();

    int gistep = imgsmallgray.step;
    int gielemsize= imgsmallgray.elemSize();

    int HSVistep = imgsmallHSV.step;
    int HSVielemsize= imgsmallHSV.elemSize();

    uchar pixval = 0;
    uchar pixvalcolorR = 0;
    uchar pixvalcolorG = 0;
    uchar pixvalcolorB = 0;
    uchar pixvalGray = 0;
    uchar pixvalHue = 0;







    //Functions //Fancy Define
#define aPixel(type, dataStart,step,size,x,y,channel)*((type*)(dataStart+step*(y)+(x)*size+channel)) // This says it is fast (maybe a lie)

    //        data_cloud->width    = pts.size();
    //        data_cloud.height   = 1;
    //        data_cloud.is_dense = false; // should be true?
    //        data_cloud.points.resize (data_cloud.width * data_cloud.height);
    pcl::PointCloud<pcl::PointXYZRGB> temp_data_cloud;

    for (int y=0; y < irows; y++)
    {
        for (int x=0; x < icols; x++)
        {
            pixval=aPixel(uchar,bgSubImageGraySmall.data,istep,ielemsize,x,y,0);
            pixvalcolorB=aPixel(uchar,imgsmall.data,cistep,cielemsize,x,y,0);
            pixvalcolorG=aPixel(uchar,imgsmall.data,cistep,cielemsize,x,y,1);
            pixvalcolorR=aPixel(uchar,imgsmall.data,cistep,cielemsize,x,y,2);
            pixvalGray=aPixel(uchar, imgsmallgray.data,gistep,gielemsize,x,y,0);
            pixvalHue=aPixel(uchar, imgsmallHSV.data,HSVistep,HSVielemsize,x,y,0);



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
    pcl:: PointCloud<PointXYZRGB>::Ptr data_cloud_PTR (new pcl::PointCloud<PointXYZRGB> (data_cloud));

    viewer->updatePointCloud(data_cloud_PTR,"datacloud");



    /// End Big Loop



    /////////////////////////////////////////////////////////////////
    // Use ICP to update current tracks
    // precondition: data_cloud (data points) vector is populated for the current frame
    // postcondition: tracks vector is updated for the current frame
    /////////////////////////////////////////////////////////////////
    qDebug()<<"!!!!!!!!!ICP update Tracks!!!!!!!!!!!!";
    qDebug()<<"Total data_cloud  pts "<<data_cloud.size()<<"  totalDatacloudpts "<<temp_data_cloud.size();
    qDebug()<<" data_cloud Before track removal "<<data_cloud.size();


    // for each track, update its transform and remove closest data points from detection
    for (uint i=0; i < activeTracks.size(); i++)
    {

        currentTrackModelPoints = model_clouds_orig;

        data_cloud=  activeTracks[i]->update(data_cloud,currentTrackModelPoints, modelTOdataThreshold, separationThreshold,trackToBlobDistanceThreshold, Ticp_maxIter, Ticp_transformationEpsilon, Ticp_euclideanDistance);

    }

    qDebug()<<" data cloud Pts left over after existing tracks have removed data "<<data_cloud.size();



      //  viewer->spinOnce(100);



    /////////////////////////
    // add new tracks
    /////////////////////////

    //  dpoints associated with tracks were removed in previous update step
    //  Now we will add new tracks for remaining points
    //  Each point gets assigned a track
    for (uint i=0; i < data_cloud.size()-1 -(model_clouds_orig.size() * .01 * modelTOdataThreshold) ; i++) //Assumption, can't have a model if there are fewer points than deemed birthable anyway    //Assumption Won't have a model with a single point
    {

        if(data_cloud.size()<1) //This seems redundant, TODO remove
        {
            break;
        }

        currentTrackModelPoints = model_clouds_orig;
        track = new Track(timeIndex, data_cloud.points[i], numOfTracks, trackToBlobDistanceThreshold,separationThreshold, resolutionFractionMultiplier); // todo: add extra parameter for initial centroid,, MAGIC NUMBER! !! 2/3 is an arbitrary threshold!!!!!
        data_cloud=  track->update(data_cloud,currentTrackModelPoints, modelTOdataThreshold, separationThreshold, trackToBlobDistanceThreshold,Ticp_maxIter, Ticp_transformationEpsilon, Ticp_euclideanDistance);
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
    // delete old tracks
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
    if (isVideoShowing && uiICP.display_pushButton->isChecked())
    {
        drawTrackResult(img);
    }
    else
    {
        //Show what the computer actually sees (at 1/resolution)
        //!!!! Change to have user control size of screen!)
        cvtColor(bgSubImageGray,bgSubImage,CV_GRAY2BGR);
        if (uiICP.display_pushButton->isChecked())
        {
            drawTrackResult(bgSubImage);
        }
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
    double fontscale=1;
    //    int thickness = 40;
    char label[10];
    trackResultImage =img.clone();
    trackResultImage.setTo(0);
    Mat trackResultAlpha;
    cvtColor(trackResultImage,trackResultAlpha,CV_BGR2BGRA);
    cvtColor(trackResultImage,trackResultImage,CV_BGR2BGRA);
    trackResultImage.setTo(Scalar(0,0,255,0));



    pcl::PointCloud<pcl::PointXYZRGB> model_cloudtodraw;


    int trackLineLength = 200;
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
        model_cloudtodraw = model_clouds_orig[activeTracks[i]->modelIndex].first;



        activeTracks[i]->getTemplatePoints(model_cloudtodraw); //translates the points

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
                cv::line(trackResultImage,currentPosition,previousPosition,Scalar(255+10*(-i),250/activeTracks.size()*i,300/activeTracks.size()*(2-i)),3*(trackLineLength-k)/trackLineLength, CV_AA);
                previousPosition = currentPosition;

            }
        }

        ///////////////
        ///Draw Bounding Box
        //////////////
        if(showBox){
            RotatedRect boundRect( RotatedRect(Point2f(activeTracks[i]->getX(),activeTracks[i]->getY()),
                                               Size(maxModelDimension,maxModelDimension),
                                               activeTracks[i]->getRotationAngle() * 180 / 3.1415926

                                               ));

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
            cv::circle(trackResultImage,Point(activeTracks[i]->getX(),activeTracks[i]->getY()),(trackToBlobDistanceThreshold),Scalar(255,255,255,150),3);
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
            for (uint j=0; j < model_cloudtodraw.size(); j++)
            {
                //cvSet2D(trackResultImage, pts[j].y, pts[j].x, CV_BGR(255,255,255));

                ///////////////
                ///Draw Point Removal Area
                /////////////

                if(showRemovalRadii){
                    cv::circle(trackResultImage,Point(model_cloudtodraw.points[j].x, model_cloudtodraw.points[j].y),separationThreshold,Scalar(70,70,70,100),1);
                }
                //Made model colored green for better visibility
                cv::circle(trackResultImage,Point(model_cloudtodraw.points[j].x, model_cloudtodraw.points[j].y),circleradius,Scalar(red,green,2,200),-1);
            }
        }//End Draw ICP Template

        ///////////////
        ///Draw ID Label Text
        //////////////
        sprintf(label, "%d", activeTracks[i]->getID());
//        cv::putText(trackResultImage, label, Point(activeTracks[i]->getX(),activeTracks[i]->getY()), font, fontscale, Scalar(255,0,0,255));

        //Write the name of the model used
        //NOT WORKING -->
        cv::putText(trackResultImage, model_clouds_orig[activeTracks[i]->modelIndex].second.toStdString()+" ("+label+")", Point(activeTracks[i]->getX(),activeTracks[i]->getY()), font, fontscale+1, Scalar(255,0,0,255));

    }


    Mat qImgARGB;
    cvtColor(trackResultImage,qImgARGB,CV_BGRA2RGBA);




    //Todo find  better way of overlaying images using alpha channels


    Mat imgBGRA;
    cvtColor(img,imgBGRA,CV_BGR2BGRA);
    //    cvtColor(trackResultImage,trackResultImage,CV_BGR2BGRA);

    alphaBlendBGRA<uchar>(imgBGRA,trackResultImage,trackResultImage);
    //    addWeighted(trackResultImage,1,imgBGRA,.5,0,trackResultImage);
    //   add(trackResultImage,imgBGRA, trackResultImage);

    cvtColor(trackResultImage,trackResultImage,CV_BGRA2BGR);

    trackResultImage = qImgARGB;
    return;
}

/**
  Takes in a path to a folder, and loads all the png "model" files into Mats

  **/
vector< pair<Mat, QString> > ICPTracker::modelFilesToMAT(QString modelFolderPath){

    QDir myDir( modelFolderPath);
    QStringList filters;
    filters<<"*.png";
    myDir.setNameFilters(filters);

    vector< pair<Mat, QString> > output;


//    int index=0;
//    QRegExp rx("_.*.btf");

    QStringList list = myDir.entryList(filters);
    for (QStringList::iterator it = list.begin(); it != list.end(); ++it) {
        pair<Mat, QString> modelAndPath;
        QString fullpath;
        QString current = *it;

        qDebug()<<current<<" current modelFile   "<<current;
            fullpath = modelFolderPath+"/"+current;
Mat modelImg;
modelImg =imread(fullpath.toStdString(),-1); //flags of zero means force grayscale load
        modelAndPath.first = modelImg;
        modelAndPath.second=current;
output.push_back(modelAndPath);

            }

return output;


}
/**
 * Given a binary image for a model target,
 * extract and store the x,y coordinates in a
 * vector of pairs.
 *Store these in a pointcloud too!
 *
 */
pcl::PointCloud<pcl::PointXYZRGB> ICPTracker::loadModelPoints(Mat imgBGRA)
{
    Point coordinate;
    Mat imgBGRAsmall;
    cv::resize(imgBGRA,imgBGRAsmall,Size(),1./resolutionFractionMultiplier,1./resolutionFractionMultiplier, INTER_NEAREST);
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
            if (pixvalAlpha > 2)
            {

                coordinate.x = x*resolutionFractionMultiplier - modelDimensions.x/2; // center //scale back into full res coordinates
                coordinate.y = y*resolutionFractionMultiplier - modelDimensions.y/2; // center

                temp_model_cloud.push_back(pcl::PointXYZRGB(pixvalcolorR,pixvalcolorG,pixvalcolorB));
                temp_model_cloud.back().x=                                          x*resolutionFractionMultiplier - modelDimensions.x/2;
                temp_model_cloud.back().y=                                       y*resolutionFractionMultiplier - modelDimensions.y/2;
                temp_model_cloud.back().z=   pixvalHue*colorRegScale; //0;


            }
        }
    }




                    if(coordinate.x<minX) minX=coordinate.x;
                    if(coordinate.y<minY) minY=coordinate.y;
                    if(coordinate.x>maxX) maxX=coordinate.x;
                    if(coordinate.y>maxY) maxY=coordinate.y;




    //   qDebug()<<loadmodelXYZRGB_cloud.back().x<<"   Y "<<loadmodelXYZRGB_cloud.back().y <<"   RGB "<<r <<"   written red "<<loadmodelXYZRGB_cloud.back().r
    //<<"  max X, Max Y, minx , minY "<<maxX<< "  "<<maxY<< "  "<<minX<< "  "<<minY;



    loadmodelXYZRGB_cloud=temp_model_cloud;


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

    //    maxModelDimension = modelImage.rows > modelImage.cols ? modelImage.rows : modelImage.cols; //Old method uses image shape
    maxModelDimension = trueModelwidth > trueModelheight ? trueModelwidth : trueModelheight; //new method uses actual image image shape

    return loadmodelXYZRGB_cloud;

}

/**
 * Given a folder of BGRA images for a model target,
 * extract and store the x,y coordinates in a
 * vector of pairs of Pointclouds and Stringnames.
 *
 */
vector<pair <PointCloud<PointXYZRGB> , QString> > ICPTracker::loadModelClouds(QString mPath)
{

   vector< pair<Mat, QString> > BGRA_plist=  modelFilesToMAT(mPath);

vector<pair <PointCloud<PointXYZRGB> , QString> >  modelClouds;

    if(isContourTracking){//todo fix! This program runs the loader before loading the settings or something, and can switch contourtracking on!
        // imgAlpha = runContourDetection(imgAlpha);
    }

    //Load the correct pixels for each model
    for(int i; i<BGRA_plist.size();i++){
        pair <PointCloud<PointXYZRGB> , QString> cloudAndString;

        cloudAndString.second=BGRA_plist[i].second;
        cloudAndString.first = loadModelPoints(BGRA_plist[i].first);

        modelClouds.push_back(cloudAndString);
    }
    //   return loadmodel_cloud;

//    return loadmodelXYZRGB_cloud;
    return modelClouds;

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
    for(mapType::const_iterator it = framesToTracksMAP.begin(); it != framesToTracksMAP.end(); ++it)
    {
        fIndex = (*it).first;
        track = (*it).second;

        //1000
        timestamp = (1000/(thefps))*fIndex;
        id = track->getID();
        fprintf(timeStampFP, "%d\n", timestamp);
        fprintf(timeStampFP_f, "%d\n", fIndex); //Fix this: determine if needs to be fIndex+1 or just fIndex

        fprintf(idFP, "%d\n", id);
        x = track->getX(fIndex+1);
        y = track->getY(fIndex+1);
        angle = (float)(track->getRotationAngle(fIndex+1));
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
    fprintf(fp,"<html><title>Multitrack Interactions Report</title>\n");
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

    //    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;


    /// Detect edges using canny
    //Canny( img, img, 50, 50*2, 3 );
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


