#include "ICPTracker.h"



ICPTracker::ICPTracker(float fps, QString bgImagepath, QString modelImagepath, QString maskImagepath, Ui::MultitrackClass uiPass)
{

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


    //Create useful copies of other images
    bgSubImage = bgImage.clone();

    trackResultImage = bgImage.clone();
    cv::cvtColor(bgSubImage, bgSubImageGray, CV_BGR2GRAY);

    //NOTE, resize NEEEEEEDS floats in the scale factor or else it FAILS!
    cv::resize(bgSubImageGray,bgSubImageGraySmall,Size(),1./resolutionFractionMultiplier,1./resolutionFractionMultiplier);
    bgSubImageGraySmallScratch.copyTo(bgSubImageGraySmall);

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

    // load model image
    modelImage = imread(modelImagepath.toStdString(),-1); //flags of zero means force grayscale load
    model_cloud_orig = loadModelPoints(modelImage);

    qDebug()<< "Model cloud after load:" << model_cloud_orig.size();
    //qDebug()<< "model cloud new:" << model_cloud_orig.points[1] << " :::model cloud old: ";

    trackToBlobDistanceThreshold = 0;
    trackDeathThreshold=20;

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
    bgSubImageGraySmallScratch.release();
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

    pcl::PointCloud<pcl::PointXYZ> currentTrackModelPoints;
    currentTrackModelPoints = model_cloud_orig;


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

    bgSubImageGraySmall.copyTo(bgSubImageGraySmallScratch);

    ///////////////
    /// The Big Loop
    /////////////


    ///////////////
    /// Collect Detections into Cloud of Data Points
    /////////////
    // iterate over decimated segmentation and create an array of data points (detections)
    // for each foreground pixel (non black pixels are foreground)


    cvtColor(bgSubImage,bgSubImageGray, CV_BGR2GRAY);// BGR -> gray
    //Get as many hard coded values as possible before we go through expensive looping!
    int irows=bgSubImageGraySmall.rows; // number of lines
    int icols = bgSubImageGraySmall.cols; // number of columns
    int istep = bgSubImageGraySmall.step;
    int ielemsize= bgSubImageGraySmall.elemSize();


    uchar pixval = 0;



    //Functions //Fancy Define
#define aPixel(type, dataStart,step,size,x,y,channel)*((type*)(dataStart+step*(y)+(x)*size+channel)) // This says it is fast (maybe a lie)

    //        data_cloud->width    = pts.size();
    //        data_cloud.height   = 1;
    //        data_cloud.is_dense = false; // should be true?
    //        data_cloud.points.resize (data_cloud.width * data_cloud.height);
    pcl::PointCloud<pcl::PointXYZ> temp_data_cloud;
//    temp_data_cloud.clear();
//    temp_data_cloud.width = icols;
//    temp_data_cloud.height = irows;

    for (int y=0; y < irows; y++)
    {
        for (int x=0; x < icols; x++)
        {
            pixval=aPixel(uchar,bgSubImageGraySmall.data,istep,ielemsize,x,y,0);

            //     pixval=aPixel(uchar,bgSubImageGraySmall.data,bgSubImageGraySmall.step,bgSubImageGraySmall.elemSize(),x,y,0);
            if (pixval > 40)
            {
                temp_data_cloud.push_back(pcl::PointXYZ(
                                              x*resolutionFractionMultiplier,
                                              y*resolutionFractionMultiplier,
                                              0));


            }
        }
    }

    data_cloud = temp_data_cloud;

    //See old code for other pixel looping methods

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

        currentTrackModelPoints = model_cloud_orig;

        data_cloud=  activeTracks[i]->update(data_cloud,currentTrackModelPoints, modelTOdataThreshold, separationThreshold,trackToBlobDistanceThreshold, Ticp_maxIter, Ticp_transformationEpsilon, Ticp_euclideanDistance);

    }

    qDebug()<<" data cloud Pts left over after existing tracks have removed data "<<data_cloud.size();

    /////////////////////////
    // add new tracks
    /////////////////////////

    //  dpoints associated with tracks were removed in previous update step
    //  Now we will add new tracks for remaining points
    //  Each point gets assigned a track
    for (uint i=0; i < data_cloud.size()-1 -(model_cloud_orig.size() * .01 * modelTOdataThreshold) ; i++) //Assumption, can't have a model if there are fewer points than deemed birthable anyway    //Assumption Won't have a model with a single point
    {
        if(data_cloud.size()<1) //This seems redundant, TODO remove
        {
            break;
        }

        currentTrackModelPoints = model_cloud_orig;
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
        cv::resize(bgSubImageGraySmallScratch, bgSubImageGray, Size(bgSubImageGray.cols,bgSubImageGray.rows)); //TODO problematic
        cvtColor(bgSubImageGray,bgSubImage,CV_GRAY2BGR);
        if (uiICP.display_pushButton->isChecked())
        {
            drawTrackResult(bgSubImage);
        }
    }



    return;
}


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
    trackResultImage.setTo(Scalar(0,0,0,0));



    pcl::PointCloud<pcl::PointXYZ> model_cloudtodraw;


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
        model_cloudtodraw = model_cloud_orig;



        activeTracks[i]->getTemplatePoints(model_cloudtodraw); //translates the points

        //  qDebug()<<"ModelCloud Size for drawing after getTemplate"<<model_cloudtodraw.size();

        ///////////////
        ///Draw Track Trails
        /////////////
        if(showTrails||true){
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
                cv::line(trackResultImage,currentPosition,previousPosition,Scalar(255+10*(-i),250/activeTracks.size()*i,300/activeTracks.size()*(2-i),255*(trackLineLength-k)/trackLineLength),3*(trackLineLength-k)/trackLineLength, CV_AA);
                previousPosition = currentPosition;

            }
        }

        ///////////////
        ///Draw Bounding Box
        //////////////
        if(showBox){
            RotatedRect boundRect( RotatedRect(Point2f(activeTracks[i]->getX(),activeTracks[i]->getY()),
                                               Size(trueModelwidth,trueModelheight),
                                               activeTracks[i]->getRotationAngle() * 180 / 3.1415926 //convert radians to degrees

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
            cv::circle(trackResultImage,Point(activeTracks[i]->getX(),activeTracks[i]->getY()),(trackToBlobDistanceThreshold*2),Scalar(0,255,255,100),1);
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
                    cv::circle(trackResultImage,Point(model_cloudtodraw.points[j].x, model_cloudtodraw.points[j].y),(separationThreshold),Scalar(250,0,0,50),1); //TODO, somehow this doesn't match with reality of removed points. Squaring, multiplying the distances does not make it work corrects
                }
                //Made model colored green for better visibility
                cv::circle(trackResultImage,Point(model_cloudtodraw.points[j].x, model_cloudtodraw.points[j].y),circleradius,Scalar(red,green,2,255),-1);
            }
        }//End Draw ICP Template

        ///////////////
        ///Draw ID Label Text
        //////////////
        if(uiICP.labelviewcheckBox->isChecked()){
        sprintf(label, "%d", activeTracks[i]->getID());
        cv::putText(trackResultImage, label, Point(activeTracks[i]->getX(),activeTracks[i]->getY()), font, fontscale, Scalar(0,0,255,255));
}


    }


    //Todo find  better way of overlaying images using alpha channels


    Mat imgBGRA;
    cvtColor(img,imgBGRA,CV_BGR2BGRA);
    cvtColor(trackResultImage,trackResultImage,CV_BGR2BGRA);

    alphaBlendBGRA<uchar>(imgBGRA,trackResultImage,trackResultImage);
//    addWeighted(trackResultImage,1,imgBGRA,.5,0,trackResultImage);
//   add(trackResultImage,imgBGRA, trackResultImage);

    cvtColor(trackResultImage,trackResultImage,CV_BGRA2BGR);

    return;
}



/**
 * Given a binary image for a model target,
 * extract and store the x,y coordinates in a
 * vector of pairs.
 *Store these in a pointcloud too!
 *
 */
pcl::PointCloud<pcl::PointXYZ> ICPTracker::loadModelPoints(Mat imgBGRA)
{
    Point coordinate;



    if(isContourTracking){//todo fix! This program runs the loader before loading the settings or something, and can switch contourtracking on!
       // imgAlpha = runContourDetection(imgAlpha);

    }


Mat tempImg;
    cv::resize(imgBGRA,tempImg,Size(),1./resolutionFractionMultiplier,1./resolutionFractionMultiplier, INTER_NEAREST);
    modelDimensions.x = tempImg.cols*resolutionFractionMultiplier;
    modelDimensions.y = tempImg.rows*resolutionFractionMultiplier;
    Mat tempImgGray;
    cvtColor(tempImg, tempImgGray,CV_BGR2GRAY); //Just gives grayscale intensity

    pcl::PointCloud<pcl::PointXYZ> loadmodel_cloud;// (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZRGB> loadmodelXYZRGB_cloud;

   int minX=DBL_MAX;
   int maxX=0;
   int minY=DBL_MAX;
   int maxY=0;

//   loadmodelXYZRGB_cloud.width= tempImg.cols;
//   loadmodelXYZRGB_cloud.height=tempImg.rows;

    for (int y=0; y < tempImg.rows; y++)
    {
        for (int x=0; x < tempImg.cols; x++)
        {
            // pixel = cvGet2D(tempImg, y, x);
            // int pixval =tempImg.at<uchar>(x,y); //This method works for a 1 channel image
           int pixval = tempImg.at<Vec4b>(y,x)[3];//Alpha Channel gives information about target location, everything else is just random

            if (pixval > 1) //There are target pixels at this location
            {



                coordinate.x = x*resolutionFractionMultiplier - modelDimensions.x/2; // center //scale back into full res coordinates
                coordinate.y = y*resolutionFractionMultiplier - modelDimensions.y/2; // center


                loadmodel_cloud.push_back(pcl::PointXYZ(
                                         x*resolutionFractionMultiplier - modelDimensions.x/2,
                                         y*resolutionFractionMultiplier- modelDimensions.y/2,
                                         0));



                loadmodelXYZRGB_cloud.push_back(pcl::PointXYZRGB(         tempImg.at<Vec4b>(y,x)[2], //Red
                                                                         tempImg.at<Vec4b>(y,x)[1], //Green
                                                                         tempImg.at<Vec4b>(y,x)[0] //Blue



                                                                   ));
                loadmodelXYZRGB_cloud.back().x=                                         x*resolutionFractionMultiplier - modelDimensions.x/2;
                loadmodelXYZRGB_cloud.back().y=                                         y*resolutionFractionMultiplier - modelDimensions.y/2;
                loadmodelXYZRGB_cloud.back().z=    0;// tempImgGray.at<uchar(x,y); //In the future assign z depth based on lightness                                     y*resolutionFractionMultiplier - modelDimensions.y/2;



                int r,g,b;
                const unsigned char *color = reinterpret_cast<const unsigned char *>(&loadmodelXYZRGB_cloud.back().rgb);
                r = color[2];
                g = color[1];
                b = color[0];

                if(coordinate.x<minX) minX=coordinate.x;
                if(coordinate.y<minY) minY=coordinate.y;
                if(coordinate.x>maxX) maxX=coordinate.x;
                if(coordinate.y>maxY) maxY=coordinate.y;




                qDebug()<<loadmodelXYZRGB_cloud.back().x<<"   Y "<<loadmodelXYZRGB_cloud.back().y <<"   RGB "<<r <<"   written red "<<loadmodelXYZRGB_cloud.back().r
                       <<"  max X, Max Y, minx , minY "<<maxX<< "  "<<maxY<< "  "<<minX<< "  "<<minY;




            }
        }
    }

     trueModelwidth;
     trueModelheight;
    trueModelwidth=maxX;
    if(maxX<abs(minX)) trueModelwidth=minX;

    trueModelheight=maxY;
    if(maxY<abs(minY)) trueModelheight=minY;

    trueModelwidth=abs(trueModelwidth*2);
    trueModelheight=abs(trueModelheight*2);
    qDebug()<<" modx width "<<trueModelwidth<<"   modHeight "<<trueModelheight;

//    maxModelDimension = modelImage.rows > modelImage.cols ? modelImage.rows : modelImage.cols; //Old method uses image shape
    maxModelDimension = trueModelwidth > trueModelheight ? trueModelwidth : trueModelheight; //Old method uses image shape

    return loadmodel_cloud;
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
