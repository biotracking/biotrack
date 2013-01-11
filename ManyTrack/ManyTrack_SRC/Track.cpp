#include "Track.h"
#include <Eigen/Geometry>




Track::Track(int index, pcl::PointXYZRGB initTranslation, int instanceID, double matchDThresh,int sepThresh, int resFracMultiplier)
{
    resolutionFracMultiplier = resFracMultiplier;
    frameIndex = index;
    birthFrameIndex = index;
    deathFrameIndex = 0;
    IDnum = instanceID;
    initialTranslation = initTranslation;
    matchDistanceThreshold = matchDThresh;
    nukeDistanceThreshold = sepThresh;
    numberOfContinuousZombieFrames = 0;
    isBirthable = false;
    isBirthFrame=true;

    //initialize in case of no matching to models
    QString modelType="none";

}

Track::~Track()
{
}

/**
 * Given a vector of data points and a vector of model
 * points, use ICP to register the model to the data.  Add registration transformation
 * to transforms vector and registration transformation from
 * the track's birth frame to absoluteTransforms vector.
 */
pcl::PointCloud<pcl::PointXYZRGB> Track::updatePosition(pcl::PointCloud<pcl::PointXYZRGB> dataPTS_cloud,vector<Model> modelPTS_clouds,
                                             int modelTODataThreshold, int separateThreshold, int matchDThresh,
                                             int ICP_ITERATIONS, double ICP_TRANSEPSILON, double ICP_EUCLIDEANDIST)
{

    matchDistanceThreshold=matchDThresh;
    nukeDistanceThreshold = separateThreshold;

    icp_maxIter=ICP_ITERATIONS;
    icp_transformationEpsilon=ICP_TRANSEPSILON;
    icp_euclideanDistance=ICP_EUCLIDEANDIST;


    pcl::PointCloud<pcl::PointXYZRGB> modelToProcess_cloud;

    Eigen::Matrix4f T;

    T.setIdentity();


    double birthAssociationsThreshold = modelPTS_clouds[0].cloud.size() * .01 *modelTODataThreshold; //TODO Shouldn't hardcode this to the first!

    double healthyAssociationsThreshold = birthAssociationsThreshold*.4; //Still healthy with 40 percent of a whole model


    if (frameIndex == birthFrameIndex)
    {

        isBirthFrame=true; // Try out Doing extra work aligning the objects if it is the very first frame
        //Determine what model this creature should use
        if(modelPTS_clouds.size()<2){// If there is only one model, just use that one
            modelIndex = 0;
        }
        else{
        modelIndex= identify(dataPTS_cloud,modelPTS_clouds);
}

    }
    else
    {
        isBirthFrame=false;


    }







    modelToProcess_cloud = modelPTS_clouds[modelIndex].cloud;

    //STRIP 3D data
    PointCloud<PointXY> dataPTS_cloud2D;
    copyPointCloud(dataPTS_cloud,dataPTS_cloud2D);
    PointCloud<PointXY> modelPTS_cloud2D;
    copyPointCloud(modelToProcess_cloud,modelPTS_cloud2D);

             PointCloud<PointXYZRGB> dataPTS_cloudStripped;
             copyPointCloud(dataPTS_cloud2D,dataPTS_cloudStripped);

             PointCloud<PointXYZRGB> modelPTS_cloudStripped;
             copyPointCloud(modelPTS_cloud2D,modelPTS_cloudStripped);


    //leave open for other possible ICP methods
    bool doPCL_ICP=true;

    //PCL implementation of ICP
    if(doPCL_ICP){

        if (dataPTS_cloud.size() > 1) //TODO FIX the math will throw an error if there are not enough data points
        {



double fitnessin;
            // Find transformation from the orgin that will optimize a match between model and target
            T=   calcTransformPCLRGB(dataPTS_cloud, modelToProcess_cloud,&fitnessin); //Use 3D color

//            T=   calcTransformPCLRGB(dataPTS_cloudStripped, modelPTS_cloudStripped, &fitnessin); // Just 2D


            //Apply the Transformation
            pcl::transformPointCloud(modelToProcess_cloud,modelToProcess_cloud, T);
            pcl::transformPointCloud(modelPTS_cloudStripped,modelPTS_cloudStripped, T);


        }

    }

    else{ //Other Registration implementations


    }


    int totalpointsBeforeRemoval=dataPTS_cloud.size();
    int totalRemovedPoints = 0;

    /** Remove old data points associated with the model
      * delete the points under where the model was placed. The amount of points destroyed in this process gives us a metric for how healthy the track is.
      *
      **/
    pcl::PointCloud<pcl::PointXYZRGB> dataPTSreduced_cloud;

    //Removeclosestdatacloudpoints strips the 3D data and nukes points in the proximitiy of where our model has lined up
    pcl::copyPointCloud(removeClosestDataCloudPoints(dataPTS_cloud, modelToProcess_cloud, nukeDistanceThreshold ),dataPTSreduced_cloud);


    totalRemovedPoints = totalpointsBeforeRemoval - dataPTSreduced_cloud.size();


    qDebug()<<"Removed Points from Track  "<<totalRemovedPoints;

    /// For Debugging we can visualize the Pointcloud
             /**
                pcl:: PointCloud<PointXYZRGB>::Ptr dataPTS_cloud_ptr (new pcl::PointCloud<PointXYZRGB> (dataPTS_cloud));
              //  copyPointCloud(modelPTS_cloud,)
                transformPointCloud(modelPTS_clouds[modelIndex].cloud,modelPTS_clouds[modelIndex].cloud,T);

                pcl:: PointCloud<PointXYZRGB>::Ptr model_cloud_ptrTempTrans (new pcl::PointCloud<PointXYZRGB> (modelPTS_clouds[modelIndex].cloud));

                pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
                viewer.showCloud(dataPTS_cloud_ptr);

            int sw=0;
                        while (!viewer.wasStopped())
                        {
                            if(sw==0){
                            viewer.showCloud(model_cloud_ptrTempTrans);
                            sw=1;
                            }
                            else{
                                viewer.showCloud(dataPTS_cloud_ptr);
            sw=0;
                            }

                        }
            /**/



    //////
    ///Determine Health of latest part of track
    ////////

    // Just born tracks go here: //Should get rid of this, doesn't make sense
    if (frameIndex == birthFrameIndex)
    {

        absoluteTransforms.push_back(T);

        //Check number of data points associated with this track, if it is greater than
        // the birth threshold then set birth flag to true.
        //If it doesn't hit this, it never gets born ever

        if ( totalRemovedPoints>birthAssociationsThreshold ) //matchScore >birthAssociationsThreshold && //Need new check for birthAssociationsthresh//  closestToModel.size() >= birthAssociationsThreshold)
        {
            isBirthable=true;


        }

    }
    // Tracks that are older than 1 frame get determined here
    else
    {
        //Check the number of data points associated with this track, if it is less than
        //the zombie/death threshold, then copy previous transform, and add this frame
        //index to zombieFrames

        //Is the track unhealthy? if so make it a zombie
        if ( totalRemovedPoints<healthyAssociationsThreshold )// !didConverge)// No zombies for now! eternal life! !didConverge) //closestToModel.size() < healthyAssociationsThreshold && absoluteTransforms.size() > 2)
        {

            absoluteTransforms.push_back(T);

            zombieIndicesIntoAbsTransforms.push_back(absoluteTransforms.size()-1);
            numberOfContinuousZombieFrames++;
        }
        else // Not zombie frame, keep using new transforms
        {

            // add to transforms
            absoluteTransforms.push_back(T);
            numberOfContinuousZombieFrames = 0;

        } //not zombie frame
    }// Not first frame



    // increment frame counter
    frameIndex++;

    return dataPTSreduced_cloud;

}

/**
  Loop over container of pairs of model clouds and string identifiers

  transform each

  find the one with the best fit

  choose this one as the identity


  **/

int Track::identify (PointCloud<PointXYZRGB> dataPTS_cloud,vector<Model> modelgroup){

    /// Timing
    double t = (double)getTickCount();


    float fit=DBL_MAX;
    int identity=0;
//    vector<pair<int,double> > id_scores;

pair <int,double>* id_score_arr;
id_score_arr = new pair <int,double>[modelgroup.size()*2];

    cv::parallel_for_(Range(0, modelgroup.size()),Identify_Parallel(dataPTS_cloud, modelgroup, this, &id_score_arr, modelgroup.size()) );



    /**/ // parallel testing
//    const Identify_Parallel body(dataPTS_cloud, modelgroup, this, &id_scores);

//        cv::parallel_for_(Range(0, modelgroup.size()),Identify_Parallel(dataPTS_cloud, modelgroup, this, &id_scores) );

        //NO VECTORS ANDY! PARALLEL HATES VECTORS
        double bestfit = DBL_MAX; // id_scores.at(0).second;
         identity=0;
         for (int i=0; i< (modelgroup.size());i++){

             int id = id_score_arr[i].first;
             double score = id_score_arr[i].second;

             qDebug()<<"score checking iterator "<<i<<" id "<< id<<" name "<<modelgroup[id].name<<" scores "<< score <<" current bestfit"<<bestfit;
     // TODO fix//This above line breaks on low resolutions. I think if the ID score is too low it gives a crazy big numberand QDebug can't write it, need to find how to make qDebug write it
             if(score<bestfit){
                 bestfit= score;
                     identity = id;
     //                qDebug()<<"Better Fit!  "<<identity<<" name "<<modelgroup[identity].name;


              }
     //        qDebug()<<"Current Best ID  "<<identity<<" name "<<modelgroup[identity].name;

         }



/*
    //Compare the Results
   double bestfit = DBL_MAX; // id_scores.at(0).second;
    identity=0;
    for (int i=0; i< (id_scores.size());i++){
        qDebug()<<"score checking iterator "<<i<<" id "<< id_scores.at(i).first<<" name "<<modelgroup[id_scores.at(i).first].name<<" scores "<< id_scores.at(i).second <<" current bestfit"<<bestfit;
// TODO fix//This above line breaks on low resolutions. I think if the ID score is too low it gives a crazy big numberand QDebug can't write it, need to find how to make qDebug write it
        if(id_scores.at(i).second<bestfit){
            bestfit= id_scores.at(i).second;
                identity =        id_scores.at(i).first;
//                qDebug()<<"Better Fit!  "<<identity<<" name "<<modelgroup[identity].name;


         }
//        qDebug()<<"Current Best ID  "<<identity<<" name "<<modelgroup[identity].name;

    }

    */


    t = ((double)getTickCount() - t)/getTickFrequency();
    qDebug() << "::: ID parallel time " << t << endl;
     t = (double)getTickCount();
    qDebug()<<"end Parallel ID testing, selected value was: "<<modelgroup[identity].name;


    /**/

    /** // Regular Serial ID Testing
    for (int i=0; i<modelgroup.size();i++){

        qDebug()<<"Model Cloud "<<modelgroup[i].name<<"  idx "<<i;

        PointCloud<PointXYZRGB> modelTOIdentify = modelgroup[i].cloud;
        int recentFit;
                 calcTransformPCLRGB(dataPTS_cloud, modelTOIdentify, &recentFit);
                 qDebug()<<"score checking  "<<i<<" recentfit "<<recentFit<<"  bestfit"<<fit;

                 if(recentFit<fit){
                     fit = recentFit;
//                  identity = i;
                 }


    }

    t = ((double)getTickCount() - t)/getTickFrequency();
    qDebug() << "::: Serial  ID time " << t << endl;
    qDebug()<<"end Regular ID testing, selected value was: "+modelgroup[identity].name;

    /**/



    return identity;

}


/**
 * Apply transformation to an input model_Cloud
 */
void Track::transformCloud(pcl::PointCloud<pcl::PointXYZRGB> modelPTS_cloud, Eigen::Matrix4f transform)
{
    pcl::transformPointCloud(modelPTS_cloud,modelPTS_cloud, transform); //This function call is weird! need to dereference pointers

}






Eigen::Matrix4f Track::calcTransformPCLRGB(pcl::PointCloud<pcl::PointXYZRGB> data_cloud,pcl::PointCloud<pcl::PointXYZRGB> model_cloud, double* fitness){

    Eigen::Matrix4f ET;
    Matrix4f guess,guess180;
    guess.setIdentity();
    guess180.setIdentity();
    pcl:: PointCloud<PointXYZRGB>::Ptr model_cloud_ptr (new pcl::PointCloud<PointXYZRGB> (model_cloud));
    pcl:: PointCloud<PointXYZRGB>::Ptr data_cloud_ptr (new pcl::PointCloud<PointXYZRGB> (data_cloud));

    //Setup ICP
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//    pcl::registration::TransformationEstimation2D<PointXYZRGB, PointXYZRGB>::Ptr trans_2D (new  pcl::registration::TransformationEstimation2D<PointXYZRGB, PointXYZRGB>);

    //online guy uses this:  typedef ConstrainedSVD<pcl::PointXYZ, pcl::PointXYZ> constSVD;  boost::shared_ptr<constSVD> constSVDptr(new constSVD);

    typedef pcl::registration::TransformationEstimation2D<PointXYZRGB, PointXYZRGB> trans_2D;
    boost::shared_ptr<trans_2D> trans2Dptr(new trans_2D);
//    icp.setTransformationEstimation (trans2Dptr);


//  IterativeClosestPointColor<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; // Test UV implmentation // Testing Color ICP -- works but the fitness scoring doesn't seem good
    icp.setInputCloud(model_cloud_ptr);
    icp.setInputTarget(data_cloud_ptr);
    pcl::PointCloud<pcl::PointXYZRGB> Final;



//   pcl::registration::TransformationEstimationSVD
//    icp.setTransformationEstimation();








    // if(model_cloud_ptr->is_dense){ //TODO Do we need this?

    //////////
    ////Set Parameters

    //TODO Test different Parameters for First frame and all others
    //First frame should match REALLY WELL

    //However, we also just removed noisy data this way, and so we would be spending lots of extra time
    //trying to align against a cluster of points we really just want to remove

    if(isBirthFrame){

        // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)

//              icp.setMaxCorrespondenceDistance (DBL_MAX);
              icp.setMaxCorrespondenceDistance (matchDistanceThreshold);

        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (icp_maxIter*3); // Work harder on birth frame //Timeout

        //Set the transformation epsilon (criterion 2)
        //This is the maximum distance that the model can move between iterations and still be thought to have converged
        icp.setTransformationEpsilon (icp_transformationEpsilon);

        //Set the euclidean distance difference epsilon (criterion 3)
        //This is how well a model needs to fit to consider the model to have converged
        // icp.setEuclideanFitnessEpsilon (icp_euclideanDistance);



        //Other Parameters

        icp.setRANSACIterations(0); //RANSAC needs the below parameter to work unless it is zero, default RANSAC properties are 1000 iterations and 0.05 distance threshold
//        icp.setRANSACOutlierRejectionThreshold(matchDistanceThreshold);


        /// Create Guess For Birthframe
        // This matrix is a Matrix4F composed of (what i think) is a translation vector on the right hand column,
        // and the top left 3X3 matrix is an AngleAxis rotation matrix

        guess <<    1,0,0, initialTranslation.x,
                0,1,0,initialTranslation.y,
                0,0,1,0,
                0,0,0,1;


    }
    else{

        // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)

        //Currently thinking that there should be no max distance, because with noisy data, erroneous points take chunks out of good detections
              icp.setMaxCorrespondenceDistance (matchDistanceThreshold);
//        icp.setMaxCorrespondenceDistance (DBL_MAX);


        // Set the maximum number of iterations (criterion 1)
        //int maxIt = uitrack.ICP_MaxIterspinBox->value(); //FIX this doesn't work to connect to the UI in this way
        icp.setMaximumIterations (icp_maxIter);  //Timeout



        //Set the transformation epsilon (criterion 2)
        //This is the maximum distance that the model can move between iterations and still be thought to have converged
        icp.setTransformationEpsilon (icp_transformationEpsilon); //Default value is 0

        //Set the euclidean distance difference epsilon (criterion 3)
        //This is how well a model needs to fit to consider the model to have converged
        //the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before
        //          * the algorithm is considered to have converged.
        //          * The error is estimated as the sum of the differences between correspondences in an Euclidean sense,
        //          * divided by the number of correspondences.
        //  icp.setEuclideanFitnessEpsilon (icp_euclideanDistance); //Default value is   -1.79769e+308


        //Other Parameters
        icp.setRANSACIterations(0); //RANSAC needs the below parameter to work unles it is zero, default RANSAC properties are 1000 iterations and 0.05 distance threshold
//        icp.setRANSACOutlierRejectionThreshold(matchDistanceThreshold);

        /// Create Guess For Latterframes
        // use last transformation added
        guess= absoluteTransforms.back();


    }

    //////////////
    ///Align 1
    //// Primary alignement with initial guess

    icp.align(Final,guess);

    ET=icp.getFinalTransformation();

    recentFitness = icp.getFitnessScore();
//    recentFitness = icp.colorfitness; // For ColorICP

//    qDebug() << "has converged:" << icp.hasConverged() << " score: " <<recentFitness<<"  RANSAC Iterations: " <<icp.getRANSACIterations()<< "RansacOutlierRejection"<< icp.getRANSACOutlierRejectionThreshold() << " Transepsilon: " <<icp.getTransformationEpsilon() <<" EuclideanFitnes: " <<icp.getEuclideanFitnessEpsilon()<< "  Data Points in sight: "<<data_cloud.size();
//    qDebug();

    matchScore = icp.getFitnessScore();
//    matchScore = icp.colorfitness; // For ColorICP

    didConverge = icp.hasConverged();

    *fitness=icp.getFitnessScore();

//     *fitness=icp.getFitnessScore(10.0);

//    *fitness=icp.colorfitness; // for ColorICP



    /* Use the rotated images from the get-go now

    if(isBirthFrame&&false ){

        //180 flip calculated manually
        //Feed in the results of the previous alignment
        // use the full previous transformation and just flip it 180 about its centroid
        //Let's give it a try
        guess180 <<    -1,0,0,0,
                0,-1,0,0,
                0,0,1,0,
                0,0,0,1;
       guess180=guess*guess180;//Multiply in the reverse order that you would like to perform the transformations

      // guess180= ET*guess180; //For some reason this doesn't work, I thought this would be more optimal? but it misses the flip more often! I must be envisioning the math wrong.The above version even works better, goes 10% faster
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp2;
//         pcl::IterativeClosestPointColor<pcl::PointXYZRGB, pcl::PointXYZRGB> icp2; // Testing Color ICP -- Does not work, get Boost shared
//        icp2=icp;


        icp2.align(Final, guess180);

        if(icp2.getFitnessScore()<recentFitness){
            recentFitness=icp2.getFitnessScore();

        }
//        qDebug() << "180 has converged:" << icp2.hasConverged() << " score: " <<icp2.getFitnessScore()<<"  RANSAC Iterations: " <<icp2.getRANSACIterations()<< "RansacOutlierRejection"<< icp2.getRANSACOutlierRejectionThreshold() << " Transepsilon: " <<icp2.getTransformationEpsilon() <<" EuclideanFitnes: " <<icp2.getEuclideanFitnessEpsilon()<< "  Data Points in sight: "<<data_cloud.size();
//        qDebug();
//        *fitness=recentFitness;

        //Temp invert!
        if(icp2.getFitnessScore()<matchScore){
            qDebug()<<"Flip it!" ;
            return icp2.getFinalTransformation();
        }


    }*/
//    *fitness=icp.colorfitness;

    return ET;

}


double Track::getX(int idx)
{
    if (idx < 0) idx = frameIndex;
    Eigen::Matrix4f T = absoluteTransforms[idx-birthFrameIndex-1]; //TODO why the -1?
    return T(0,3);
}

double Track::getY(int idx)
{
    if (idx < 0) idx = frameIndex;
    Eigen::Matrix4f T = absoluteTransforms[idx-birthFrameIndex-1];
    return T(1,3);
}

pair <Point,double> Track::getXYT(int idx){
    //cout << " --- Get Rotations --- " << endl;
pair <Point,double> returnInfo;
    if (idx < 0) idx = frameIndex;
    Eigen::Matrix4f T = absoluteTransforms[idx-birthFrameIndex-1];
    float theta;

    //Manually create a cloud and shift
    pcl::PointCloud<pcl::PointXYZ>  standardcloud;




    // make the angle match convention of pointing down X axis = 0 degrees, poiting down positive y axis = 90 degrees

    pcl::PointXYZ p1,p2;
    p1.x=0; p1.y=0;
    p2.x=100; p2.y=0;

    standardcloud.push_back(p1);
    standardcloud.push_back(p2);
    // transformCloud(standardcloud, T);
    pcl::transformPointCloud(standardcloud,standardcloud, T);

    theta = atan2((float)(standardcloud.points[1].y-standardcloud.points[0].y), (float)(standardcloud.points[1].x-standardcloud.points[0].x)); // changed this function to make consistent
    //  cout << " The manually translated angle in degrees : " <<theta*180/3.14157 << endl;

    /* I don't know why this doesn't work for angles between 240 and 360
    Eigen::Matrix3f m;
     m = T.topLeftCorner(3,3);
     AngleAxisf aa(m);

     cout << " The aa axis : " << aa.axis() << endl;
     cout << " The aa angle in degrees : " <<aa.angle()*180/3.14157 << endl;

     theta=aa.angle();
     cout << " --- END Get Rotations --- " << endl;
*/
    Point pointXY;
    pointXY.x=standardcloud.points[0].x;
    pointXY.y=standardcloud.points[0].y;

    returnInfo.first= pointXY;

    returnInfo.second = theta;
    return returnInfo;

}

double Track::getScale(int idx)
{
    // TODO: remove or handle transformation with a scale DOF
    int theint=idx;
    theint++;
    return 1;
}

double Track::getRotationAngle(int idx)
{

    //cout << " --- Get Rotations --- " << endl;

    if (idx < 0) idx = frameIndex;
    Eigen::Matrix4f T = absoluteTransforms[idx-birthFrameIndex-1];
    float theta;

    //Manually create a cloud and shift
    pcl::PointCloud<pcl::PointXYZ>  standardcloud;




    // make the angle match convention of pointing down X axis = 0 degrees, poiting down positive y axis = 90 degrees

    pcl::PointXYZ p1,p2;
    p1.x=0; p1.y=0;
    p2.x=100; p2.y=0;

    standardcloud.push_back(p1);
    standardcloud.push_back(p2);
    // transformCloud(standardcloud, T);
    pcl::transformPointCloud(standardcloud,standardcloud, T);

    theta = atan2((float)(standardcloud.points[1].y-standardcloud.points[0].y), (float)(standardcloud.points[1].x-standardcloud.points[0].x)); // changed this function to make consistent
    //  cout << " The manually translated angle in degrees : " <<theta*180/3.14157 << endl;

    /* I don't know why this doesn't work for angles between 240 and 360
    Eigen::Matrix3f m;
     m = T.topLeftCorner(3,3);
     AngleAxisf aa(m);

     cout << " The aa axis : " << aa.axis() << endl;
     cout << " The aa angle in degrees : " <<aa.angle()*180/3.14157 << endl;

     theta=aa.angle();
     cout << " --- END Get Rotations --- " << endl;
*/
    return theta;


}

/**
 * Given a vector of Points, transform them and return them as an out param.
 */
void Track::getTemplatePoints(pcl::PointCloud<pcl::PointXYZRGB> &modelPts, int idx)
{
    if (idx < 0) idx = frameIndex;
    Eigen::Matrix4f T = absoluteTransforms[idx-birthFrameIndex-1];
    // transformCloud(modelPts,T);
    pcl::transformPointCloud(modelPts,modelPts, T);

    return;
}

/**
 * Given a cloud of datapts (an in/out variable), a vector of dataPts, an index to a specific
 * point in datapts and a distanceThreshold, add the list of indices into dataPts which are
 * within the distanceThreshold  to dirtyPts.
 *
 * @param dataPTSreduced_cloud a reference to a vector of Points
 * @param point_cloud_for_reduction   the collection of points that we want to delete some points from
  * @param distanceThreshold an integer Euclidean distance
 */
pcl::PointCloud<pcl::PointXYZRGB> Track::removeClosestDataCloudPoints(pcl::PointCloud<pcl::PointXYZRGB> point_cloud_for_reduction,pcl::PointCloud<pcl::PointXYZRGB> removal_Cloud, int distanceThreshold){

        //NOTE: you cannot feed a KNN searcher clouds with 1 or fewer datapoints!

        //KDTREE SEARCH

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        // Neighbors within radius search

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        float point_radius = distanceThreshold;

        PointCloud<pcl::PointXYZRGB> point_cloud_flattened;//Point cloud with extra Hue Dimension crushed to 0

        PointCloud<pcl::PointXYZRGB> point_cloud_for_return;
        point_cloud_for_return.reserve(point_cloud_for_reduction.size());

        // K nearest neighbor search with Radius

        if(point_cloud_for_reduction.size()>1){

            bool *marked= new bool[point_cloud_for_reduction.size()];
            memset(marked,false,sizeof(bool)*point_cloud_for_reduction.size());


            //Make a version of the original data cloud that is flattened to z=0 but with the same indice
            copyPointCloud( point_cloud_for_reduction,point_cloud_flattened);

            for(int q=0; q<point_cloud_flattened.size();q++){

                point_cloud_flattened.at(q).z=0;

            }

           pcl:: PointCloud<PointXYZRGB>::Ptr point_cloud_for_reduction_ptr (new pcl::PointCloud<PointXYZRGB> (point_cloud_flattened));


            kdtree.setInputCloud (point_cloud_for_reduction_ptr); //Needs to have more than 1 data pt or segfault


            // iterate over points in model and remove those points within a certain distance
            for (unsigned int c=0; c < removal_Cloud.size(); c++)
            {

                if(point_cloud_for_reduction.size()<2){
                    break;
                }


                pcl::PointXYZRGB searchPoint;

                searchPoint.x = removal_Cloud.points[c].x;
                searchPoint.y = removal_Cloud.points[c].y;
               //Need to take z as zero when using a flattened data point cloud
               searchPoint.z = 0;

                // qDebug() <<"Datapts before incremental remove"<< point_cloud_for_reduction.size();
                if ( kdtree.radiusSearch (searchPoint, point_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
                {
                    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                        if(point_cloud_for_reduction.size()>0) ///NOTE CHANGED FROM > 1

                        marked[pointIdxRadiusSearch[i]]=true;
    //                        point_cloud_for_reduction.erase(point_cloud_for_reduction.begin()+pointIdxRadiusSearch[i]);// point_cloud_for_reduction.points[ pointIdxRadiusSearch[i] ]
                    }
                }


            }


//TODO We can make the above PARALLEL (probably!)

            //DESTROY ALL MARKED POINTS
            for(uint q=0; q< point_cloud_for_reduction.size(); q++){
                if(!marked[q]){
                    point_cloud_for_return.push_back(point_cloud_for_reduction.at(q));
    //                point_cloud_for_return.at(q) = point_cloud_for_reduction.at(q);

                }

            }



            delete[] marked;
        }

        //point_cloud_for_reduction.resize();
        return point_cloud_for_return;

    }






