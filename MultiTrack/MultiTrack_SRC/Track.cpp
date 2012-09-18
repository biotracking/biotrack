#include "Track.h"
#include <Eigen/Geometry>


Track::Track(int index, pcl::PointXYZ initTranslation, int identification, double matchDThresh,int sepThresh, int resFracMultiplier)
{
    resolutionFracMultiplier = resFracMultiplier;
    frameIndex = index;
    birthFrameIndex = index;
    deathFrameIndex = 0;
    id = identification;
    initialTranslation = initTranslation;
    matchDistanceThreshold = matchDThresh;
    nukeDistanceThreshold = sepThresh;
    numberOfContinuousZombieFrames = 0;
    isBirthable = false;
    isBirthFrame=true;

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
pcl::PointCloud<pcl::PointXYZ> Track::update(pcl::PointCloud<pcl::PointXYZ> dataPTS_cloud,pcl::PointCloud<pcl::PointXYZ> modelPTS_cloud,
                                             int modelTODataThreshold, int separateThreshold, int matchDThresh,
                                             int ICP_ITERATIONS, double ICP_TRANSEPSILON, double ICP_EUCLIDEANDIST)
{

    matchDistanceThreshold=matchDThresh;
    nukeDistanceThreshold = separateThreshold;

    icp_maxIter=ICP_ITERATIONS;
    icp_transformationEpsilon=ICP_TRANSEPSILON;
    icp_euclideanDistance=ICP_EUCLIDEANDIST;



    pcl::PointCloud<pcl::PointXYZ> tformedModel_cloud;

    pcl::PointCloud<pcl::PointXYZ> modelToProcess_cloud;

    Eigen::Matrix4f T;

    T.setIdentity();


    double birthAssociationsThreshold = modelPTS_cloud.size() * .01 *modelTODataThreshold;

    double healthyAssociationsThreshold = birthAssociationsThreshold*.4; //Still healthy with 40 percent of a whole model


    if (frameIndex == birthFrameIndex)
    {

        isBirthFrame=true; // Try out Doing extra work aligning the objects if it is the very first frame


    }
    else
    {
        isBirthFrame=false;


    }
    modelToProcess_cloud = modelPTS_cloud;

    //leave open for other possible ICP methods
    bool doPCL=true;
    //PCL implementation of ICP
    if(doPCL){

        if (dataPTS_cloud.size() > 1) //TODO FIX the math will throw an error if there are not enough data points
        {
            // Find transformation from the orgin that will optimize a match between model and target
            T=   updateTransformPCL(dataPTS_cloud, modelToProcess_cloud);
            pcl::transformPointCloud(modelToProcess_cloud,modelToProcess_cloud, T);

        }

    }

    else{ //no other ICP implementation currently
    }


    int totalpointsBeforeRemoval=dataPTS_cloud.size();
    int totalRemovedPoints = 0;

    /** Remove old data points associated with the model
      * delete the points under where the model was placed. The amount of points destroyed in this process gives us a metric for how healthy the track is.
      *
      **/
    pcl::PointCloud<pcl::PointXYZ> dataPTSreduced_cloud;

    removeClosestDataCloudPoints(dataPTS_cloud, modelToProcess_cloud, nukeDistanceThreshold );
    pcl::copyPointCloud(removeClosestDataCloudPoints(dataPTS_cloud, modelToProcess_cloud, nukeDistanceThreshold ),dataPTSreduced_cloud);

    totalRemovedPoints = totalpointsBeforeRemoval - dataPTSreduced_cloud.size();




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
 * Apply transformation to an input model_Cloud
 */
void Track::transformCloud(pcl::PointCloud<pcl::PointXYZ> modelPTS_cloud, Eigen::Matrix4f transform)
{
    pcl::transformPointCloud(modelPTS_cloud,modelPTS_cloud, transform); //This function call is weird! need to dereference pointers

}



Eigen::Matrix4f Track::updateTransformPCL(pcl::PointCloud<pcl::PointXYZ> data_cloud,pcl::PointCloud<pcl::PointXYZ> model_cloud){

    Eigen::Matrix4f ET;
    Matrix4f guess,guess180;
    guess.setIdentity();
    guess180.setIdentity();
    pcl:: PointCloud<PointXYZ>::Ptr model_cloud_ptr (new pcl::PointCloud<PointXYZ> (model_cloud));
    pcl:: PointCloud<PointXYZ>::Ptr data_cloud_ptr (new pcl::PointCloud<PointXYZ> (data_cloud));

    //Setup ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(model_cloud_ptr);
    icp.setInputTarget(data_cloud_ptr);
    pcl::PointCloud<pcl::PointXYZ> Final;



    // if(model_cloud_ptr->is_dense){ //TODO Do we need this?

    //////////
    ////Set Parameters

    //TODO Test different Parameters for First frame and all others
    //First frame should match REALLY WELL

    //However, we also just removed noisy data this way, and so we would be spending lots of extra time
    //trying to align against a cluster of points we really just want to remove

    if(isBirthFrame){

        // Set the max correspondence distance (e.g., correspondences with higher distances will be ignored)
        //Currently thinking that there should be no max distance, because with noisy data, erroneous points take chunks out of good detections
              icp.setMaxCorrespondenceDistance (DBL_MAX);

        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (icp_maxIter*3);  //Timeout is greater for birthframe (this kills performance for noisy data however)

        //Set the transformation epsilon (criterion 2)
        //This is the maximum distance that the model can move between iterations and still be thought to have converged
        icp.setTransformationEpsilon (icp_transformationEpsilon);

        //Set the euclidean distance difference epsilon (criterion 3)
        //This is how well a model needs to fit to consider the model to have converged
        // icp.setEuclideanFitnessEpsilon (icp_euclideanDistance);



        //Other Parameters

        //        icp.setRANSACIterations(icp_maxIter); //RANSAC needs the below parameter to work unles it is zero, default RANSAC properties are 1000 iterations and 0.05 distance threshold
        icp.setRANSACIterations(0); //RANSAC needs the below parameter to work unless it is zero, default RANSAC properties are 1000 iterations and 0.05 distance threshold
        icp.setRANSACOutlierRejectionThreshold(matchDistanceThreshold);


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

              icp.setMaxCorrespondenceDistance (matchDistanceThreshold);


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
        // icp.setRANSACIterations(icp_maxIter); //RANSAC needs the below parameter to work unles it is zero, default RANSAC properties are 1000 iterations and 0.05 distance threshold

        icp.setRANSACIterations(0); //RANSAC needs the below parameter to work unles it is zero, default RANSAC properties are 1000 iterations and 0.05 distance threshold
        icp.setRANSACOutlierRejectionThreshold(matchDistanceThreshold);

        /// Create Guess For Latterframes
        // use last transformation added
        guess= absoluteTransforms.back();


    }

    //////////////
    ///Align 1
    //// Primary alignement with initial guess

    icp.align(Final,guess);

    ET=icp.getFinalTransformation();


    qDebug() << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore()<<"  RANSAC Iterations: " <<icp.getRANSACIterations()<< "RansacOutlierRejection"<< icp.getRANSACOutlierRejectionThreshold() << " Transepsilon: " <<icp.getTransformationEpsilon() <<" EuclideanFitnes: " <<icp.getEuclideanFitnessEpsilon()<< "  Data Points in sight: "<<data_cloud.size();
    qDebug();

    matchScore = icp.getFitnessScore();
    didConverge = icp.hasConverged();


    if(isBirthFrame ){

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
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
        icp2=icp;


        icp2.align(Final, guess180);

        qDebug() << "180 has converged:" << icp2.hasConverged() << " score: " <<icp2.getFitnessScore()<<"  RANSAC Iterations: " <<icp2.getRANSACIterations()<< "RansacOutlierRejection"<< icp2.getRANSACOutlierRejectionThreshold() << " Transepsilon: " <<icp2.getTransformationEpsilon() <<" EuclideanFitnes: " <<icp2.getEuclideanFitnessEpsilon()<< "  Data Points in sight: "<<data_cloud.size();
        qDebug();

        //Temp invert!
        if(icp2.getFitnessScore()<matchScore){
            qDebug()<<"Flip it!" ;
            return icp2.getFinalTransformation();
        }


    }
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
void Track::getTemplatePoints(pcl::PointCloud<pcl::PointXYZ> &modelPts, int idx)
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
  * @param distanceThreshold an integer Euclidean distance
 */
pcl::PointCloud<pcl::PointXYZ> Track::removeClosestDataCloudPoints(pcl::PointCloud<pcl::PointXYZ> point_cloud_for_reduction,pcl::PointCloud<pcl::PointXYZ> removal_Cloud, int distanceThreshold){


    //NOTE: you cannot feed a KNN searcher clouds with 1 or fewer datapoints!

    //KDTREE SEARCH

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // Neighbors within radius search

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float point_radius = distanceThreshold;

    PointCloud<pcl::PointXYZ> point_cloud_for_return;
    point_cloud_for_return.reserve(point_cloud_for_reduction.size());

    // K nearest neighbor search with Radius

    if(point_cloud_for_reduction.size()>1){

        bool *marked= new bool[point_cloud_for_reduction.size()];
        memset(marked,false,sizeof(bool)*point_cloud_for_reduction.size());
//        for(uint q=0; q< point_cloud_for_reduction.size(); q++){
//            marked[q]=false;

//        }

        pcl:: PointCloud<PointXYZ>::Ptr point_cloud_for_reduction_ptr (new pcl::PointCloud<PointXYZ> (point_cloud_for_reduction));


        kdtree.setInputCloud (point_cloud_for_reduction_ptr); //Needs to have more than 1 data pt or segfault


        // iterate over points in model and remove those points within a certain distance
        for (unsigned int c=0; c < removal_Cloud.size(); c++)
        {

            if(point_cloud_for_reduction.size()<2){
                break;
            }


            pcl::PointXYZ searchPoint;

            searchPoint.x = removal_Cloud.points[c].x;
            searchPoint.y = removal_Cloud.points[c].y;
            searchPoint.z = removal_Cloud.points[c].z;

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





