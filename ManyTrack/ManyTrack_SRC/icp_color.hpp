/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
//#include <pcl/registration/boost.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPointColor<PointSource, PointTarget>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
	
  PointCloudTarget target = *cloud;
  pcl::PointCloud<Eigen::MatrixXf>::Ptr cloud_tree( new pcl::PointCloud<Eigen::MatrixXf> ());

  cloud_tree->channels["cicp"].name     = "cicp";
  cloud_tree->channels["cicp"].offset   = 0;
  cloud_tree->channels["cicp"].size     = 4;
  cloud_tree->channels["cicp"].count    = 5;
  cloud_tree->channels["cicp"].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_tree->points.resize ( target.points.size (), 5);
  cloud_tree->is_dense = true;

  cloud_limits_[0] = std::numeric_limits<float>::infinity ();
  cloud_limits_[1] = std::numeric_limits<float>::infinity ();
  cloud_limits_[2] = std::numeric_limits<float>::infinity ();
  cloud_limits_[3] = -std::numeric_limits<float>::infinity ();
  cloud_limits_[4] = -std::numeric_limits<float>::infinity ();
  cloud_limits_[5] = -std::numeric_limits<float>::infinity ();

  for (size_t i = 0; i < target.points.size (); ++i)
  {
    if ( target.points[i].data[0] < cloud_limits_[0])
      cloud_limits_[0] = target.points[i].data[0];
    if ( target.points[i].data[1] < cloud_limits_[1])
      cloud_limits_[1] = target.points[i].data[1];
    if ( target.points[i].data[2] < cloud_limits_[2])
      cloud_limits_[2] = target.points[i].data[2];

    if ( target.points[i].data[0] > cloud_limits_[3])
      cloud_limits_[3] = target.points[i].data[0];
    if ( target.points[i].data[1] > cloud_limits_[4])
      cloud_limits_[4] = target.points[i].data[1];
    if ( target.points[i].data[2] > cloud_limits_[5])
      cloud_limits_[5] = target.points[i].data[2];
  }

  float check_d1 = cloud_limits_[3] - cloud_limits_[0];
  float check_d2 = cloud_limits_[4] - cloud_limits_[1];
  float check_d3 = cloud_limits_[5] - cloud_limits_[2];

  if (std::isnan (check_d1) || check_d1 == 0)
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] The difference cloud_limits_[3] - cloud_limits_[0] could not be equal to 0 or NaN! The difference is currently equal to %f.\n", getClassName ().c_str (), check_d1);
    return;
  }
  else if (std::isnan (check_d2) || check_d2 == 0)
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] The difference cloud_limits_[4] - cloud_limits_[1] could not be equal to 0 or NaN! The difference is currently equal to %f.\n", getClassName ().c_str (), check_d2);
    return;
  }
  else if (std::isnan (check_d3) || check_d3 == 0)
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] The difference cloud_limits_[5] - cloud_limits_[2] could not be equal to 0 or NaN! The difference is currently equal to %f.\n", getClassName ().c_str (), check_d3);
    return;
  }
  else
  {
    for (size_t i = 0; i < target.points.size (); ++i)
    {
      cloud_tree->points(i, 0) = (target.points[i].data[0]-cloud_limits_[0]) / check_d1;
      cloud_tree->points(i, 1) = (target.points[i].data[1]-cloud_limits_[1]) / check_d2;
      cloud_tree->points(i, 2) = (target.points[i].data[2]-cloud_limits_[2]) / check_d3;

      float u_yuv = (-(0.148 * target.points[i].r) - (0.291 * target.points[i].g) + (0.439 * target.points[i].b) + 128 ) /255.0;
      float v_yuv = ( (0.439 * target.points[i].r) - (0.368 * target.points[i].g) - (0.071 * target.points[i].b) + 128 ) /255.0;

      cloud_tree->points(i, 3) = u_yuv;
      cloud_tree->points(i, 4) = v_yuv;
    }

    target_ = target.makeShared ();
    tree_->setInputCloud (target_);
    tree_color_->setInputCloud(cloud_tree);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pcl::IterativeClosestPointColor<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess)
{
  // Allocate enough space to hold the results
  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudTarget input_corresp;
  input_corresp.points.resize (indices_->size ());

  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;

  // If the guessed transformation is non identity
  if (guess != Eigen::Matrix4f::Identity ())
  {
    // Initialise final transformation to the guessed one
    final_transformation_ = guess;
    // Apply guessed transformation prior to search for neighbours
    transformPointCloud (output, output, guess);
  }

  // Resize the vector of distances between correspondences 
  std::vector<float> previous_correspondence_distances (indices_->size ());
  correspondence_distances_.resize (indices_->size ());

  while (!converged_)           // repeat until convergence
  {
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;
    // And the previous set of distances
    previous_correspondence_distances = correspondence_distances_;

    int cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());

    pcl::PointCloud<Eigen::MatrixXf>::Ptr cloud_tree ( new pcl::PointCloud<Eigen::MatrixXf> );

    cloud_tree->channels["cicp"].name     = "cicp";
    cloud_tree->channels["cicp"].offset   = 0;
    cloud_tree->channels["cicp"].size     = 4;
    cloud_tree->channels["cicp"].count    = 5;
    cloud_tree->channels["cicp"].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_tree->points.resize ( indices_->size (), 5);
    cloud_tree->is_dense = true;

    for (size_t idx = 0; idx < indices_->size (); ++idx)
    {
      cloud_tree->points(idx, 0) = (output.points[ (*indices_)[idx] ].data[0]-cloud_limits_[0]) / (cloud_limits_[3]-cloud_limits_[0]);
      cloud_tree->points(idx, 1) = (output.points[ (*indices_)[idx] ].data[1]-cloud_limits_[1]) / (cloud_limits_[4]-cloud_limits_[1]);
      cloud_tree->points(idx, 2) = (output.points[ (*indices_)[idx] ].data[2]-cloud_limits_[2]) / (cloud_limits_[5]-cloud_limits_[2]);

      float u_yuv = (-(0.148 * output.points[ (*indices_)[idx] ].r) - (0.291 * output.points[ (*indices_)[idx] ].g) + (0.439 * output.points[ (*indices_)[idx] ].b) + 128 ) /255.0;
      float v_yuv = ( (0.439 * output.points[ (*indices_)[idx] ].r) - (0.368 * output.points[ (*indices_)[idx] ].g) - (0.071 * output.points[ (*indices_)[idx] ].b) + 128 ) /255.0;

      cloud_tree->points(idx, 3) = u_yuv;
      cloud_tree->points(idx, 4) = v_yuv;

      // Iterating over the entire index vector and  find all correspondences
      if (!tree_color_->nearestKSearch (*cloud_tree, idx, 1, nn_indices, nn_dists))
      {
	PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[idx]);
	return;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        source_indices[cnt] = (*indices_)[idx];
        target_indices[cnt] = nn_indices[0];
        cnt++;
      }

      // Save the nn_dists[0] to a global vector of distances
      correspondence_distances_[(*indices_)[idx]] = std::min (nn_dists[0], static_cast<float> (dist_threshold));
    }

    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    // Resize to the actual number of valid correspondences
    source_indices.resize (cnt); target_indices.resize (cnt);

    std::vector<int> source_indices_good;
    std::vector<int> target_indices_good;
    {
      // From the set of correspondences found, attempt to remove outliers
      // Create the registration model
      typedef typename SampleConsensusModelRegistration<PointSource>::Ptr SampleConsensusModelRegistrationPtr;
      SampleConsensusModelRegistrationPtr model;
      model.reset (new SampleConsensusModelRegistration<PointSource> (output.makeShared (), source_indices));
      // Pass the target_indices
      model->setInputTarget (target_, target_indices);
      // Create a RANSAC model
      RandomSampleConsensus<PointSource> sac (model, inlier_threshold_);
      sac.setMaxIterations (ransac_iterations_);

      // Compute the set of inliers
      if (!sac.computeModel ())
      {
        source_indices_good = source_indices;
        target_indices_good = target_indices;
      }
      else
      {
        std::vector<int> inliers;
        // Get the inliers
        sac.getInliers (inliers);
        source_indices_good.resize (inliers.size ());
        target_indices_good.resize (inliers.size ());

        boost::unordered_map<int, int> source_to_target;
        for (unsigned int i = 0; i < source_indices.size(); ++i)
          source_to_target[source_indices[i]] = target_indices[i];

        // Copy just the inliers
        std::copy(inliers.begin(), inliers.end(), source_indices_good.begin());
        for (size_t i = 0; i < inliers.size (); ++i)
          target_indices_good[i] = source_to_target[inliers[i]];
      }
    }

    // Check whether we have enough correspondences
    cnt = static_cast<int> (source_indices_good.size ());
    if (cnt < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters.\n", getClassName ().c_str ());
      converged_ = false;
      return;
    }

    PCL_DEBUG ("[pcl::%s::computeTransformation] Number of correspondences %d [%f%%] out of %zu points [100.0%%], RANSAC rejected: %zu [%f%%].\n", 
        getClassName ().c_str (), 
        cnt, 
        (static_cast<float> (cnt) * 100.0f) / static_cast<float> (indices_->size ()), 
        indices_->size (), 
        source_indices.size () - cnt, 
        static_cast<float> (source_indices.size () - cnt) * 100.0f / static_cast<float> (source_indices.size ()));
  
    // Estimate the transform
    //rigid_transformation_estimation_(output, source_indices_good, *target_, target_indices_good, transformation_);
    transformation_estimation_->estimateRigidTransformation (output, source_indices_good, *target_, target_indices_good, transformation_);

    // Tranform the data
    transformPointCloud (output, output, transformation_);

    // Obtain the final transformation    
    final_transformation_ = transformation_ * final_transformation_;

    nr_iterations_++;

    // Update the vizualization of icp convergence
    if (update_visualizer_ != 0)
      update_visualizer_(output, source_indices_good, *target_, target_indices_good );

    // Various/Different convergence termination criteria
    // 1. Number of iterations has reached the maximum user imposed number of iterations (via 
    //    setMaximumIterations)
    // 2. The epsilon (difference) between the previous transformation and the current estimated transformation 
    //    is smaller than an user imposed value (via setTransformationEpsilon)
    // 3. The sum of Euclidean squared errors is smaller than a user defined threshold (via 
    //    setEuclideanFitnessEpsilon)

    if (nr_iterations_ >= max_iterations_ ||
        (transformation_ - previous_transformation_).array ().abs ().sum () < transformation_epsilon_ ||
        fabs (this->getFitnessScore (correspondence_distances_, previous_correspondence_distances)) <= euclidean_fitness_epsilon_
       )
    {
      converged_ = true;

      //Andy Added
      colorfitness = this->getFitnessScore (correspondence_distances_, previous_correspondence_distances);

//      cout<<"color fitness "<<colorfitness;
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, (transformation_ - previous_transformation_).array ().abs ().sum ());

      PCL_DEBUG ("nr_iterations_ (%d) >= max_iterations_ (%d)\n", nr_iterations_, max_iterations_);
      PCL_DEBUG ("(transformation_ - previous_transformation_).array ().abs ().sum () (%f) < transformation_epsilon_ (%f)\n",
                 (transformation_ - previous_transformation_).array ().abs ().sum (), transformation_epsilon_);
      PCL_DEBUG ("fabs (getFitnessScore (correspondence_distances_, previous_correspondence_distances)) (%f) <= euclidean_fitness_epsilon_ (%f)\n",
                 fabs (this->getFitnessScore (correspondence_distances_, previous_correspondence_distances)),
                 euclidean_fitness_epsilon_);

    }
  }
}

