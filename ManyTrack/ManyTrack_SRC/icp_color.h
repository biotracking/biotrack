/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#ifndef PCL_ICP_COLOR_H_
#define PCL_ICP_COLOR_H_

// PCL includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace pcl
{

template <typename PointSource, typename PointTarget>
class IterativeClosestPointColor : public Registration<PointSource, PointTarget>
{
  typedef typename Registration<PointSource, PointTarget>::PointCloudSource PointCloudSource;
  typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
  typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

  typedef typename Registration<PointSource, PointTarget>::PointCloudTarget PointCloudTarget;
  typedef typename Registration<PointSource, PointTarget>::PointCloudTargetConstPtr PointCloudTargetConstPtr;

  typedef PointIndices::Ptr PointIndicesPtr;
  typedef PointIndices::ConstPtr PointIndicesConstPtr;

  public:
    /** \brief Empty constructor. */
    IterativeClosestPointColor ()
    : tree_color_ (new pcl::KdTreeFLANN<Eigen::MatrixXf> (false))
    {
      reg_name_ = "IterativeClosestPointColor";
      ransac_iterations_ = 1000;
      transformation_estimation_.reset (new pcl::registration::TransformationEstimationSVD<PointSource, PointTarget>);
    };

    double colorfitness;
    virtual void
    setInputTarget (const PointCloudTargetConstPtr &cloud);

  protected:
    /** \brief Rigid transformation computation method  with initial guess.
      * \param output the transformed input point cloud dataset using the rigid transformation found
      * \param guess the initial guess of the transformation to compute
      */
    virtual void
    computeTransformation (PointCloudSource &output, const Eigen::Matrix4f &guess);

    pcl::KdTreeFLANN<Eigen::MatrixXf>::Ptr tree_color_;
    float cloud_limits_[6];

    using Registration<PointSource, PointTarget>::reg_name_;
    using Registration<PointSource, PointTarget>::getClassName;
    using Registration<PointSource, PointTarget>::input_;
    using Registration<PointSource, PointTarget>::indices_;
    using Registration<PointSource, PointTarget>::target_;
    using Registration<PointSource, PointTarget>::nr_iterations_;
    using Registration<PointSource, PointTarget>::max_iterations_;
    using Registration<PointSource, PointTarget>::ransac_iterations_;
    using Registration<PointSource, PointTarget>::previous_transformation_;
    using Registration<PointSource, PointTarget>::final_transformation_;
    using Registration<PointSource, PointTarget>::transformation_;
    using Registration<PointSource, PointTarget>::transformation_epsilon_;
    using Registration<PointSource, PointTarget>::converged_;
    using Registration<PointSource, PointTarget>::corr_dist_threshold_;
    using Registration<PointSource, PointTarget>::inlier_threshold_;
    using Registration<PointSource, PointTarget>::min_number_correspondences_;
    using Registration<PointSource, PointTarget>::update_visualizer_;
    using Registration<PointSource, PointTarget>::correspondence_distances_;
    using Registration<PointSource, PointTarget>::euclidean_fitness_epsilon_;
    using Registration<PointSource, PointTarget>::transformation_estimation_;
    using Registration<PointSource, PointTarget>::tree_;
  };
}

#include "icp_color.hpp"

#endif  //#ifndef PCL_ICP_COLOR_H_
