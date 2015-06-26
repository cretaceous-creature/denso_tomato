//
//  Point_Cloud_Processing.h
//  boundary_estimation
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __boundary_estimation__Point_Cloud_Analysis__
#define __boundary_estimation__Point_Cloud_Analysis__

#include "../include/constants.h"

class PointCloudDataAnalysis
{
private:
   pcl::PointCloud<PointT>::Ptr cloud__;
   vector<int> index;

   //! Size of the neigbour search space
   const int neigborSize___;
   const double neigborRadius__;
   
   float computeVectorScalarProduct(const Eigen::Vector3f,
                                    const Eigen::Vector3f);
   
public:

   PointCloudDataAnalysis() : neigborSize___ (25), neigborRadius__ (0.01) {}
   
   
   pcl::PointCloud<PointT>::Ptr getInputPointCloud();
   vector<int> getFilteredPointCloudIndex();
   void removeNaNPointCloud();
   void pclNearestNeigbourSearch(vector<vector<int> > &,
                                 bool = true,
                                 const int = 12,
                                 const double = 0.01);
   
   void setInputPointCloud(pcl::PointCloud<PointT>::Ptr);
   void computePointCloudSurfaceNormal(pcl::PointCloud<pcl::Normal>::Ptr,
                                       const int = 12,
                                       const double = 0.01,
                                       bool = true);
   void objectIntraBoundaryEstimation(pcl::PointCloud<pcl::Normal>::Ptr,
                                      vector<float> &);
};
#endif /* defined(__boundary_estimation__Point_Cloud_Analysis__) */
