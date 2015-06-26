//
//  Point_Cloud_Analysis.cpp
//  Object Boundary estimation
//
//  Created by Chaudhary Krishneel on 11/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/Point_Cloud_Analysis.h"

/************************************************************************************************************
  Function to return the set input point cloud
************************************************************************************************************/
pcl::PointCloud<PointT>::Ptr PointCloudDataAnalysis::getInputPointCloud()
{
   return this->cloud__;
}

/************************************************************************************************************
  Function to return the filtered point cloud index
************************************************************************************************************/
 vector<int> PointCloudDataAnalysis::getFilteredPointCloudIndex()
{
   return this->index;
}

/************************************************************************************************************
  Function to remove point cloud with NaN value
************************************************************************************************************/
 void PointCloudDataAnalysis::removeNaNPointCloud()
{
   if(this->cloud__->empty())
   {
      std::cout << "ERROR! No point cloud data for NaN removing...." << std::endl;
      exit(-1);
   }
   this->index.clear();
   pcl::removeNaNFromPointCloud(*cloud__, *cloud__, this->index);
}


/************************************************************************************************************
  Function to set the input point cloud to extract features
************************************************************************************************************/
 void PointCloudDataAnalysis::setInputPointCloud(pcl::PointCloud<PointT>::Ptr _cloud)
{
   this->cloud__ = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
   pcl::copyPointCloud<PointT, PointT>(*_cloud, *cloud__);   
}

/************************************************************************************************************
  Function to compute point cloud surface normal
************************************************************************************************************/
 void PointCloudDataAnalysis::computePointCloudSurfaceNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal,
                                                            const int _knn,
                                                            const double _radius,
                                                            bool isKNN)
{
   if(this->cloud__->empty())
   {
      std::cout << "The Input cloud is Empty....." << std::endl;
      exit(-1);
   }
   
   pcl::NormalEstimationOMP<PointT, pcl::Normal> surfaceNormal__;
   surfaceNormal__.setInputCloud(this->cloud__);
   surfaceNormal__.setNumberOfThreads(8);
   
   pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
   surfaceNormal__.setSearchMethod(tree);
   
   if(isKNN)
   {
      surfaceNormal__.setKSearch(_knn);
   }
   else
   {
      surfaceNormal__.setRadiusSearch(_radius);
   }
   surfaceNormal__.compute(*_normal);
}

/************************************************************************************************************
  Function to compute the inner connecting/separating object boundaries
************************************************************************************************************/
 void PointCloudDataAnalysis::objectIntraBoundaryEstimation(pcl::PointCloud<pcl::Normal>::Ptr _normals,
                                                                  vector<float> & _boundary_vals)
{
   if(this->cloud__-> empty())
   {
      std::cout << "ERROR! No input cloud for object intra boundary estimaiton" << std::endl;
      return;
   }

   pcl::PointCloud<PointT>::Ptr _cloud (new pcl::PointCloud<PointT>);
   pcl::copyPointCloud<PointT, PointT> (*(this)->cloud__, *_cloud);
   
   //! get the neigbouring points
   vector<vector<int> > pointIdxNKN;
   this->pclNearestNeigbourSearch(pointIdxNKN, false, this->neigborSize___, 0.01);

   for (int j = 0; j < pointIdxNKN.size(); j++) {
      Eigen::Vector3f seed_vector__ = Eigen::Vector3f(_normals->points[j].normal_x,
                                                      _normals->points[j].normal_y,
                                                      _normals->points[j].normal_z);
      float max_difference__ = 0.0;
      float min_difference__ = (sizeof(short) * M_PI);
      
      for (int i = 0; i < pointIdxNKN[j].size(); i++) {
         int index = pointIdxNKN[j][i];
         Eigen::Vector3f neigbor_vector__ = Eigen::Vector3f(_normals->points[index].normal_x,
                                                            _normals->points[index].normal_y,
                                                            _normals->points[index].normal_z);
         float vector_scalar_prod =  this->computeVectorScalarProduct(neigbor_vector__, seed_vector__);

         if(vector_scalar_prod > max_difference__)
         {
            max_difference__ = vector_scalar_prod;
         }
         if(vector_scalar_prod < min_difference__)
         {
            min_difference__ = vector_scalar_prod;
         }
      }
      float variance__ = max_difference__ - min_difference__;
      _boundary_vals.push_back(variance__);
      
      if(variance__ > 0.10f)
      {
         _cloud->points[j].r = 0 * 255;
         _cloud->points[j].g = 0 * 255;
         _cloud->points[j].b = variance__ * 255;
      }
   }
   //pcl::io::savePCDFileASCII("/home/krishneel/Desktop/tomato_boundary.pcd", *_cloud);
}

/************************************************************************************************************
  Function to compute the vector dot product
************************************************************************************************************/
 float PointCloudDataAnalysis::computeVectorScalarProduct(const Eigen::Vector3f _normal_vector,
                                                         const Eigen::Vector3f _reference_vector)
{
   float scalarProduct__ = ((_normal_vector.dot(_reference_vector)) /
                            ((_normal_vector.norm() * _reference_vector.norm())));
   return static_cast<float> (scalarProduct__);
}

/************************************************************************************************************
  Function to compute the nearest neigbor of point cloud Normals
************************************************************************************************************/
 void PointCloudDataAnalysis::pclNearestNeigbourSearch(vector<vector<int> > &pointIndices,
                                                             bool isNeighborBased,
                                                             const int k,
                                                             const double radius)
{
   if(this->cloud__->empty())
   {
      std::cout << "ERROR! No input data for neigbor estimation..." << std::endl;
   }

   pcl::KdTreeFLANN<PointT> kdtree;
   kdtree.setInputCloud(this->cloud__);

   vector<vector<float> > pointSquaredDistance;

   for(int i = 0; i < this->cloud__->size(); i++)
   {
      //! tempory memeory buffer
      vector<int> pointIdx;
      vector<float> pointSqDist;

      //! seed point for computing the neighbour
      PointT searchPoint = this->cloud__->points[i];
      
      if(isNeighborBased)
      {
         kdtree.nearestKSearch(searchPoint, k, pointIdx, pointSqDist);
      }
      else
      {
         kdtree.radiusSearch(searchPoint, radius, pointIdx, pointSqDist);
      }

      //! store the data
      pointIndices.push_back(pointIdx);
      pointSquaredDistance.push_back(pointSqDist);

      //! Clear memory buffer
      pointIdx.clear();
      pointSqDist.clear();
   }
}


