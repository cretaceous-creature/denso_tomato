//
//  Point_Cloud_Processing.h
//  boundary_estimation
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __boundary_estimation__Point_Cloud_Processing__
#define __boundary_estimation__Point_Cloud_Processing__

#include "../include/constants.h"
#include "../include/RGBD_Image_Processing.h"
#include "../include/Point_Cloud_Analysis.h"
#include "../include/mouse_event.h"

class PointCloudProcessing :
                            public RGBDImageProcessing,
                            public PointCloudDataAnalysis,
                            public ObjectBoundary
{
private:
   ros::NodeHandle nh_;
   ros::Publisher publisher__;
   ros::Subscriber subscriber__;

   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
  
   vector<int> __indices__;
   pcl::PointCloud<PointT>::Ptr pcl_cloud__;
   pcl::PointCloud<PointT>::Ptr filter_cloud__;
  
   //! Variable for distance filter
   const float MAX_DISTANCE;
   const float MIN_DISTANCE;

   void makeObjectTemplates(cv::Mat &);
   bool fileExists(const std::string&) ;

   bool maketemplate;
   cv_bridge::CvImagePtr cv_ptr;
   
protected:
   vector<int> __pointCloudIndices__;
   vector<Cloud3D2DIndex> pcl3D_2D_index__;
   
public:
   PointCloudProcessing() : MAX_DISTANCE(1.2f),
                            MIN_DISTANCE(0.80f),
                            maketemplate(true),
                            RGBDImageProcessing(sizeof(char)),
                            it_(nh_),
                            cv_ptr(new cv_bridge::CvImage())
   {
      ROS_INFO("RUNNING TOMATO DETECTOR ON 2D RGB SPACE");
      ROS_INFO("-> Subsribe to '\033[1;31m /tomato/detection/output \033[0m' topic to view results");
      
      
      __indices__.clear();
      __pointCloudIndices__.clear();
      pcl3D_2D_index__.clear();
      
      pcl_cloud__ = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);

      this->image_sub_ = it_.subscribe("/camera/rgb/image_rect_color",
                                       sizeof(char),
                                       &PointCloudProcessing::imageCallBack,
                                       this);
      
      this->subscriber__ = nh_.subscribe ("/camera/depth_registered/points",
                                          sizeof(char),
                                          &PointCloudProcessing::cloudCallBack,
                                          this);
      this->publisher__ = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud/view",
                                                                   sizeof(char));
      this->image_pub_ = it_.advertise("/tomato/detection/output",
                                 sizeof(char));
   }
  
   //! Function to get the index of points from 2D depth map
   void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &);
   void imageCallBack(const sensor_msgs::ImageConstPtr &);
   void setFilteredIndexFromDepth(vector<int> &);
   void getPointCloudRegion(pcl::PointCloud<PointT>::Ptr, int = 20);
   pcl::PointCloud<PointT>::Ptr filterPointCloudby2DIndices(const pcl::PointCloud<PointT>::Ptr, bool = false);
   void pointCloud2RGBDImage(pcl::PointCloud<PointT>::Ptr, cv::Mat &, cv::Mat &);
   bool processRGBDImage(Mat &, Mat &, Mat &);
   void pclDistanceFilter(boost::shared_ptr<pcl::PCLPointCloud2> , pcl::PCLPointCloud2 & );
  
   void pointCloudObjectRegionSegmentation(pcl::PointCloud<PointT>::Ptr, cv::Rect, cv::Mat &, bool);
   bool projectRGBDContour2PointCloud(pcl::PointCloud<PointT>::Ptr, vector<vector<Point> > &); 
   void getContourFromSelectedObjectRegion(Mat &, Rect, vector<vector<Point> > &);
   void process3DObjectPointCloud(pcl::PointCloud<PointT>::Ptr);
   
   void pclObjectCentroidBroadcast(pcl::PointCloud<PointT>::Ptr, Rect );
};
#endif /* defined(__boundary_estimation__Point_Cloud_Processing__) */
