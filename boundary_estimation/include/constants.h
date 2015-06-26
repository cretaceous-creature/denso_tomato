//
//  constants.h
//  Object boundary estimation
//
//  Created by Chaudhary Krishneel on 11/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __boundary_estimation__constants__
#define __boundary_estimation__constants__

#include <iostream>
#include <fstream>
#include <sys/stat.h>
using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;


#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>

#include <tf/transform_broadcaster.h>

//! image transport ROS OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

typedef pcl::PointXYZRGB PointT;

/********************************************************************
//! Struct for meta table for storing the point cloud index
 and 2D image position
********************************************************************/
struct Cloud3D2DIndex
{
   int index;
   int x;
   int y;
};


inline void cvCross(Mat &image, Point pt, int lnght = 3, int thickness = 1)
{
   line(image, pt, Point(pt.x, pt.y + lnght), Scalar(255,0,0), thickness);
   line(image, pt, Point(pt.x, pt.y - lnght), Scalar(255,0,0), thickness);
   line(image, pt, Point(pt.x + lnght, pt.y), Scalar(255,0,0), thickness);
   line(image, pt, Point(pt.x - lnght, pt.y), Scalar(255,0,0), thickness);
}

/**
 * Function to convert Numbers to string to string
 */
template<typename T>
inline std::string convertNumber2String(T c_frame)
{
   std::string frame_num;
   std::stringstream out;
   out << c_frame;
   frame_num = out.str();

   return frame_num;
}

#endif 
