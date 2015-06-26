//
//  Color Histogram Descriptors.cpp
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 11/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/Point_Cloud_Processing.h"

/************************************************************************************************************
  Function create model template on the file
************************************************************************************************************/
void PointCloudProcessing::makeObjectTemplates(cv::Mat &image)
{
   if(image.empty())
   {
      ROS_ERROR("Image cannot be used to make template");
      exit(-1);
   }

   string path_ = "/home/krishneel/catkin_ws/src/boundary_estimation/";
   string logFilename = "obj_template.txt";

   ROS_INFO("Checking Template status...");
   bool is_exist = this->fileExists(path_ + logFilename);
   
   if(!is_exist)
   {
      ofstream outFile((path_ + logFilename).c_str(), ios::out);
      
      ROS_INFO("First create a template of tomato.");
      ROS_INFO("Enter # of tomato template to create:");
      int tmp_num;
      cin >> tmp_num;

      ROS_INFO("Please select the tomato region to make a template batch files.");
      int condition = 0;
      do
      {
         cv::Rect rect_ = this->cvGetobjectBoundary(image);
         Mat roi = image(rect_);

         imwrite(path_ + "image_" + convertNumber2String(condition) + ".jpg", roi);
         outFile << "image_" + convertNumber2String(condition) + ".jpg" << endl;
         
         
      }while (condition++ < tmp_num);
      outFile.close();
      ROS_INFO("Template created successfully in .ros folder");
      
   }
   else
   {
      ROS_INFO("Object template OK.");
   }
}

/************************************************************************************************************
  Function to check if files exits in the directory
************************************************************************************************************/
bool PointCloudProcessing::fileExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}


/************************************************************************************************************
 * Function to subscibe to the kinect RGB image topic
/************************************************************************************************************/
void PointCloudProcessing::imageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
   try
   {
      this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }
}

/************************************************************************************************************
  Function handler to sensor message
************************************************************************************************************/
void PointCloudProcessing::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
   boost::shared_ptr<pcl::PCLPointCloud2> cloud (new pcl::PCLPointCloud2);
   // boost::shared_ptr<pcl::PCLPointCloud2> cloud_filtered (new pcl::PCLPointCloud2);
   
   //! convert sensor message to point cloud
   pcl_conversions::toPCL (*cloud_msg, *cloud);
   
   
   //! Filter point cloud using depth distance
   pcl::PCLPointCloud2 *cloud_filtered = new pcl::PCLPointCloud2;
   this->pclDistanceFilter(cloud, *cloud_filtered);
   
   
   //! convert PCLPointcloud to pcl type
   pcl::PointCloud<PointT>::Ptr pcl_cloud (new pcl::PointCloud<PointT>);
   pcl::fromPCLPointCloud2 (*cloud_filtered, *pcl_cloud);
   
   //! Create a copy of the original Point cloud for processing
   pcl::PointCloud<PointT>::Ptr copy_pcl_cloud__ (new pcl::PointCloud<PointT>);
   pcl::copyPointCloud<PointT, PointT> (*pcl_cloud, *copy_pcl_cloud__);


   Mat kinect_img;
   if(!this->cv_ptr->image.empty())
   {
      kinect_img = this->cv_ptr->image.clone(); 
      //imshow("kinect", kinect_img);
   }
   
   /***************************************************************************/
   bool isProcessThis = false;
   if(!pcl_cloud->empty() && !isProcessThis)
   {
     //! Project 3D PointCloud data to 2D RGB-D data for processing
     Mat depthImage__;
     Mat rgbImage__;
     this->pointCloud2RGBDImage(pcl_cloud, rgbImage__, depthImage__);

     // if(this->maketemplate)
     // {
     //    this->makeObjectTemplates(rgbImage__);
     //    this->maketemplate = false;
     // }
     
     bool isObjectRegion = this->processRGBDImage(kinect_img, rgbImage__, depthImage__);
     
     //--------------------------------------------------
     //! Process only if the object is detected on 2D image space
     if(isObjectRegion)
     {
       //! detected object region bounding box and contour
       vector<Rect> obj_region__;
       vector<vector<Point> > obj_contour__;
       this->getObjectRegion(obj_region__, obj_contour__);
       
       //! in extension change to based detection loop
       int _index = 0;
       //this->pointCloudObjectRegionSegmentation(pcl_cloud, obj_region__[_index], depthImage__, true);
       //pcl::io::savePCDFileASCII("/home/krishneel/Desktop/tomato_1.pcd", *pcl_cloud);
       
       //! Do not process if the cloud is empty
       if(!pcl_cloud->empty())
       {
         //! Process filtered 3D point cloud of the target object
         //process3DObjectPointCloud(pcl_cloud);
         /*
         obj_contour__.clear();       //! Get contour around the selected object region
         this->getContourFromSelectedObjectRegion(depthImage__, obj_region__[_index], obj_contour__);
         //***this->projectRGBDContour2PointCloud(copy_pcl_cloud__, obj_contour__);


         //! ---> transform the detected object centroid to point cloud
         this->pclObjectCentroidBroadcast(copy_pcl_cloud__, obj_region__[_index]);
         */
       }
     }
     //--------------------------------------------------
   }
   /***************************************************************************/
   
   /* Convert pcl::PointCloud to PCLPointCloud2 type */
   boost::shared_ptr<pcl::PCLPointCloud2> pcl2PCL2 (new pcl::PCLPointCloud2);
   //pcl::toPCLPointCloud2(*pcl_cloud, *pcl2PCL2);
   pcl::toPCLPointCloud2(*copy_pcl_cloud__, *pcl2PCL2);
   
   
   //! convert PCL to ros message
   sensor_msgs::PointCloud2 output;
   pcl_conversions::fromPCL(*pcl2PCL2, output);
   
   //! Publish the data 
   publisher__.publish (output);

   //! publish the rgb image frame
   cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);
   out_msg->header = this->cv_ptr->header;
   out_msg->encoding = sensor_msgs::image_encodings::BGR8;
   out_msg->image = kinect_img;
   image_pub_.publish(out_msg->toImageMsg());
}

/************************************************************************************************************
  Function to filter pointcloud data based on the distance data
************************************************************************************************************/
 void PointCloudProcessing::pclDistanceFilter(boost::shared_ptr<pcl::PCLPointCloud2> cloud,
                                             pcl::PCLPointCloud2 &cloud_filtered)
{
   pcl::PCLPointCloud2ConstPtr cloudPtr (cloud);
   pcl::PassThrough<pcl::PCLPointCloud2> pass;
   pass.setInputCloud(cloudPtr);
   pass.setFilterFieldName ("z");
   pass.setKeepOrganized(true);
   pass.setFilterLimits(0, this->MAX_DISTANCE);
   pass.filter(cloud_filtered);
}

/************************************************************************************************************
  Function to process 2D image for object detection. Note inpute are distance filted region
************************************************************************************************************/
bool PointCloudProcessing::processRGBDImage(Mat &kframe,
                                            Mat &rgbImg,
                                            Mat &depthImg)
{
   if(!rgbImg.data)
   {
      std::cout << "Error processing..." << std::endl;
      exit(-1);
   }
   bool isObjectRegion = true;
   
   //! sliding window object detection
   this->slidingWindowDetector(rgbImg);
   
   //! detected object region bounding box and contour
   vector<Rect> obj_region__;
   vector<vector<Point> > obj_contour__;
   
   this->getObjectRegion(obj_region__, obj_contour__);
   
   //! incase no object detected
   if(obj_region__.empty() || obj_contour__.empty())
   {
     isObjectRegion = false;
   }
   
   for (int i = 0; i < obj_region__.size(); i++) {
      //rectangle(rgbImg, obj_region__[i], Scalar(0, 255, 0), 2);
      rectangle(kframe, obj_region__[i], Scalar(0, 255, 0), 2);
      
      Rect rect = obj_region__[i];
      Point center = Point((rect.x + rect.width/2), (rect.y + rect.height/2));
      //circle(rgbImg, center, 4, Scalar(0, 255, 0), CV_FILLED);
      cvCross(kframe, center, 4);
   }
   //imshow("Object Region", kframe);
   return isObjectRegion;
}

/************************************************************************************************************
  Function to extract 2D rgb and depth image from the point cloud data
************************************************************************************************************/
 void PointCloudProcessing::pointCloud2RGBDImage(pcl::PointCloud<PointT>::Ptr _cloud,
                                                Mat &rgbImage,
                                                Mat &depthImage)
{
   depthImage = Mat(_cloud->height, _cloud->width, CV_8U);
   rgbImage = Mat ::zeros(_cloud->height, _cloud->width, CV_8UC3);

   vector<int> indices;
   for (int j = 0; j < _cloud->height; j++) {
      for (int i = 0; i < _cloud->width; i++) {
         int index = i + (j * _cloud->width);
         float distance_ = _cloud->points[index].z;
        
         // if(distance_ > this->MAX_DISTANCE || distance_ < this->MIN_DISTANCE || distance_ != distance_)
         if (distance_ != distance_)
         {
            depthImage.at<uchar>(j,i) = 0.0f;

            rgbImage.at<Vec3b>(j,i)[2] = 0.0f;
            rgbImage.at<Vec3b>(j,i)[1] = 0.0f;
            rgbImage.at<Vec3b>(j,i)[0] = 0.0f;
         }
         else
         {
            //! Copying the depth data
            //depthImage.at<uchar>(j,i) = (distance_ / this->MAX_DISTANCE) * 255.0;
            depthImage.at<uchar>(j,i) = (this->MAX_DISTANCE - distance_ / this->MAX_DISTANCE) * 255.0;
            
            //! Copying the RGB data from the Point Cloud
            rgbImage.at<Vec3b>(j,i)[2] = _cloud->points[index].r;
            rgbImage.at<Vec3b>(j,i)[1] = _cloud->points[index].g;
            rgbImage.at<Vec3b>(j,i)[0] = _cloud->points[index].b;

            indices.push_back(index);
         }
      }
   }  
   this->setFilteredIndexFromDepth(indices);
   //imshow("rgb", rgbImage);
   //imwrite("/home/krishneel/Desktop/model.jpg", rgbImage);
   //imshow("depth", depthImage);
   waitKey(3);
}


/************************************************************************************************************
  Function get the indices of 2D filtered points 
************************************************************************************************************/
 void PointCloudProcessing::setFilteredIndexFromDepth(vector<int> &indices)
{
   this->__indices__ = indices;
}

/************************************************************************************************************
  Function to extract the index of tomato on the point cloud
************************************************************************************************************/
 void PointCloudProcessing::getPointCloudRegion(pcl::PointCloud<PointT>::Ptr cloud,
                                               int K)
{
   pcl::KdTreeFLANN<PointT> kdtree;
   kdtree.setInputCloud(cloud);

   vector<vector<int> > pointIdxNKNSearch;
   vector<vector<float> > pointNKNSquaredDistance;
   
   for (int i = 0; i < this->__indices__.size(); i++) 
   {
      int index = this->__indices__[i];
      PointT seed_point = cloud->points[index];

      vector<int> pointIdxN;
      vector<float> pointNKSqDist;
      kdtree.nearestKSearch(seed_point, K, pointIdxN, pointNKSqDist);

      pointIdxNKNSearch.push_back(pointIdxN);
      pointNKNSquaredDistance.push_back(pointNKSqDist);
   }
}

/************************************************************************************************************
  Function to plot 2D processed indices to the 3D pointcloud
************************************************************************************************************/
 pcl::PointCloud<PointT>::Ptr  PointCloudProcessing::filterPointCloudby2DIndices(const pcl::PointCloud<PointT>::Ptr _cloud,
                                                                                bool isProcess) 
{
   pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
   filtered_cloud->reserve(this->__indices__.size());
   
   std::cout << "\033[1;31m\t\t .... filtering pointcloud..... \033[0m" << _cloud->size() << std::endl;
   //std::cout << "Indicies Size: " << __indices__.size()  << std::endl;

   if(isProcess & !this->__indices__.empty())
   {
      for (int i = 0; i < this->__indices__.size(); i++) {
         int index = this->__indices__[i];
         PointT pt = _cloud->points[index];
         filtered_cloud->push_back(pt);
      }
   }   
   //pcl::io::savePCDFileASCII("/home/krishneel/Desktop/depth.pcd", *_cloud);
   //std::cout << "Filtered Size... " << filtered_cloud->size() << std::endl;
   return filtered_cloud;
}


/************************************************************************************************************
 Function to segment the object region point cloud. Note the return unorganised point cloud
 is used to compute the object BOUNDARY

 @ _cloud      - Raw point cloud 
 @ _region     - Region around the target object estimated by 2D RGB-D data
 @ _depthImage - 2D depth map 
 @ _isMaskDepth - if the 2D depthMap is to be filtered too
************************************************************************************************************/
 void PointCloudProcessing::pointCloudObjectRegionSegmentation(pcl::PointCloud<PointT>::Ptr _cloud,
                                                              Rect _region,
                                                              Mat &_depthImage,
                                                              bool _isMaskDepth)
{
   if(_cloud->height == sizeof(char))
   {
      std::cout << "ERROR! In Segmentation due to non-regular size..." << std::endl;
      exit(-1);
   }
   //! copy of the cloud
   pcl::PointCloud<PointT>::Ptr n_cloud (new pcl::PointCloud<PointT>);
   this->setInputPointCloud(_cloud);
   this->removeNaNPointCloud();
   
   vector<int> index__ = getFilteredPointCloudIndex();
   this->__pointCloudIndices__ = index__;
   this->pcl3D_2D_index__.clear();
   
   //! Convert the filtered data to 2D RGB space
   Mat image__ = Mat::zeros(sizeof(char), _cloud->height * _cloud->width, CV_8UC3);
   Mat depth__ = Mat::zeros(sizeof(char), _depthImage.rows * _depthImage.cols, CV_8U);

     
   for (int k = 0; k < index__.size(); k++) {
      int width__ = _region.x + _region.width;
      int height__ = _region.y + _region.height;
      int x__ = _region.x;
      int y__ = _region.y;
      
      //! condition if the size of _region is larger than image size
      if(width__ > _cloud->width)
      {
         width__ -= (width__ - _cloud->width);
      }
      else if(_region.x < 0)
      {
         x__ = 0;
      }
      
      if(height__ > _cloud->height)
      {
         height__ -= (height__ - _cloud->height);
      }
      else if (_region.y < 0)
      {
         y__ = 0;
      }

      //! _region greater than image space
      if(_region.width > _cloud->width || _region.height > _cloud->height)
      {
         std::cout << "ERROR!.. Object Region greater than image size..." << std::endl;
         break;
      }
      
      for (int j = y__; j < height__; j++) {
         for (int i = x__; i < width__; i++)
         {
            int idx = i + (j * _cloud->width);
            if(idx == index__[k])
            {
               image__.at<Vec3b>(index__[k])[0] = _cloud->points[index__[k]].b;
               image__.at<Vec3b>(index__[k])[1] = _cloud->points[index__[k]].g;
               image__.at<Vec3b>(index__[k])[2] = _cloud->points[index__[k]].r;

               //! filter point cloud
               n_cloud->push_back(_cloud->points[index__[k]]);
               
               if(_isMaskDepth)
               {
                  depth__.at<uchar>(index__[k]) = _depthImage.at<uchar>(index__[k]);
               }

               //! Save the 3D 2D index
               Cloud3D2DIndex cIdx;
               cIdx.index = index__[k];
               cIdx.x = i;
               cIdx.y = j;
               this->pcl3D_2D_index__.push_back(cIdx);

               break;
            }
         }
      }
   }
   image__ = image__.reshape(0, _cloud->height);
   _depthImage = depth__.reshape(0, _cloud->height);
   
   //imshow("After Filter RGB", image__);
   //imshow("After depth", _depthImage);
   
   std::cout << "Filtered Cloud size: " << n_cloud->size() << "\t" << index__.size() << std::endl;
   _cloud->clear();
   _cloud->reserve(n_cloud->size());
   pcl::copyPointCloud<PointT, PointT> (*n_cloud, *_cloud);
}

/************************************************************************************************************
  Function to project the object cluster contour from 2D RGB-D data to the Point cloud data
************************************************************************************************************/
 bool PointCloudProcessing::projectRGBDContour2PointCloud(pcl::PointCloud<PointT>::Ptr _cloud,
                                                         vector<vector<Point> > &_contours)
{
   if(_cloud->height == sizeof(char))
   {
      ROS_ERROR( "\033[1;31m ERROR: Projecting the contour from 2D to Point Cloud...! \033[0m");
      return false;
   }
   else
   {
      for (int j = 0; j < _contours.size(); j++) {
         for (int i = 0; i < _contours[j].size(); i++) {
            int contour_index = _contours[j][i].x + (_contours[j][i].y * _cloud->width);
            
            _cloud->points[contour_index].r = 0;
            _cloud->points[contour_index].g = 255;
            _cloud->points[contour_index].b = 0;
         }
      }
   }
}

/************************************************************************************************************
  Function to compute contour around the selected object region
************************************************************************************************************/
 void PointCloudProcessing::getContourFromSelectedObjectRegion(Mat &_depthImage,
                                                              Rect _rect,
                                                               vector<vector<Point> > &_contours)
{
   _contours.clear();
   vector<Vec4i> hierarchy;

   Mat canny_output;
   Canny(_depthImage, canny_output, 10, 50);
   findContours(canny_output, _contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

   Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
   for (int i = 0; i < _contours.size(); i++) {
      drawContours(drawing, _contours, i, Scalar(0,255,255), 2, 8, hierarchy, 0, Point());
   }
   //imshow("Canny out", canny_output);
   //imshow("Input Depth", _depthImage);
   //imshow("Depth region", drawing);
}

/************************************************************************************************************
  Function to process the target object 3D point cloud for normal, object region and boundary
************************************************************************************************************/
 void PointCloudProcessing::process3DObjectPointCloud(pcl::PointCloud<PointT>::Ptr _cloud)
{
   if(!_cloud->empty())
   {
      //! compute point cloud normal
      pcl::PointCloud<pcl::Normal>::Ptr normal__(new pcl::PointCloud<pcl::Normal>);
      this->setInputPointCloud(_cloud);
      this->computePointCloudSurfaceNormal(normal__, 0, 0.01f, false);

      //! get values for probable boundary estimation
      vector<float> _boundary_vals;
      objectIntraBoundaryEstimation(normal__, _boundary_vals);
      
      //std::cout << "Size: " << _boundary_vals.size() << "\t" << this->pcl3D_2D_index__.size() << std::endl;
      if(_boundary_vals.size() == this->pcl3D_2D_index__.size()
         && !_boundary_vals.empty() & !this->pcl3D_2D_index__.empty())
      {
         Mat plotMD = Mat::zeros(480, 640, CV_8UC3);
         for (int i = 0; i < _boundary_vals.size(); i++) {
            int x = this->pcl3D_2D_index__[i].x;
            int y = this->pcl3D_2D_index__[i].y;
            
            plotMD.at<Vec3b>(y, x)[0] = _boundary_vals[i] * 255.0f;

            plotMD.at<Vec3b>(y, x)[1] = normal__->points[i].normal_x * 255.0f;
            plotMD.at<Vec3b>(y, x)[2] = normal__->points[i].normal_y * 255.0f;
         }
         //imshow("Plotted Region", plotMD);
      }
   }
}


/************************************************************************************************************
  Function to project the 2D estimated object centroid to the point cloud data
************************************************************************************************************/
 void PointCloudProcessing::pclObjectCentroidBroadcast(pcl::PointCloud<PointT>::Ptr _cloud, Rect rect)
{
   //! center of the detected object region
   int centroid_index__ = ((rect.x + rect.width/2) + (rect.y + rect.height/2));

   for (int i = 0; i < this-> pcl3D_2D_index__.size(); i++) {
      if((this->pcl3D_2D_index__[i].x == (rect.x + rect.width/2)) &&
         (this->pcl3D_2D_index__[i].y == (rect.y + rect.height/2)))
      {
         int center_index__ = this->pcl3D_2D_index__[i].index;

         std::cout << "\t\t Index: " << center_index__ << std::endl;
         //! Broadcast routine
         static tf::TransformBroadcaster br;
         tf::Transform transform;
         transform.setOrigin( tf::Vector3(_cloud->points[center_index__].x,
                                          _cloud->points[center_index__].y,
                                          _cloud->points[center_index__].z) );
         tf::Quaternion q;
         q.setRPY(0, 0, 0);
         transform.setRotation(q);
         br.sendTransform(tf::StampedTransform(transform,
                                               ros::Time::now(),
                                               "camera_rgb_optical_frame" ,
                                               "tomato"));
         break;
      }
   }
}

