
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>

#include <tf/transform_broadcaster.h>

#include "../include/Color_Histogram_Descriptors.h"
#include "../include/Histogram_of_Oriented_Gradients.h"

using namespace cv;
using namespace std;
typedef pcl::PointXYZRGB PointT;

/************************************************************************************************************/
//! Struct for meta table for storing the point cloud index and 2D image position
/************************************************************************************************************/
struct Cloud3D2DIndex
{
   int index;
   int x;
   int y;
};

/************************************************************************************************************/
class RGBDImageProcessing: public ColorHistogramDescriptors, public HOGFeatureDescriptors
{
private:
   
   Mat tmpl_colorMD__;
   Mat tmpl_hogMD__;

   //! downsampling scale size
   int downSample;

   //! Path to the template
   string template_path__;
   
   //! memeory to hold the region of the detected object
   vector<Rect> object_region__;
   vector<vector<Point> > contours;
   
   double GaussianKernel(double , double = 1.0, double = 2.0);
   void loadTemplateModel();

   //! Template sliding window size
   int SLIDING_WINDOW;
  
   //! Masking threshold
   float threshold__;
   int scale__;
   /*Note: Later extend the framework to multiple template model and region*/
  
public:
  
   RGBDImageProcessing(string templ_path, int dw_sm, const int thresh_ = 4) :
      downSample(dw_sm),
      template_path__(templ_path),
      object_region__(0),
      threshold__(0.55f),
      tmpl_hogMD__(Mat()),
      scale__(thresh_),
      tmpl_colorMD__(Mat())
   {
      std::cout << "\033[3;31mLoading 2D Object Template..... \033[0m"  <<std::endl;
      this->loadTemplateModel();
      std::cout << "\033[1;31mObject template loaded successfully! \033[0m" << std::endl;





   }
  
   void getObjectRegion(vector<Rect> &, vector<vector<Point> > &);
   void getFeatureMat(Mat &, Mat &, Mat &, bool = true);
   void slidingWindowDetector(const Mat &);
   void estimateObjectRegion(Mat &, Mat &);
};

/************************************************************************************************************
  Function to search 2D image space using sliding window method 
************************************************************************************************************/
inline void RGBDImageProcessing::slidingWindowDetector(const Mat &img)
{
   //! Clear previous buffer
   this->object_region__.clear();
   
   Mat image;
   resize(img, image, Size(img.cols/this->downSample, img.rows/this->downSample));
   
   int scale = this->scale__;
   double prev_prob = 0.0;
   double SCALE_FACTOR = -1.0;
   
   Mat likelihoodMap = Mat::zeros(image.rows, image.cols, CV_32F);
   Mat probabilityMap = Mat::zeros(image.rows, image.cols, CV_8U);
   
   //! Specify the padding in the image boundaries
   const int PADDING_EDGE = 40;
   for (int j = 0 /*PADDING_EDGE*/; j < image.rows/2/* - PADDING_EDGE*/; j += (this->SLIDING_WINDOW/scale)) {
     for (int i = PADDING_EDGE; i < image.cols - PADDING_EDGE; i += (this->SLIDING_WINDOW/scale))
      {
         Rect rect = Rect (i, j, this->SLIDING_WINDOW, this->SLIDING_WINDOW);
         
         if((rect.x + rect.width < image.cols) && (rect.y + rect.height < image.rows))
         {
            Mat roi = image(rect).clone();

            bool isBlackRegion = true;
            //! ADD! checck for black pixel only region and skip it
            for (int y = 0; y < roi.rows; y++) {
               for (int x = 0; x < roi.cols; x++) {
                  int r = static_cast<int>(roi.at<Vec3b>(y,x)[2]);
                  int g = static_cast<int>(roi.at<Vec3b>(y,x)[1]);
                  int b = static_cast<int>(roi.at<Vec3b>(y,x)[0]);

                  if(r > 0 && g > 0 && b > 0)
                  {
                     isBlackRegion = false;
                     break;
                  }
               }
            }

            double likelihood = 0.0;
            if(!isBlackRegion)
            {
               Mat hogMD_hist;
               Mat colorMD_hist;
               this->getFeatureMat(roi, colorMD_hist, hogMD_hist, false);

               double distance__color = compareHist(this->tmpl_colorMD__, colorMD_hist, CV_COMP_BHATTACHARYYA);
               //double distance__hog  = compareHist(this->tmpl_hogMD__, hogMD_hist, CV_COMP_BHATTACHARYYA);

               if(distance__color < 1)
               {
                  likelihood = exp(SCALE_FACTOR * distance__color);// * exp(SCALE_FACTOR * (distance__hog));
               }
            }            
            double d = 1/likelihood;
            likelihood += GaussianKernel(d);
            
            if(i > 0 && j > 0)
            {
               likelihood = (likelihood + prev_prob)/2;
            }
            prev_prob = likelihood;
            //std::cout << "\033[2;31mLikelihood Value:  \033[2m" << distance__color << "\t" << distance__hog<<std::endl;
            
            if(likelihood < this->threshold__)
            {
               likelihood = 0;
            }
            else
            {
              rectangle(probabilityMap, rect, Scalar(255), CV_FILLED);
            }
            //! Plot the probablity map
            rectangle(likelihoodMap, rect, Scalar(likelihood), CV_FILLED);
         }
      }
   }
   Mat image_ = img.clone();
   this->estimateObjectRegion(image_, probabilityMap);
   imshow("Likelihood Map", likelihoodMap);
}

/************************************************************************************************************
  Function to compute 2D image features using HS color and HOG 
************************************************************************************************************/
inline void RGBDImageProcessing::getFeatureMat(Mat &roi, Mat &colorMD_hist, Mat &hogMD_hist, bool isHog)
{
   //! Photometrical Features
   computeHistogram(roi, colorMD_hist, 20, 20, true);
   
   //! Geometrical Features
   Mat hogMD;
   if(isHog)
   {
      hogMD = computeHOG(roi);
      hogMD_hist = orientation_histogram(hogMD, 0, 9, true);
   }
}


/************************************************************************************************************
  Function to load template model and compute the template features
************************************************************************************************************/
inline void RGBDImageProcessing::loadTemplateModel()
{
   Mat tmpl_img = imread(this->template_path__, CV_LOAD_IMAGE_COLOR);
   if(!tmpl_img.data)
   {
      std::cout << "\033[1;31m ERROR! No Template Image Found \n Loading Failed... \033[0m"  <<std::endl;
      exit(-1);
   }
   resize(tmpl_img,
          tmpl_img,
          Size(tmpl_img.cols/this->downSample,
               tmpl_img.rows/this->downSample));
   this->getFeatureMat(tmpl_img, this->tmpl_colorMD__, this->tmpl_hogMD__);
   //! set the sliding window size
   this->SLIDING_WINDOW = max(tmpl_img.rows, tmpl_img.cols);
}

/************************************************************************************************************
  Function to plot the 2D object region of 2D image space 
************************************************************************************************************/
inline void RGBDImageProcessing::estimateObjectRegion(Mat &image, Mat &probabilityMap)
{
   //! adjment for the bounding box
  const int ADJUSTMENT = 0;
  
   Mat prob;
   vector<Vec4i> hierarchy;
   this->contours.clear();
   
   Canny (probabilityMap, prob, 5, 255);
   findContours(prob, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
   
   for(int i = 0; i < contours.size(); i++)
   {
      Rect boundingBox = boundingRect(contours[i]);
      boundingBox.width += ADJUSTMENT;
      boundingBox.height += ADJUSTMENT;

      //rectangle(image, boundingBox, Scalar(0,255,0), 2);
      //drawContours(image, this->contours, i, Scalar(0,255,0), 2);
      
      //! convert to full image space
      boundingBox.x *= this->downSample;
      boundingBox.y *= this->downSample;
      boundingBox.width *= this->downSample;
      boundingBox.height *= this->downSample;
      
      this->object_region__.push_back(boundingBox);
   }
   //imshow("Bounding Box", image);
}

/************************************************************************************************************
  Function for return the postion of the object on 2D RGB-D space
************************************************************************************************************/
inline void RGBDImageProcessing::getObjectRegion(vector<Rect> &detected_region, vector<vector<Point> > &region_contours) 
{
   detected_region.clear();
   region_contours.clear();
   detected_region = this->object_region__;
   region_contours = this->contours;
}

/************************************************************************************************************
  Function for compute Gaussian weight 
************************************************************************************************************/
inline double RGBDImageProcessing::GaussianKernel(double d, double sigma, double s)
{
   //double sigma = 1.0;
   s = (s * sigma * sigma);
   if(d >= 1.0)
   {
      return 0;
   }
   else
   {
      return (exp(-(d*d)/s))/(M_PI * s);
   }
}





//****************************************************************************************************************************
//! Class to process the 3D point cloud for feature computation
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

/************************************************************************************************************
  Function to return the set input point cloud
************************************************************************************************************/
inline pcl::PointCloud<PointT>::Ptr PointCloudDataAnalysis::getInputPointCloud()
{
   return this->cloud__;
}

/************************************************************************************************************
  Function to return the filtered point cloud index
************************************************************************************************************/
inline vector<int> PointCloudDataAnalysis::getFilteredPointCloudIndex()
{
   return this->index;
}

/************************************************************************************************************
  Function to remove point cloud with NaN value
************************************************************************************************************/
inline void PointCloudDataAnalysis::removeNaNPointCloud()
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
inline void PointCloudDataAnalysis::setInputPointCloud(pcl::PointCloud<PointT>::Ptr _cloud)
{
   this->cloud__ = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
   pcl::copyPointCloud<PointT, PointT>(*_cloud, *cloud__);   
}

/************************************************************************************************************
  Function to compute point cloud surface normal
************************************************************************************************************/
inline void PointCloudDataAnalysis::computePointCloudSurfaceNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal,
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
inline void PointCloudDataAnalysis::objectIntraBoundaryEstimation(pcl::PointCloud<pcl::Normal>::Ptr _normals,
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
inline float PointCloudDataAnalysis::computeVectorScalarProduct(const Eigen::Vector3f _normal_vector,
                                                         const Eigen::Vector3f _reference_vector)
{
   float scalarProduct__ = ((_normal_vector.dot(_reference_vector)) /
                            ((_normal_vector.norm() * _reference_vector.norm())));
   return static_cast<float> (scalarProduct__);
}

/************************************************************************************************************
  Function to compute the nearest neigbor of point cloud Normals
************************************************************************************************************/
inline void PointCloudDataAnalysis::pclNearestNeigbourSearch(vector<vector<int> > &pointIndices,
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





//****************************************************************************************************************************
class PointCloudProcessing : public RGBDImageProcessing, public PointCloudDataAnalysis
{
private:
   ros::NodeHandle nh_;
   ros::Publisher publisher__;
   ros::Subscriber subscriber__;
  
   vector<int> __indices__;
   pcl::PointCloud<PointT>::Ptr pcl_cloud__;
   pcl::PointCloud<PointT>::Ptr filter_cloud__;
  
   //! Variable for distance filter
   const float MAX_DISTANCE;
   const float MIN_DISTANCE;
  
protected:
   vector<int> __pointCloudIndices__;
   vector<Cloud3D2DIndex> pcl3D_2D_index__;
   
public:
  
   PointCloudProcessing() : MAX_DISTANCE(2.0f), MIN_DISTANCE(0.80f),
                            RGBDImageProcessing("/home/krishneel/catkin_ws/src/boundary_estimation/tomato_1.jpg", 1)
   {
      std::cout << "\nSubscribing......\n" << std::endl;
      
      __indices__.clear();
      __pointCloudIndices__.clear();
      pcl3D_2D_index__.clear();
      
      pcl_cloud__ = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
      
      //this->subscriber__ = nh_.subscribe ("/camera/depth_registered/points", 1, &PointCloudProcessing::cloudCallBack, this);
      this->subscriber__ = nh_.subscribe ("/camera_remote/depth_registered/points", 1, &PointCloudProcessing::cloudCallBack,this);
      this->publisher__ = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud/view", 1);
      
      std::cout << "\033[1;31mPoint Cloud Processing constructor..... \033[0m\n" << std::endl;
   }
  
   //! Function to get the index of points from 2D depth map
   void cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &);
   void setFilteredIndexFromDepth(vector<int> &);
   void getPointCloudRegion(pcl::PointCloud<PointT>::Ptr, int = 20);
   pcl::PointCloud<PointT>::Ptr filterPointCloudby2DIndices(const pcl::PointCloud<PointT>::Ptr, bool = false);
   void pointCloud2RGBDImage(pcl::PointCloud<PointT>::Ptr, Mat &, Mat &);
   bool processRGBDImage(Mat &, Mat &);
   void pclDistanceFilter(boost::shared_ptr<pcl::PCLPointCloud2> , pcl::PCLPointCloud2 & );
  
   void pointCloudObjectRegionSegmentation(pcl::PointCloud<PointT>::Ptr, Rect, Mat &, bool);
   bool projectRGBDContour2PointCloud(pcl::PointCloud<PointT>::Ptr, vector<vector<Point> > &); 
   void getContourFromSelectedObjectRegion(Mat &, Rect, vector<vector<Point> > &);
   void process3DObjectPointCloud(pcl::PointCloud<PointT>::Ptr);

   void pclObjectCentroidBroadcast(pcl::PointCloud<PointT>::Ptr, Rect );
};

/************************************************************************************************************
  Function handler to sensor message
************************************************************************************************************/
inline void PointCloudProcessing::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
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
   
   
   /***************************************************************************/
   if(!pcl_cloud->empty())
   {
     //! Project 3D PointCloud data to 2D RGB-D data for processing
     Mat depthImage__;
     Mat rgbImage__;
     this->pointCloud2RGBDImage(pcl_cloud, rgbImage__, depthImage__);
     bool isObjectRegion = this->processRGBDImage(rgbImage__, depthImage__);
     
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
       this->pointCloudObjectRegionSegmentation(pcl_cloud, obj_region__[_index], depthImage__, true);
       //pcl::io::savePCDFileASCII("/home/krishneel/Desktop/tomato_1.pcd", *pcl_cloud);
       
       //! Do not process if the cloud is empty
       
       if(!pcl_cloud->empty())
       {
         //! Process filtered 3D point cloud of the target object
         process3DObjectPointCloud(pcl_cloud);
         
         obj_contour__.clear();       //! Get contour around the selected object region
         this->getContourFromSelectedObjectRegion(depthImage__, obj_region__[_index], obj_contour__);
         //***this->projectRGBDContour2PointCloud(copy_pcl_cloud__, obj_contour__);


         //! ---> transform the detected object centroid to point cloud
         this->pclObjectCentroidBroadcast(copy_pcl_cloud__, obj_region__[_index]);
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
}

/************************************************************************************************************
  Function to filter pointcloud data based on the distance data
************************************************************************************************************/
inline void PointCloudProcessing::pclDistanceFilter(boost::shared_ptr<pcl::PCLPointCloud2> cloud,
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
inline bool PointCloudProcessing::processRGBDImage(Mat &rgbImg,
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
      rectangle(rgbImg, obj_region__[i], Scalar(0, 255, 0), 2);

      Rect rect = obj_region__[i];
      Point center = Point((rect.x + rect.width/2), (rect.y + rect.height/2));
      circle(rgbImg, center, 4, Scalar(0, 255, 0), CV_FILLED);
      //line(rgbImg, center, Point(center.x + 20, center.y), Scalar(255, 0, 255), 1);
   }
   imshow("Object Region", rgbImg);
   
   return isObjectRegion;
}

/************************************************************************************************************
  Function to extract 2D rgb and depth image from the point cloud data
************************************************************************************************************/
inline void PointCloudProcessing::pointCloud2RGBDImage(pcl::PointCloud<PointT>::Ptr _cloud,
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
   imshow("rgb", rgbImage);
   //imwrite("/home/krishneel/Desktop/model.jpg", rgbImage);
   
   imshow("depth", depthImage);
   waitKey(3);
}


/************************************************************************************************************
  Function get the indices of 2D filtered points 
************************************************************************************************************/
inline void PointCloudProcessing::setFilteredIndexFromDepth(vector<int> &indices)
{
   this->__indices__ = indices;
}

/************************************************************************************************************
  Function to extract the index of tomato on the point cloud
************************************************************************************************************/
inline void PointCloudProcessing::getPointCloudRegion(pcl::PointCloud<PointT>::Ptr cloud,
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
inline pcl::PointCloud<PointT>::Ptr  PointCloudProcessing::filterPointCloudby2DIndices(const pcl::PointCloud<PointT>::Ptr _cloud,
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
inline void PointCloudProcessing::pointCloudObjectRegionSegmentation(pcl::PointCloud<PointT>::Ptr _cloud,
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
   
   vector<int> index__ = this->getFilteredPointCloudIndex();
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
inline bool PointCloudProcessing::projectRGBDContour2PointCloud(pcl::PointCloud<PointT>::Ptr _cloud,
                                                         vector<vector<Point> > &_contours)
{
   if(_cloud->height == sizeof(char))
   {
      std::cout << "\033[1;31m ERROR: Projecting the contour from 2D to Point Cloud...! \033[0m" <<  std::endl;
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
inline void PointCloudProcessing::getContourFromSelectedObjectRegion(Mat &_depthImage,
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
   imshow("Depth region", drawing);
}

/************************************************************************************************************
  Function to process the target object 3D point cloud for normal, object region and boundary
************************************************************************************************************/
inline void PointCloudProcessing::process3DObjectPointCloud(pcl::PointCloud<PointT>::Ptr _cloud)
{
   if(!_cloud->empty())
   {
      //! compute point cloud normal
      pcl::PointCloud<pcl::Normal>::Ptr normal__(new pcl::PointCloud<pcl::Normal>);
      this->setInputPointCloud(_cloud);
      this->computePointCloudSurfaceNormal(normal__, 0, 0.01f, false);

      //! get values for probable boundary estimation
      vector<float> _boundary_vals;
      this->objectIntraBoundaryEstimation(normal__, _boundary_vals);
      
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
         imshow("Plotted Region", plotMD);
      }
   }
}


/************************************************************************************************************
  Function to project the 2D estimated object centroid to the point cloud data
************************************************************************************************************/
inline void PointCloudProcessing::pclObjectCentroidBroadcast(pcl::PointCloud<PointT>::Ptr _cloud, Rect rect)
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

/************************************************************************************************************
  Main Program
************************************************************************************************************/
int main(int argc, char** argv)
 {
    ros::init(argc, argv, "PointCloud");
    PointCloudProcessing pcp;
    
    ros::spin();

    return 0;
 }
