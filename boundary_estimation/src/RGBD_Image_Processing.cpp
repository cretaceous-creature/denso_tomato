//
//  Color Histogram Descriptors.cpp
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 11/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/RGBD_Image_Processing.h"

/************************************************************************************************************
  Function to search 2D image space using sliding window method 
************************************************************************************************************/
 void RGBDImageProcessing::slidingWindowDetector(const Mat &img)
{
   //ROS_INFO( "Sliding Window Object detector....");
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
   const int PADDING_EDGE = 0;
   for (int j = 0 /*PADDING_EDGE*/; j < image.rows/* - PADDING_EDGE*/; j += (this->SLIDING_WINDOW/scale)) {
      for (int i = 0 /*PADDING_EDGE*/; i < image.cols/* - PADDING_EDGE*/; i += (this->SLIDING_WINDOW/scale))
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
               double distance__color = compareHist(this->tmpl_colorMD__,
                                                    colorMD_hist,
                                                    CV_COMP_BHATTACHARYYA);
               // double distance__hog  = compareHist(this->tmpl_hogMD__, hogMD_hist, CV_COMP_BHATTACHARYYA);

               if(distance__color <= 1)
               {
                  likelihood = exp(SCALE_FACTOR * distance__color);// * exp(SCALE_FACTOR * (distance__hog));
               }
            }            
            double d = 1/likelihood;
            //!!likelihood += GaussianKernel(d);

            if(i > 0 && j > 0)
            {
               likelihood = (likelihood + prev_prob)/2;
            }
            prev_prob = likelihood;
            
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
   //imshow("Likelihood Map", likelihoodMap);
   //ROS_INFO( "Object detection completed....");
}

/************************************************************************************************************
  Function to compute 2D image features using HS color and HOG 
************************************************************************************************************/
 void RGBDImageProcessing::getFeatureMat(Mat &roi, Mat &colorMD_hist, Mat &hogMD_hist, bool isHog)
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
 void RGBDImageProcessing::loadTemplateModel()
{
   //string path_ = "/home/krishneel/catkin_ws/src/boundary_estimation/";
   string logFilename = "image_1.jpg";

   //Mat tmpl_img = imread(path_ + logFilename, CV_LOAD_IMAGE_COLOR);
   Mat tmpl_img = imread(logFilename, CV_LOAD_IMAGE_COLOR);
   if(!tmpl_img.data)
   {     
      ROS_ERROR("ERROR! No Template Image Found \n Loading Failed... ");
      return;
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
 void RGBDImageProcessing::estimateObjectRegion(Mat &image, Mat &probabilityMap)
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

      rectangle(image, boundingBox, Scalar(0,255,0), 2);
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
void RGBDImageProcessing::getObjectRegion(vector<Rect> &detected_region, vector<vector<Point> > &region_contours) 
{
   detected_region.clear();
   region_contours.clear();
   detected_region = this->object_region__;
   region_contours = this->contours;
}

/************************************************************************************************************
  Function for compute Gaussian weight 
************************************************************************************************************/
 double RGBDImageProcessing::GaussianKernel(double d, double sigma, double s)
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

