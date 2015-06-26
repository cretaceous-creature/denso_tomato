//
//  RGBD_Image_Processing.h
//  boundary_estimation
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __boundary_estimation__RGBD_Image_Processing__
#define __boundary_estimation__RGBD_Image_Processing__

#include "../include/constants.h"
#include "../include/Color_Histogram_Descriptors.h"
#include "../include/Histogram_of_Oriented_Gradients.h"


class RGBDImageProcessing: public ColorHistogramDescriptors, public HOGFeatureDescriptors
{
private:
   
   Mat tmpl_colorMD__;
   Mat tmpl_hogMD__;

   //! downsampling scale size
   int downSample;
   
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
  
   RGBDImageProcessing(int dw_sm, const int thresh_ = 4) :
      downSample(dw_sm),
      object_region__(0),
      threshold__(0.50f),
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


#endif /* defined(__boundary_estimation__RGBD_Image_Processing__) */
