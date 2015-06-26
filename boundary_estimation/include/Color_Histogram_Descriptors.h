//
//  Color Histogram Descriptors.h
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __Handheld_Object_Tracking__Color_Histogram_Descriptors__
#define __Handheld_Object_Tracking__Color_Histogram_Descriptors__

//#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;


class ColorHistogramDescriptors {

private:
    static const int h_bins = 20;
    static const int s_bins = 20;
    
public:
    /* compute 3 channel Image histogram */
    void imageHistogram             (Mat &, Mat &, int ch1 CV_DEFAULT(180), int ch2 CV_DEFAULT(256), int ch3 CV_DEFAULT(256));
    void HueSaturationHistogram     (Mat, Mat &);
    /* Function to compute the HS histogram */
    void computeHistogramDescriptor (const Mat &, Mat &);
    /* Function for computing histogram similarities */
    void computeHistogram           (Mat &, Mat &, int hBin CV_DEFAULT(64), int sBin CV_DEFAULT(32), bool isNormalized CV_DEFAULT(true));
    /* Function to compute the gray level histogram */
    void grayLevelHistogram         (Mat &, Mat &);
};

#endif /* defined(__Handheld_Object_Tracking__Color_Histogram_Descriptors__) */
