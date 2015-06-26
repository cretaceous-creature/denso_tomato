//
//  Histogram of Oriented Gradients.h
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#ifndef __Handheld_Object_Tracking__Histogram_of_Oriented_Gradients__
#define __Handheld_Object_Tracking__Histogram_of_Oriented_Gradients__

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;



class HOGFeatureDescriptors {

protected:
    
#define N_BINS      9
#define ANGLE       360.0
#define BINS_ANGLE  40.0
#define CELL        8
#define BLOCK       2
    
private:
    
    void bilinear_interpolation         (float, int &, int &);
    void image_gradient                 (Mat &, Mat &);
    Mat  blockGradient                  (int, int, Mat &);
    void getHOG                         (Mat &, Mat &, Mat &);
    
public:
    
    Mat computeHOG                      (const Mat &);
    Mat orientation_histogram           (const Mat&, int CV_DEFAULT(0), int CV_DEFAULT(255), bool CV_DEFAULT(false));
    double computeHOGHistogramDistances (Mat &, vector<Mat>);
};

#endif /* defined(__Handheld_Object_Tracking__Histogram_of_Oriented_Gradients__) */
