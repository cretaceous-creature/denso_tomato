//
//  Color Histogram Descriptors.cpp
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 11/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/Color_Histogram_Descriptors.h"

//! ---------------------------------------------------------------------------------------------------------------------
//! Function to compute Histogram of Training Dataset
void ColorHistogramDescriptors::computeHistogramDescriptor(const Mat &imageMAT, Mat &featureMD){
    
    featureMD = Mat((int)sizeof(char), (h_bins * s_bins), CV_32FC1);
    Mat img = imageMAT.clone();
    
    Mat imgBin;
    HueSaturationHistogram(img, imgBin);
    //! copy the feature vector..  somehow push creates 2 channel
    
    for (int y = 0; y < imgBin.cols; y++) {
        featureMD.at<float>(0, y) = imgBin.at<float>(0,y);
    }
}
//! ---------------------------------------------------------------------------------------------------------------------
//! Function to compute the H-S histogram bins of size 30 x 32 respectiviely. Input is RGB format image
//! Use the same function for testing
void ColorHistogramDescriptors::HueSaturationHistogram(Mat image, Mat &bin){
    
    Mat hsv;
    cvtColor(image, hsv, CV_BGR2HSV);
    
    int histSize[] = { this->h_bins, this->s_bins };
    Mat hist;
    float h_range[] = { 0, 179 };
    float s_range[] = { 0, 255 };
    
    //    float h_range[] = { 0, 255 };
    //    float s_range[] = { 0, 255 };
    const float* ranges[] = { h_range, s_range };
    
    int channels[] = { 0, 1 };
    
    calcHist( &hsv, 1, channels, Mat(), hist, 2, histSize, ranges, true, false );
    double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);
    
    //! Drawing histogram
    //    int scale = 20;
    //    Mat histImg = Mat::zeros(s_bins*scale, h_bins*scale, CV_8UC3);
    
    //! convert hist to 1D
    bin = Mat(1, (hist.rows * hist.cols), CV_32F);
    int index = 0;
    for( int h = 0; h < h_bins; h++ ){
        for( int s = 0; s < s_bins; s++ ){
            float binVal = hist.at<float>(h, s);
            float intensity = binVal /(double)maxVal;
            bin.at<float>(0, index++) = (float)intensity;
            //            cout << intensity << "  ";
            //            rectangle( histImg, Point(h*scale, s*scale), Point( (h+1)*scale - 1, (s+1)*scale - 1), Scalar::all(intensity), CV_FILLED );
        }
    }
    //    cout << bin << endl << endl;
    //    cout << hist.size() << bin.size() << endl;
    //    imshow( "H-S Histogram", histImg );
}

//! ---------------------------------------------------------------------------------------------------------------------
//! Function to compute the RGB color histogram
void ColorHistogramDescriptors::imageHistogram(Mat &src, Mat &bin, int ch1, int ch2, int ch3){
    
    vector<Mat> bgr_planes;
    split( src, bgr_planes );
    
    /// Establish the number of bins
//    int histSize = 8;
    
    /// Set the ranges ( for B,G,R) )
    float h_range[] = {0, 256};
    const float * h_histRange = {h_range};
    
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    
    bool uniform = true;
    bool accumulate = false;
    
    Mat b_hist;
    Mat g_hist;
    Mat r_hist;
    
    /// Compute the histograms:
    calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &ch1, &h_histRange, uniform, accumulate );
    calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &ch2, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &ch3, &histRange, uniform, accumulate );
    
    normalize(b_hist, b_hist, 0, 512, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, 512, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, 512, NORM_MINMAX, -1, Mat() );
    
    
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/ch3 );

     Mat histImage( 480, 640, CV_8UC3, Scalar( 0,0,0) );
    /// Draw for each channel
    for( int i = 1; i < ch1; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
             Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
             Scalar( 255, 0, 0), 2, 8, 0  );
    }

    for( int i = 1; i < ch3; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
             Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
             Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
             Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
             Scalar( 0, 0, 255), 2, 8, 0  );
    }

    
    /// Display
    namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    imshow("calcHist Demo", histImage );
    
    waitKey(0);

}

//! ---------------------------------------------------------------------------------------------------------------------
//! Function to compute HS Histogram
void ColorHistogramDescriptors::computeHistogram(Mat &src, Mat &hist, int hBin, int sBin, bool isNormalized){
    
    Mat hsv;
    cvtColor(src, hsv, CV_BGR2HSV);
    
    int histSize[] = {hBin, sBin};
    float h_ranges[] = {0, 180};
    float s_ranges[] = {0, 256};
    
    const float* ranges[] = {h_ranges, s_ranges};
    
    int channels[] = {0,1};
    calcHist(&hsv, 1, channels, Mat(), hist, 2, histSize, ranges, true, false);
    
    if (isNormalized) {
        normalize(hist, hist, 0, 1, NORM_MINMAX, -1, Mat());
    }
}


//! ---------------------------------------------------------------------------------------------------------------------
//! Function to compute the gray level histogram of the image
void ColorHistogramDescriptors::grayLevelHistogram(Mat &src, Mat &bin){
    
    if (src.type() != CV_8U) {
        cvtColor(src, src, CV_BGR2GRAY);
    }
    /// Establish the number of bins
    int histSize = 256;
    
    float h_range[] = {0, 256};
    const float * h_histRange = {h_range};
    
    bool uniform = true;
    bool accumulate = false;
    
    Mat b_hist;
    
    /// Compute the histograms:
    calcHist( &src, 1, 0, Mat(), b_hist, 1, &histSize, &h_histRange, uniform, accumulate );
    normalize(b_hist, b_hist, 0, 1, NORM_MINMAX, -1, Mat() );
}
