//
//  Histogram of Oriented Gradients.cpp
//  Handheld Object Tracking
//
//  Created by Chaudhary Krishneel on 3/11/14.
//  Copyright (c) 2014 Chaudhary Krishneel. All rights reserved.
//

#include "../include/Histogram_of_Oriented_Gradients.h"

//! Based on Histogram Bin center distance to the current orientaion
void HOGFeatureDescriptors::bilinear_interpolation(float angle, int &lower_index, int &higher_index){
    
    float nearest_lower = 1000.0;
    float nearest_higher = 1000.0;
    
    lower_index = 0;
    higher_index = 0;
    for (int i = BINS_ANGLE/2; i < ANGLE; i += BINS_ANGLE) {
        
        float distance = abs(angle - i);
        if (i < angle) {
            if (distance < nearest_lower) {
                nearest_lower = distance;
                lower_index = i;
            }
        }
        else{
            if (distance < nearest_higher) {
                nearest_higher = distance;
                higher_index = i;
            }
        }
    }
}

//! Function computes the image gradient of unsigned bins and perform bilinear interpolation
//! CELL (8) x CELL cell graident orietnation and Magnitude is computed and quantatized into 9bin histogram via bilinear interpolation
void HOGFeatureDescriptors::image_gradient(Mat &image, Mat &hog_bins){
    
    Mat xsobel, ysobel;
    Sobel(image, xsobel, CV_32F, 1, 0, 3);
    Sobel(image, ysobel, CV_32F, 0, 1, 3);
    
    Mat Imag, Iang; //! Angle and Maginitue
    cartToPolar(xsobel, ysobel, Imag, Iang, true);
    
    add(Iang, Scalar(360), Iang, Iang < 0);
    add(Iang, Scalar(-360), Iang, Iang >= 360);
    
    Mat orientation_histogram;
    for (int j = 0; j < image.rows; j += CELL) {
        for (int i = 0; i < image.cols; i += CELL) {
            
            Rect rect = Rect(i, j, CELL, CELL);
            if ((rect.x + rect.width <= image.cols) && (rect.y + rect.height <= image.rows)) {
                
                Mat bin = Mat::zeros(1, N_BINS, CV_32F); //! Matrix with 9 bin quantized histogram of Cell x Cell
                
                //! Nested for loop for Cell x Cell
                for (int y = rect.y; y < (rect.y + rect.height); y++) {
                    for (int x = rect.x; x < (rect.x + rect.width); x++) {
                        float angle = (float)Iang.at<float>(y,x);
                        
                        int l_bin, h_bin;
                        bilinear_interpolation(angle, l_bin, h_bin);
                        
                        //! Distance from the Orientation to Bin Center
                        float l_ratio = 1 - (angle - l_bin)/BINS_ANGLE;
                        float h_ratio = 1 - (angle - h_bin)/BINS_ANGLE;
                        
                        int l_index = (l_bin-(BINS_ANGLE/2))/BINS_ANGLE;
                        int h_index = (h_bin-(BINS_ANGLE/2))/BINS_ANGLE;
                        
                        bin.at<float>(0, l_index) += (Imag.at<float>(y,x) * l_ratio);
                        bin.at<float>(0, h_index) += (Imag.at<float>(y,x) * h_ratio); //! Bilinear interpolation of 9bin histogram
                    }
                }
                orientation_histogram.push_back(bin); //! Each cell 9-Bin Histogram
            }
        }
    }
    hog_bins = orientation_histogram.clone(); //! All image space cell histogram
}


//! Function to compute the 4 cell or 1 block histogram of size 9 * 4 = [36 x 1] features size

Mat HOGFeatureDescriptors::blockGradient(int col, int row, Mat &bins){
    
    Mat block_hogMD = Mat(Size(N_BINS * BLOCK * BLOCK, 1), CV_32F);
    
    int icounter = 0;
    for (int j = row; j < (row + BLOCK); j++) {
        for (int i = col; i < (col + BLOCK); i++) {
            int index = (j * CELL) + i;             //! index of the bin to extract from Histogram Mat (bins)
            
            for (int k = 0; k < N_BINS; k++) {
                block_hogMD.at<float>(0, icounter++) = bins.at<float>(index);
            }
        }
    }
    //    cout << "HOG Size:" << block_hogMD.size() << endl;
    
    return block_hogMD;
}


Mat HOGFeatureDescriptors::orientation_histogram(const Mat& src, int minVal, int maxVal, bool normed ) {
    
    Mat result;
    int histSize = maxVal - minVal;
    
    float range[] = { static_cast<float>(minVal), static_cast<float>(maxVal+1) } ;
    const float* histRange = { range };
    calcHist(&src, 1, 0, Mat(), result, 1, &histSize, &histRange, true, false);
    
    if(normed) {
        //       normalize(result, result, 1, 0, NORM_L1, -1, Mat()); //! Normalized so Sum = 1
        normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
    }
    return result;
}


//! NOTE - Histogram contains frequency of count and NOT individual elements.

//! Function computes HOG descriptor
//! The window is slide through the image column and each block or cell x cell size is used to compute hog descriptor.
void HOGFeatureDescriptors::getHOG(Mat &image, Mat &bins, Mat &featureMD){
    
    for (int j = 0; j < image.rows; j += CELL) {
        for (int i = 0; i < image.cols; i += CELL) {                        //! Image space sliding window of size CELL with 50% overlap
            Rect rect = Rect(i, j, CELL*BLOCK, CELL*BLOCK);                 //! Block containing 4 cell of 8 x 8
            
            if ((rect.x + rect.width <= image.cols) && (rect.y + rect.height <= image.rows)) { //! Check to see if Sliding window is within image space
        
                Mat hogMD = this->blockGradient(rect.x, rect.y, bins);

//                normalize(hogMD, hogMD, 1, 0, CV_L2); //! Block Normalization using L2-Norm distance
                featureMD.push_back(hogMD); //! Each block is pushed to this Mat.
            }
        }
    }
}


//! Function returns HOG descriptor for a image
Mat HOGFeatureDescriptors::computeHOG(const Mat &img){
    
    Mat image = img.clone();
    
    if (image.type() != CV_8U) {
        cvtColor(image, image, CV_BGR2GRAY);
    }
    
    Mat bins;                           //! Oriented Bi-linear interpolated Gradient Orietnation
    image_gradient(image, bins);
    Mat featureMD;                      //! HOG Feature Descriptors
    getHOG(image, bins, featureMD);
    featureMD = featureMD.reshape(1,1);
    
    return featureMD;
}

double HOGFeatureDescriptors::computeHOGHistogramDistances(Mat &patch, vector<Mat> imageHOG){
    
    double sum = 0.0;
    double argMinDistance = 1000.0;
    for (int i = 0; i < imageHOG.size(); i++) {
        Mat img_hog = imageHOG[i];
        double d = compareHist(patch, img_hog, CV_COMP_BHATTACHARYYA);
        
        if (d < argMinDistance) {
            argMinDistance = static_cast<double>(d);
        }
    }
    sum = static_cast<double>(argMinDistance);
    return static_cast<double>(sum);
}

