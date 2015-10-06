#ifndef TOMATO_PEDICEL_DETECTION_PEDICEL_DETECT_HPP_
#define TOMATO_PEDICEL_DETECTION_PEDICEL_DETECT_HPP_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <vector>
#include <string>

#include "jsk_perception/oriented_gradient.hpp"

namespace tomato_pedicel_detection {

class PedicelDetector {
 public:
  explicit PedicelDetector(cv::Mat& in_image);
  ~PedicelDetector() {}

  void Detect(cv::Mat& out_image);

 private:
  cv::Mat in_image_;
};

}  // namespace

#endif  // TOMATO_PEDICEL_DETECTION_PEDICEL_DETECT_HPP_
