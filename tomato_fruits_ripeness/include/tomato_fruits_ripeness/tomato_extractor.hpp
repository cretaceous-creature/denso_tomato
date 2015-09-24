#ifndef TOMATO_FRUITS_RIPENESS_TOMATO_EXTRACTOR_HPP_
#define TOMATO_FRUITS_RIPENESS_TOMATO_EXTRACTOR_HPP_

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <iostream>
#include <vector>
#include <string>

namespace tomato_fruits_ripeness {

class TomatoExtractor {
 public:
  explicit TomatoExtractor(cv::Mat& in_image);
  ~TomatoExtractor() {}

  void SetCenter(int x, int y);
  void SetRadius(int x, int y);
  void CreateMask();
  void Draw(cv::Mat& img);
  void Extract();

 private:
  cv::Mat in_image_;
  cv::Mat mask_image_;
  int cx_, cy_;
  double radius_;
  std::vector<float> hsvhist_;
  size_t h_num_;
  size_t s_num_;

  double inline Sigmoid_(double x, double a) {
    return (0.5 * (tanh(0.5 * a * x) + 1.0));
  }
};

}  // namespace

#endif  // TOMATO_FRUITS_RIPENESS_TOMATO_EXTRACTOR_HPP_
