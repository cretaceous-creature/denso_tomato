#include "tomato_pedicel_detection/pedicel_detect.hpp"

namespace tomato_pedicel_detection {

PedicelDetector::PedicelDetector(cv::Mat& in_image) :
    in_image_(in_image) {
}

  void PedicelDetector::Detect(cv::Mat& out_image) {
  cv::Mat hsv_image;
  jsk_perception::calcOrientedGradient(in_image_, hsv_image);
  cv::cvtColor(hsv_image, out_image, CV_HSV2BGR);

}

}  // namespace



int main(int argc, char **argv) {

  if (argc < 2) {
    std::cerr << "usage: pedicel_detect <filename>" << std::endl;
    return 1;
  }

  cv::Mat in_image = cv::imread(argv[1], 1);
  if (!in_image.data) {
    std::cerr << "could not read " << argv[1] << std::endl;
    return 1;
  }

  tomato_pedicel_detection::PedicelDetector* detect =
      new tomato_pedicel_detection::PedicelDetector(in_image);

  cv::Mat out_image;
  detect->Detect(out_image);

  cv::namedWindow("Image");
  cv::imshow("Image", out_image);

  while (true) {
    char c = cv::waitKey(0);
  }

  delete detect;
  return 0;
}
