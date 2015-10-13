#include "tomato_pedicel_detection/pedicel_detect.hpp"

namespace tomato_pedicel_detection {
  
PedicelDetector::PedicelDetector(cv::Mat& in_image) :
  in_image_(in_image) {
}

void PedicelDetector::CalcGradient(cv::Mat& out_image) {
  jsk_perception::calcOrientedGradient(in_image_, hsv_image_);
  cv::cvtColor(hsv_image_, out_image, CV_HSV2BGR);
}
  
void PedicelDetector::Detect(cv::Mat& out_image) {
  CalcGradient(out_image);

  cv::Mat para_edge_image(hsv_image_.size(), CV_8UC1);
  for (size_t y = 0; y < hsv_image_.rows; y++)
    for (size_t x = 0; x < hsv_image_.cols; x++)
      para_edge_image.at<uint8_t>(y, x) = 0;

  cv::Mat cos_sin_table(hsv_image_.size(), CV_32FC3);
  for (size_t y = 0; y < hsv_image_.rows; y++) {
    for (size_t x = 0; x < hsv_image_.cols; x++) {
      cv::Vec3b& p = hsv_image_.at<cv::Vec3b>(y, x);
      cv::Vec3f& cs = cos_sin_table.at<cv::Vec3f>(y, x);
      double th = static_cast<double>(p[0]) / 180.0 * M_PI;
      double cosp = cos(th);
      double sinp = sin(th);
      cs[0] = static_cast<float>(cosp);
      cs[1] = static_cast<float>(sinp);
    }
  }
  

  for (size_t r = 4; r < 32; r++) {
    for (size_t th = 0; th < 180; th += 10) {
      double th_rad = static_cast<double>(th) / 180.0 * M_PI;
      double cost = cos(th_rad);
      double sint = sin(th_rad);
      int32_t ox = static_cast<int32_t>(cost * r);
      int32_t oy = static_cast<int32_t>(sint * r);
      
      for (size_t y = 32; y < hsv_image_.rows - 32; y++) {
        for (size_t x = 32; x < hsv_image_.cols - 32; x++) {
          cv::Vec3b& p0 = hsv_image_.at<cv::Vec3b>(y - oy, x - ox);
          cv::Vec3b& p1 = hsv_image_.at<cv::Vec3b>(y + oy, x + ox);
          cv::Vec3f& cs0 = cos_sin_table.at<cv::Vec3f>(y - oy, x - ox);
          cv::Vec3f& cs1 = cos_sin_table.at<cv::Vec3f>(y + oy, x + ox);

          double cos_p0 = cs0[0];
          double sin_p0 = cs0[1];
          double cos_p1 = cs1[0];
          double sin_p1 = cs1[1];

          double inp0 = fabs(cos_p0 * cost + sin_p0 * sint);
          double inp1 = fabs(cos_p1 * cost + sin_p1 * sint);

          para_edge_image.at<uint8_t>(y, x) +=
              static_cast<uint8_t>(
                  inp0 * inp1
                  * static_cast<double>(p0[2])
                  * static_cast<double>(p1[2])
                  / 255.0 / 4.0);
        }
      }
    }
  }

  // cv::cvtColor(hsv_image_, out_image, CV_HSV2BGR);
  // para_edge_image.copyTo(out_image);

  out_image.create(hsv_image_.size(), CV_8UC3);
  for (size_t y = 0; y < hsv_image_.rows; y++) {
    for (size_t x = 0; x < hsv_image_.cols; x++) {
      cv::Vec3b& op = out_image.at<cv::Vec3b>(y, x);
      cv::Vec3b& hsv = hsv_image_.at<cv::Vec3b>(y, x);
      uint8_t pe = para_edge_image.at<uint8_t>(y, x);
      op[0] = 0;
      op[1] = hsv[2];
      op[2] = pe;
    }
  }
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
  // detect->CalcGradient(out_image);

  cv::namedWindow("Image");
  cv::imshow("Image", out_image);

  while (true) {
    char c = cv::waitKey(0);
  }

  delete detect;
  return 0;
}
