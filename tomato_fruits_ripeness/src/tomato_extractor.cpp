#include "tomato_fruits_ripeness/tomato_extractor.hpp"

namespace tomato_fruits_ripeness {

TomatoExtractor::TomatoExtractor(cv::Mat& in_image) :
    in_image_(in_image) {
  mask_image_.create(in_image_.size(), CV_32S);
  cx_ = 0;
  cy_ = 0;

  h_num_ = 32;
  s_num_ = 32;
  hsvhist_.resize(h_num_ * s_num_);
}

void TomatoExtractor::SetCenter(int x, int y) {
  cx_ = x;
  cy_ = y;
}

void TomatoExtractor::SetRadius(int x, int y) {
  double dx = static_cast<double>(cx_ - x);
  double dy = static_cast<double>(cy_ - y);
  radius_ = sqrt(dx * dx + dy * dy);
}

void TomatoExtractor::Draw(cv::Mat& img) {
  // in_image_.copyTo(img);
  // cv::circle(img, cv::Point(cx_, cy_), static_cast<int>(radius_),
  //              cv::Scalar(255, 255, 0), 3);

  img.create(in_image_.size(), CV_8UC3);

  int width = in_image_.cols;
  int height = in_image_.rows;

  for (size_t y = 0; y < height; y++) {
    for (size_t x = 0; x < width; x++) {
      cv::Vec3b& opx = img.at<cv::Vec3b>(y, x);
      cv::Vec3b& ipx = in_image_.at<cv::Vec3b>(y, x);
      float weight = (0.3 + mask_image_.at<float>(y, x)) / 1.3;
      if (weight < 1e-5) {
        opx[0] = opx[1] = opx[2] = 0;
      } else {
        opx[0] = static_cast<unsigned char>(
            static_cast<float>(ipx[0]) * weight);
        opx[1] = static_cast<unsigned char>(
            static_cast<float>(ipx[1]) * weight);
        opx[2] = static_cast<unsigned char>(
            static_cast<float>(ipx[2]) * weight);
      }
    }
  }
}

void TomatoExtractor::CreateMask() {
  int x0 = cx_ - static_cast<int>(radius_);
  int x1 = cx_ + static_cast<int>(radius_);
  int y0 = cy_ - static_cast<int>(radius_);
  int y1 = cy_ + static_cast<int>(radius_);
  int width = in_image_.cols;
  int height = in_image_.rows;

  int rad2 = static_cast<int>(radius_ * radius_);

  x0 = (x0 < 0) ? 0 : x0;
  x1 = (x1 > width) ? width : x1;
  y0 = (y0 < 0) ? 0 : y0;
  y1 = (y1 > height) ? height : y1;

  for (size_t y = 0; y < y0; y++) {
    for (size_t x = 0; x < width; x++) {
      mask_image_.at<float>(y, x) = 0;
    }
  }

  for (size_t y = y0; y < y1; y++) {
    int y2 = (y - cy_) * (y - cy_);
    for (size_t x = 0; x < x0; x++) {
      mask_image_.at<float>(y, x) = 0;
    }

    for (size_t x = x0; x < x1; x++) {
      int x2 = (x - cx_) * (x - cx_);
      if ((x2 + y2) > rad2) {
        mask_image_.at<float>(y, x) = 0;
      } else {
        double rad = sqrt(static_cast<double>(x2 + y2));
        double rr = rad / radius_;
        double weight = Sigmoid_(0.8 - rr, 5.0);

        mask_image_.at<float>(y, x) = static_cast<float>(weight);
      }
    }

    for (size_t x = x1; x < width; x++) {
      mask_image_.at<float>(y, x) = 0;
    }
  }

  for (size_t y = y1; y < height; y++) {
    for (size_t x = 0; x < width; x++) {
      mask_image_.at<float>(y, x) = 0;
    }
  }
}

void TomatoExtractor::Extract() {
  cv::Mat hsv_image;
  cv::cvtColor(in_image_, hsv_image, CV_BGR2HSV);

  int x0 = cx_ - static_cast<int>(radius_);
  int x1 = cx_ + static_cast<int>(radius_);
  int y0 = cy_ - static_cast<int>(radius_);
  int y1 = cy_ + static_cast<int>(radius_);
  int width = in_image_.cols;
  int height = in_image_.rows;

  int rad2 = static_cast<int>(radius_ * radius_);
  x0 = (x0 < 0) ? 0 : x0;
  x1 = (x1 > width) ? width : x1;
  y0 = (y0 < 0) ? 0 : y0;
  y1 = (y1 > height) ? height : y1;

  float mask_total = 0.0;

  hsvhist_.clear();
  hsvhist_.resize(h_num_ * s_num_);
  for (size_t i = 0; i < hsvhist_.size(); i++) {
    hsvhist_[i] = 0.0;
  }
  float hd = 180.0 / h_num_;
  float sd = 255.0 / s_num_;

  for (size_t y = y0; y < y1; y++) {
    int y2 = (y - cy_) * (y - cy_);
    for (size_t x = x0; x < x1; x++) {
      int x2 = (x - cx_) * (x - cx_);
      if ((x2 + y2) > rad2) continue;

      float weight = mask_image_.at<float>(y, x);
      mask_total += weight;
      cv::Vec3b& hsv = hsv_image.at<cv::Vec3b>(y, x);

      size_t hi = static_cast<size_t>(static_cast<float>(hsv[0]) / hd);
      size_t si = static_cast<size_t>(static_cast<float>(hsv[1]) / sd);
      size_t idx = hi * s_num_ + si;
      hsvhist_[idx] += weight;
    }
  }

  // Normalize
  for (size_t i = 0; i < hsvhist_.size(); i++) {
    hsvhist_[i] /= mask_total;
  }

  // output
  std::cout << "i";  // category
  for (size_t i = 0; i < hsvhist_.size(); i++) {
    std::cout << " " << hsvhist_[i];
  }
  std::cout << std::endl;
}

}  // namespace

static void OnMouseCallback(int event, int x, int y, int flag, void* param) {
  cv::Mat img;
  tomato_fruits_ripeness::TomatoExtractor* extract =
      reinterpret_cast<tomato_fruits_ripeness::TomatoExtractor*>(param);
  if (event == cv::EVENT_LBUTTONDOWN) {
    extract->SetCenter(x, y);
  } else if (event == cv::EVENT_LBUTTONUP) {
    extract->Extract();
  } else if (flag == cv::EVENT_FLAG_LBUTTON) {
    extract->SetRadius(x, y);
    extract->CreateMask();
    extract->Draw(img);
    cv::imshow("Image", img);
  }
  // std::cout << "event: " << event << ", flag: " << flag << std::endl;
}


int main(int argc, char **argv) {

  if (argc < 2) {
    std::cerr << "usage: tomato_extractor <filename>" << std::endl;
    return 1;
  }

  cv::Mat in_image = cv::imread(argv[1], 1);
  if (!in_image.data) {
    std::cerr << "could not read " << argv[1] << std::endl;
    return 1;
  }

  tomato_fruits_ripeness::TomatoExtractor* extract =
      new tomato_fruits_ripeness::TomatoExtractor(in_image);

  cv::namedWindow("Image");
  cv::setMouseCallback("Image", OnMouseCallback,
                       reinterpret_cast<void*>(extract));

  cv::imshow("Image", in_image);

  while (true) {
    char c = cv::waitKey(0);
  }

  delete extract;
  return 0;
}
