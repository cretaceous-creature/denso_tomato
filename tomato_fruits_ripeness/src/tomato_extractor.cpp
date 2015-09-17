#include "tomato_fruits_ripeness/tomato_extractor.hpp"

namespace tomato_fruits_ripeness {

TomatoExtractor::TomatoExtractor(cv::Mat& in_image) :
    in_image_(in_image) {
  cx_ = 0;
  cy_ = 0;
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
  in_image_.copyTo(img);
  cv::circle(img, cv::Point(cx_, cy_), static_cast<int>(radius_),
             cv::Scalar(255, 255, 0), 3);
}

void TomatoExtractor::Extract() {

}

}  // namespace

static void OnMouseCallback(int event, int x, int y, int flag, void* param) {
  cv::Mat img;
  tomato_fruits_ripeness::TomatoExtractor* extract =
      reinterpret_cast<tomato_fruits_ripeness::TomatoExtractor*>(param);
  if (event == cv::EVENT_LBUTTONDOWN) {
    extract->SetCenter(x, y);
  } else if (flag == cv::EVENT_FLAG_LBUTTON) {
    extract->SetRadius(x, y);
    extract->Draw(img);
    cv::imshow("Image", img);
  } else if (event == cv::EVENT_LBUTTONUP) {
    extract->Extract();
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
