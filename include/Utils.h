#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace TS_SfM{
  class Frame;
  cv::Mat ChooseDescriptor(const Frame& f0, const Frame& f1,
                           const cv::Point3f& p, const cv::DMatch& match);
}
