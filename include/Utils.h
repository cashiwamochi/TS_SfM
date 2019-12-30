#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace TS_SfM{
  class Frame;
  cv::Mat ChooseDescriptor(const Frame& f0, const Frame& f1,
                           const cv::Point3f& p, const cv::DMatch& match);

  bool CheckIndex(const int& src_frame_idx, const int& dst_frame_idx,
      const std::vector<bool>& vb_initialized, const int length);

  Eigen::Vector2d ProjectToImage(const cv::Mat& K, const cv::Mat& cTw, const cv::Point3f& pt);
}
