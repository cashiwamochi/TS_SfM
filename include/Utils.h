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

  // template<typename _Tp, int _rows, int _cols>
  // void cv2eigen(const cv::Mat& src,
  //                Eigen::Matrix<_Tp, _rows, _cols>& dst );
  //
  // template<typename _Tp, int _rows, int _cols>
  // void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols>& src,
  //               cv::Mat& dst );
}
