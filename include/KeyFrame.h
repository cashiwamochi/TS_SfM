#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class Frame;

  class KeyFrame{
    public:
      KeyFrame(const Frame& f);
      ~KeyFrame();

      const int m_id;

      cv::Mat GetDescriptors() const; 
      std::vector<cv::KeyPoint> GetKeyPoints() const;
      cv::Mat GetImage() const;

    private:
      const cv::Mat m_m_image;
      cv::Mat m_m_cTw; // (4 x 4, CV_F64C1)

      const std::vector<cv::KeyPoint> m_v_kpts;
      const cv::Mat m_m_descriptors;

      const std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vvv_grid_kpts;
      const std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;
      const std::vector<std::vector<unsigned int>> m_vv_num_grid_kpts;
      const unsigned int m_num_assigned_kps;

  };
};

