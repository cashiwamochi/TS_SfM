#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class Frame;

  class KeyFrame{
    public:
      KeyFrame(const Frame& f);
      KeyFrame(){};
      ~KeyFrame(){};

      int m_id;

      cv::Mat GetDescriptors() const; 
      std::vector<cv::KeyPoint> GetKeyPoints() const;
      cv::Mat GetImage() const;

    private:
      cv::Mat m_m_image;
      cv::Mat m_m_cTw; // (4 x 4, CV_F32C1)

      std::vector<cv::KeyPoint> m_v_kpts;
      cv::Mat m_m_descriptors;

      std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vvv_grid_kpts;
      std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;
      std::vector<std::vector<unsigned int>> m_vv_num_grid_kpts;
      unsigned int m_num_assigned_kps;

  };
};

