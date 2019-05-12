#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class KPExtractor;

  class Frame{
    public:
      Frame(const int id, const cv::Mat& m_image, const std::shared_ptr<KPExtractor>& p_extractor);
      Frame();
      ~Frame();

      void ShowFeaturePoints();

      const int m_id;
      const cv::Mat m_m_image;

    private:
      bool m_is_key;
      cv::Mat m_m_cTw; // (4 x 4, CV_F64C1)
      std::vector<cv::KeyPoint> m_v_kpts;
      cv::Mat m_m_descriptors;

      std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vvv_grid_kpts;
      std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;

  };
};

