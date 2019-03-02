#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class KPExtractor;

  class Frame{
    public:
      Frame(const cv::Mat& m_image, const std::shared_ptr<KPExtractor>& p_extractor);
      Frame();
      ~Frame();

    private:
      unsigned int m_id;
      cv::Mat m_m_image;

  };
};

