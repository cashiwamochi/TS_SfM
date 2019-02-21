#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace TS_SfM {
  class KPExtractor;

  class Frame{
    public:
      Frame(const cv::Mat& m_image, KPExtractor* p_extractor);
      Frame();
      ~Frame();

    private:
      unsigned int m_id;
      cv::Mat m_m_image;

  };
};

