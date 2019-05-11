#include "Frame.h"
#include "KPExtractor.h"

namespace TS_SfM {
  Frame::Frame(const int id, const cv::Mat& m_image, 
               const std::shared_ptr<KPExtractor>& p_extractor)
    : m_id(id), m_m_image(m_image)
  {
    p_extractor->ExtractFeaturePoints(m_m_image, m_v_kpts, m_m_descriptors);

    std::cout << "[LOG.Frame.FeaturePoints] "
              << m_m_descriptors.rows
              << std::endl;

    // p_extractor->DistrubuteKP2Grids(m_m_image, m_v_kpts, m_m_descriptors);
  }

  Frame::Frame()
    : m_id(0), m_m_image(cv::Mat::zeros(3,3,CV_8UC1))
  {
  }

  Frame::~Frame() {
  }
};

