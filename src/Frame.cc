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

    std::vector<std::vector<std::pair<cv::Point2f,cv::Point2f>>> vvpair_grid_corners = p_extractor->GetGrids();

    for(auto v : vvpair_grid_corners) {
      for(auto p : v) {
        std::cout << p.first.x << ":" << p.first.y << std::endl; 
        std::cout << p.second.x << ":" << p.second.y << std::endl; 
        std::cout << "-----------------------------" << std::endl; 
      } 
    }

    p_extractor->DistributeToGrids(m_v_kpts, m_m_descriptors,
                                   m_vvv_grid_kpts, m_vvm_grid_descs);
  }

  Frame::Frame()
    : m_id(0), m_m_image(cv::Mat::zeros(3,3,CV_8UC1))
  {
  }

  Frame::~Frame() {
  }

  void Frame::ShowFeaturePoints() {
    cv::Mat output;
    cv::drawKeypoints(m_m_image, m_v_kpts, output);

    cv::imshow("test", output);
    cv::waitKey(0); 
  
    return;
  }

};

