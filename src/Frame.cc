#include "Frame.h"
#include "KPExtractor.h"

namespace TS_SfM {
  Frame::Frame(const int id, const cv::Mat& m_image, 
               const std::shared_ptr<KPExtractor>& p_extractor)
    : m_id(id), m_m_image(m_image)
  {
    p_extractor->ExtractFeaturePoints(m_m_image, m_v_kpts, m_m_descriptors);

    // std::cout << "[LOG.Frame.FeaturePoints] "
    //           << m_m_descriptors.rows
    //           << std::endl;

    std::vector<std::vector<std::pair<cv::Point2f,cv::Point2f>>>
      vvpair_grid_corners = p_extractor->GetGrids();

#if 0
    for(auto v : vvpair_grid_corners) {
      for(auto p : v) {
        std::cout << p.first.x << ":" << p.first.y << std::endl; 
        std::cout << p.second.x << ":" << p.second.y << std::endl; 
        std::cout << "-----------------------------" << std::endl; 
      } 
    }
#endif

    m_vv_num_grid_kpts = p_extractor->DistributeToGrids(m_v_kpts, m_m_descriptors,
                                                        m_vvv_grid_kpts, m_vvm_grid_descs);

#if 0
    for(auto v : m_vv_num_grid_kpts) {
      for(auto num : v) {
        std::cout << num << std::endl; 
        std::cout << "-----------------------------" << std::endl; 
      } 
    }
#endif

    m_num_assigned_kps = 0;
    for(auto v : m_vv_num_grid_kpts) 
      for(auto n : v) 
        m_num_assigned_kps += n;

#if 0
    std::cout << "[LOG.Frame.AssingedFeaturePoints] "
              << m_num_assigned_kps
              << std::endl;
#endif

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

  void Frame::ShowFeaturePointsInGrids() {
    cv::Mat temp, output;
    output = m_m_image.clone();

    for(size_t i = 0; i < m_vvv_grid_kpts.size(); i++) {
      for(size_t j = 0; j < m_vvv_grid_kpts[i].size(); j++) {
        temp = output.clone();
        cv::drawKeypoints(temp, m_vvv_grid_kpts[i][j], output);
      }
    }

    cv::imshow("grid-feature-points", output);
    cv::waitKey(0); 
  
    return;
  }

};

