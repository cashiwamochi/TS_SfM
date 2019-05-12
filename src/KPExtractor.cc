#include "KPExtractor.h"

namespace TS_SfM {
  KPExtractor::KPExtractor(const unsigned int image_width,
              const unsigned int image_height,
              const ExtractorConfig& _config)
    : m_image_width(image_width), m_image_height(image_height),
      m_num_vertical_grid((unsigned int)image_height/_config.grid_height), 
      m_num_horizontal_grid((unsigned int)image_width/_config.grid_width), m_config(_config)
  {
    // AKAZE Config
    cv::AKAZE::DescriptorType descriptor_type=cv::AKAZE::DESCRIPTOR_MLDB;
    int descriptor_size=0;
    int descriptor_channels=3;
    float threshold=m_config.threshold;
    int nOctaves=m_config.octaves;
    int nOctaveLayers=m_config.octavelayers;
    cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
    m_p_extractor
      = cv::AKAZE::create(descriptor_type,descriptor_size,descriptor_channels,threshold,
                          nOctaves, nOctaveLayers, diffusivity);

    SetGrids();
  }

  KPExtractor::KPExtractor()
  : m_image_width(0), m_image_height(0),
    m_num_vertical_grid(0), m_num_horizontal_grid(0), 
    m_config(ExtractorConfig())
  {
  }

  KPExtractor::~KPExtractor()
  {
  }

  void KPExtractor::ExtractFeaturePoints(cv::Mat m_input,
                                         std::vector<cv::KeyPoint>& v_kpts,
                                         cv::Mat& m_descriptors) {
    v_kpts.clear();
    m_descriptors.release();
    m_p_extractor->detectAndCompute(m_input, cv::noArray(), v_kpts, m_descriptors);
  
    return; 
  }

  void DistributeToGrids(const cv::Mat& vm_descriptors,
                         const std::vector<cv::KeyPoint>& v_keypoints,
                         std::vector<std::vector<std::vector<cv::KeyPoint>>>& vvv_grid_kpts,
                         std::vector<std::vector<cv::Mat>>& vvm_descs) 
  {
  
    
    return;
  }

  void KPExtractor::SetGrids() {
    std::vector< std::vector<std::pair<cv::Point2f,cv::Point2f>>> vv_grid_points; 
    vv_grid_points.resize(m_num_vertical_grid);

    int offset_horizontal = (m_image_width % m_config.grid_width)/2;
    int offset_vertical = (m_image_height % m_config.grid_height)/2;

    for(unsigned int row = 0; row < m_num_vertical_grid; row++) {
      std::vector<std::pair<cv::Point2f,cv::Point2f>> v_grid_points_horizontal;
      v_grid_points_horizontal.reserve(m_num_horizontal_grid);

      for(unsigned int col = 0; col < m_num_horizontal_grid; col++) {
        cv::Point2f top_left(offset_horizontal+col*m_config.grid_width,
                             offset_vertical+row*m_config.grid_height);
        cv::Point2f bottom_right(offset_horizontal+(col+1)*m_config.grid_width,
                                 offset_vertical+(row+1)*m_config.grid_height);
        
        if(col == 0) {
          top_left.x = 0.0; 
        }
        else if(col == m_num_horizontal_grid-1) {
          bottom_right.x = m_image_width; 
        }

        if(row == 0) {
          top_left.y = 0.0; 
        }
        else if(row == m_num_vertical_grid-1) {
          bottom_right.y = m_image_height; 
        }
        v_grid_points_horizontal.push_back(std::make_pair(top_left, bottom_right));  
      }
      vv_grid_points.push_back(v_grid_points_horizontal);
    }

    m_vvpair_grid_corners = vv_grid_points;

    return;
  }

  std::vector< std::vector<std::pair<cv::Point2f, cv::Point2f>>> KPExtractor::GetGrids() {
    return m_vvpair_grid_corners;
  }
  
};
