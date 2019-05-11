#include "KPExtractor.h"

namespace TS_SfM {
  /*
  KPExtractor::KPExtractor(const unsigned int image_width,
                           const unsigned int image_height,
                           const unsigned int cell_width,
                           const unsigned int cell_height) 
    : m_image_width(image_width), m_image_height(image_height),
      m_cell_width(cell_width), m_cell_height(cell_height),
      m_num_vertical_grid((unsigned int)image_height/cell_height), 
      m_num_horizontal_grid((unsigned int)image_width/cell_width) 
  {
    cv::AKAZE::DescriptorType descriptor_type=cv::AKAZE::DESCRIPTOR_MLDB;
    int descriptor_size=0;
    int descriptor_channels=3;
    float threshold=0.001f;
    int nOctaves=4;
    int nOctaveLayers=4;
    cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
    m_p_extractor
      = cv::AKAZE::create(descriptor_type,descriptor_size,descriptor_channels,threshold,
                          nOctaves, nOctaveLayers, diffusivity);
  }
  */

  KPExtractor::KPExtractor(const unsigned int image_width,
              const unsigned int image_height,
              const ExtractorConfig& _config)
    : m_image_width(image_width), m_image_height(image_height),
      m_num_vertical_grid((unsigned int)image_height/_config.grid_height), 
      m_num_horizontal_grid((unsigned int)image_width/_config.grid_width), m_config(_config)
  {
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

  // std::vector< std::vector<int> >
  //   KPExtractor::DestributeToGrids(const std::vector<cv::KeyPoint>& v_keypoints) 
  // {
  //   std::vector< std::vector<int> > v_kpID_grid
  //     = std::vector<std::vector<int> > (m_num_vertical_grid, std::vector<int>(m_num_horizontal_grid, -1));
  //
  //
  //
  //   return v_kpID_grid;
  // }
};
