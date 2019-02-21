#include "KPExtractor.h"

namespace TS_SfM {
  KPExtractor::KPExtractor(const unsigned int image_width,
                           const unsigned int image_height,
                           const unsigned int cell_width,
                           const unsigned int cell_height) 
    : m_image_width(image_width), m_image_height(image_height),
      m_cell_width(cell_width), m_cell_height(cell_height),
      m_num_vertical_grid((unsigned int)image_height/cell_height), 
      m_num_horizontal_grid((unsigned int)image_width/cell_width) 
  {
  }

  KPExtractor::KPExtractor()
  : m_image_width(0), m_image_height(0),
    m_cell_width(0), m_cell_height(0),
    m_num_vertical_grid(0), m_num_horizontal_grid(0) 
  {
  }

  KPExtractor::~KPExtractor()
  {
  }

  std::vector<cv::KeyPoint> KPExtractor::detect(cv::Mat m_image) {
    std::vector<cv::KeyPoint> v_keypoints;



    return v_keypoints;
  }

  std::vector< std::vector<int> >
    KPExtractor::destributeToGrid(const std::vector<cv::KeyPoint>& v_keypoints) 
  {
    std::vector< std::vector<int> > v_kpID_grid
      = std::vector<std::vector<int> > (m_num_vertical_grid, std::vector<int>(m_num_horizontal_grid, -1));



    return v_kpID_grid;
  }
};
