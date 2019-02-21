#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace TS_SfM {
  class KPExtractor {
    public:
      KPExtractor();
      KPExtractor(const unsigned int image_width,
                  const unsigned int image_height,
                  const unsigned int cell_width=100,
                  const unsigned int cell_height=100);
      ~KPExtractor();

      std::vector<cv::KeyPoint> detect(cv::Mat m_image);
      std::vector< std::vector<int> > destributeToGrid(const std::vector<cv::KeyPoint>& v_keypoints); 

    private:
      const unsigned int m_image_width, m_image_height;
      const unsigned int m_cell_width, m_cell_height;
      const unsigned int m_num_vertical_grid, m_num_horizontal_grid;
  
  };
};
