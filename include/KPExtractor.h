#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

namespace TS_SfM {
  class KPExtractor {
    public:

      struct ExtractorConfig {
        std::string str_descriptor;
        float threshold;
        unsigned int octaves;
        unsigned int octavelayers;
        unsigned int grid_width;
        unsigned int grid_height;
      };

      KPExtractor();
      // KPExtractor(const unsigned int image_width,
      //             const unsigned int image_height,
      //             const unsigned int cell_width=100,
      //             const unsigned int cell_height=100);

      KPExtractor(const unsigned int image_width,
                  const unsigned int image_height,
                  const ExtractorConfig& _config);
      ~KPExtractor();

      // std::vector< std::vector<int> > DestributeToGrid(const std::vector<cv::KeyPoint>& v_keypoints); 

  void ExtractFeaturePoints(cv::Mat m_input,
                                         std::vector<cv::KeyPoint>& v_kpts,
                                         cv::Mat& m_descriptors);

    private:
      const unsigned int m_image_width, m_image_height;
      const unsigned int m_num_vertical_grid, m_num_horizontal_grid;
      const ExtractorConfig m_config;

      cv::Ptr<cv::Feature2D> m_p_extractor;
  
  };
};
