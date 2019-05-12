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

      KPExtractor(const unsigned int image_width,
                  const unsigned int image_height,
                  const ExtractorConfig& _config);
      ~KPExtractor();

      void DistributeToGrids(const cv::Mat& vm_descriptors,
                             const std::vector<cv::KeyPoint>& v_keypoints,
                             std::vector<std::vector<std::vector<cv::KeyPoint>>>& vvv_grid_kpts,
                             std::vector<std::vector<cv::Mat>>& vvm_descs); 

      void ExtractFeaturePoints(cv::Mat m_input,
                                             std::vector<cv::KeyPoint>& v_kpts,
                                             cv::Mat& m_descriptors);

      std::vector< std::vector<std::pair<cv::Point2f, cv::Point2f>>> GetGrids();

    private:

      void SetGrids();

      const unsigned int m_image_width, m_image_height;
      const unsigned int m_num_vertical_grid, m_num_horizontal_grid;
      const ExtractorConfig m_config;

      cv::Ptr<cv::Feature2D> m_p_extractor;

      std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vv_grid_kpts;
      std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;

      std::vector<std::vector<std::pair<cv::Point2f,cv::Point2f>>> m_vvpair_grid_corners;
  
  };
};
