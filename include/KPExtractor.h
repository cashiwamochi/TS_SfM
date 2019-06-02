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
        unsigned int num_in_grid;
      };

      KPExtractor();

      KPExtractor(const unsigned int image_width,
                  const unsigned int image_height,
                  const ExtractorConfig& _config);
      ~KPExtractor();

      std::vector<std::vector<unsigned int>> 
        DistributeToGrids(
          const std::vector<cv::KeyPoint>& v_keypoints,
          const cv::Mat& m_descriptors,
          std::vector<std::vector<std::vector<cv::KeyPoint>>>& vvv_grid_kpts,
          std::vector<std::vector<cv::Mat>>& vvm_grid_descs);

      void ExtractFeaturePoints(cv::Mat m_input,
                                std::vector<cv::KeyPoint>& v_kpts,
                                cv::Mat& m_descriptors);

      const ExtractorConfig GetConfig(); 
      const std::pair<unsigned int, unsigned int> GetGridSize(); 

      std::vector< std::vector<std::pair<cv::Point2f, cv::Point2f>>> GetGrids();

    private:

      struct KPData {
        cv::KeyPoint kp;
        cv::Mat descriptor;

        KPData(cv::KeyPoint _kp, cv::Mat _desc) {
          kp = _kp;
          descriptor = _desc;
        }

        static bool cmp(const KPData &a, const KPData &b)
        {
          return a.kp.response > b.kp.response;
        }

      };


      std::pair<int, int> GetWhichGrid(const cv::Point2f& pt);
      void SetGrids();

      const unsigned int m_image_width, m_image_height;
      const unsigned int m_num_vertical_grid, m_num_horizontal_grid;
      const ExtractorConfig m_config;

      cv::Ptr<cv::Feature2D> m_p_extractor;


      std::vector<std::vector<std::pair<cv::Point2f,cv::Point2f>>> m_vvpair_grid_corners;
  
  };
};
