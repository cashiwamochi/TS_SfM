#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class KPExtractor;

  class Frame{
    public:
      Frame(const int id, const cv::Mat& m_image, const std::shared_ptr<KPExtractor>& p_extractor);
      Frame();
      ~Frame();

      void ShowFeaturePoints();
      void ShowFeaturePointsInGrids();

      const int m_id;

      cv::Mat GetDescriptors() const; 
      std::vector<cv::KeyPoint> GetKeyPoints() const;
      cv::Mat GetImage() const;
      cv::Mat GetPose() const;
      std::vector<std::vector<std::vector<cv::KeyPoint>>> GetGridKeyPoints() const;
      std::vector<std::vector<cv::Mat>> GetGridDescs() const;
      std::vector<std::vector<unsigned int>> GetGridKeyPointsNum() const;
      unsigned int GetAssignedKeyPointsNum() const;

      void SetPose(const cv::Mat& _cTw){m_m_cTw = _cTw.clone();};


    private:
      const cv::Mat m_m_image;
      bool m_is_key;
      cv::Mat m_m_cTw; // (4 x 4, CV_F32C1)

      // original data
      std::vector<cv::KeyPoint> m_v_kpts;
      cv::Mat m_m_descriptors;

      // data assigned to grids
      std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vvv_grid_kpts;
      std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;
      std::vector<std::vector<unsigned int>> m_vv_num_grid_kpts;
      unsigned int m_num_assigned_kps;

  };
};

