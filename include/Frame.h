#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class KPExtractor;

  class Frame{
    struct Match {
      int src_idx;
      int dst_idx;
    };
    public:
#if 0
      Frame(const int id, const cv::Mat& m_image, const std::shared_ptr<KPExtractor>& p_extractor);
#endif
      Frame(const int id, const std::string path);
      ~Frame();

      void ShowFeaturePoints();
      void ShowFeaturePointsInGrids();

      const int m_id;
      const std::string m_str_path;

      cv::Mat GetDescriptors() const; 
      std::vector<cv::KeyPoint> GetKeyPoints() const;
      cv::Mat GetImage() const;
      cv::Mat GetPose() const;
      std::vector<std::vector<std::vector<cv::KeyPoint>>> GetGridKeyPoints() const;
      std::vector<std::vector<cv::Mat>> GetGridDescs() const;
      std::vector<std::vector<unsigned int>> GetGridKeyPointsNum() const;
      std::vector<std::vector<std::vector<int>>> GetGridKpIdx() const;
      unsigned int GetAssignedKeyPointsNum() const;

      std::unique_ptr<KPExtractor> Initialize(std::unique_ptr<KPExtractor> p_extractor, bool& isOK);

      void SetPose (const cv::Mat& _cTw) {
        m_m_cTw = _cTw.clone();
        m_m_wTc = cv::Mat::zeros(3,4,_cTw.type());
        m_m_wTc.rowRange(0,3).colRange(0,3) = _cTw.rowRange(0,3).colRange(0,3).t();
        m_m_wTc.rowRange(0,3).col(3) = -1.0
                                      *_cTw.rowRange(0,3).colRange(0,3).t() * _cTw.rowRange(0,3).col(3);
      };
      inline void SetMatchesToOld(const std::vector<cv::DMatch>& _v_matches_01) {
        m_v_matches_to_old.clear();
        m_v_matches_to_old.reserve(_v_matches_01.size());
        for(cv::DMatch match : _v_matches_01) {
          m_v_matches_to_old.push_back(Match{match.trainIdx, match.queryIdx});
        }
      };
      inline void SetMatchesToNew(const std::vector<cv::DMatch>& _v_matches_01) {
        m_v_matches_to_new.clear();
        for(cv::DMatch match : _v_matches_01) {
          m_v_matches_to_new.push_back(Match{match.queryIdx, match.trainIdx});
        }
      };


    private:
      cv::Mat m_m_image;
      bool m_is_key;
      cv::Mat m_m_cTw; // (3 x 4, CV_F32C1)
      cv::Mat m_m_wTc; // (3 x 4, CV_F32C1)

      // original data
      std::vector<cv::KeyPoint> m_v_kpts;
      cv::Mat m_m_descriptors;

      // data assigned to grids
      std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vvv_grid_kpts;
      std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;
      std::vector<std::vector<std::vector<int>>> m_vvv_grid_kp_idx;
      std::vector<std::vector<unsigned int>> m_vv_num_grid_kpts;
      unsigned int m_num_assigned_kps;

      std::vector<bool> m_vb_triangulated; 
      std::vector<Match> m_v_matches_to_old;
      std::vector<Match> m_v_matches_to_new;

  };
};

