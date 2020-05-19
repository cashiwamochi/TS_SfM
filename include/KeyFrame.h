#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace TS_SfM {
  class Frame;

  struct MatchInfo;

  class KeyFrame{
    public:
      KeyFrame(const Frame& f);
      KeyFrame(){m_b_activated=false;};
      ~KeyFrame(){};

      int m_id;

      cv::Mat GetDescriptors() const; 
      std::vector<cv::KeyPoint> GetKeyPoints() const;
      cv::Mat GetImage() const;
      cv::Mat GetPose() {return m_m_cTw;};
      cv::Mat GetPoseTrans() {return m_m_cTw.rowRange(0,3).col(3);};
      cv::Mat GetPoseRot() {return m_m_cTw.rowRange(0,3).colRange(0,3);};
      cv::Point2f GetObs(const int& kp_id) { return m_v_kpts[kp_id].pt; }

      // KeyFrame is activated if only it has pose
      bool IsActivated() const {return m_b_activated;};

    private:
      cv::Mat m_m_image;
      cv::Mat m_m_cTw; // (3 x 4, CV_F32C1)

      std::vector<cv::KeyPoint> m_v_kpts;
      cv::Mat m_m_descriptors;

      bool m_b_activated;

      std::vector<std::vector<std::vector<cv::KeyPoint>>> m_vvv_grid_kpts;
      std::vector<std::vector<cv::Mat>> m_vvm_grid_descs;
      std::vector<std::vector<std::vector<int>>> m_vvv_grid_kp_idx;
      std::vector<std::vector<unsigned int>> m_vv_num_grid_kpts;
      unsigned int m_num_assigned_kps;

  };
};

