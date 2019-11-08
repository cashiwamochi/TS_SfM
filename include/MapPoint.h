#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

namespace TS_SfM {

  class MapPoint {
    struct Pos {
      float x;
      float y;
      float z;
    };

    struct MatchInfo {
      int frame_id;
      int kpt_id;
    };

    public:
      MapPoint(){};
      MapPoint(cv::Point3f pt) {
        m_pos.x = pt.x;
        m_pos.y = pt.y;
        m_pos.z = pt.z;
        m_is_activated = false;
      };
      ~MapPoint(){};

      void SetMatchInfo(std::vector<MatchInfo> _v_match_info) {
        for(auto match_info : _v_match_info)
          m_v_match_info.push_back(match_info);
      }

      void SetMatchInfo(MatchInfo _match_info) {
        m_v_match_info.push_back(_match_info);
      }

      void SetDesctiptor(const cv::Mat& _desc) {
        m_m_descriptor = _desc.clone();
      }

      bool Activate() {
        if(m_m_descriptor.empty() || m_v_match_info.size() == 0)
        {
          m_is_activated = false;
        }
        else {
          m_is_activated = true;
        }
        return m_is_activated;
      }


    private:
      Pos m_pos;
      cv::Mat m_m_descriptor;
      std::vector<MatchInfo> m_v_match_info;
      bool m_is_activated;

  };

} // namespace
