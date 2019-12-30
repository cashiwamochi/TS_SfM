#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

namespace TS_SfM {
    struct MatchInfo {
      int frame_id;
      int kpt_id;
    };

  class MapPoint {
    public:
      MapPoint(){};
      MapPoint(cv::Point3f pt) {
        m_pos.x = pt.x;
        m_pos.y = pt.y;
        m_pos.z = pt.z;
        m_is_activated = false;
      };
      MapPoint(const float& x, const float& y, const float& z) {
        m_pos.x = x;
        m_pos.y = y;
        m_pos.z = z;
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

      void SetDescriptor(const cv::Mat& _desc) {
        m_m_descriptor = _desc.clone();
      }

      void SetPosition(const float x, const float y, const float z) {
        m_pos = cv::Point3f(x,y,z);
      }

      cv::Point3f GetPosition() const {
        return m_pos;
      }

      bool IsActivated() {
        return m_is_activated;
      }

      bool Activate() {
        if(m_m_descriptor.empty() || m_v_match_info.size() == 0)
        {
          m_is_activated = false;
          std::cout << "[Warning] MapPoint hasn't beed activated.\n";
        }
        else {
          m_is_activated = true;
        }
        return m_is_activated;
      }

      int GetObsNum() const {
        return (int)m_v_match_info.size();
      }

      MatchInfo GetMatchInfo(const int& idx) const {
        return m_v_match_info[idx];
      }


    private:
      cv::Point3f m_pos;
      cv::Mat m_m_descriptor;
      std::vector<MatchInfo> m_v_match_info;
      bool m_is_activated;

  };

} // namespace
