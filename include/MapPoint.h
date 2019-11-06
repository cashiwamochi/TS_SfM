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
      ~MapPoint(){};


    private:
      Pos m_pos;
      cv::Mat m_m_descriptor;
      std::vector<int> m_observe_cam_idx;
      std::vector<MatchInfo> m_v_match_info;

  };

} // namespace
