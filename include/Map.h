#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <mutex>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace TS_SfM {

  class Map {
    public:
      Map(){};
      ~Map(){};

      void Initialize(std::vector<KeyFrame> v_keyframes, std::vector<MapPoint> v_mappoints);

      std::mutex m_update_mtx;

      KeyFrame& GetKeyFrameObservingMapPoint(const int& map_id);

    private:
      std::vector<MapPoint> m_v_mappoints;
      std::vector<KeyFrame> m_v_keyframes;




  };
} // namespace
