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

      void SetInitialMap();

      std::mutex m_update_mtx;

    private:
      std::list<MapPoint> m_l_map_point;
      std::list<KeyFrame> m_l_keyframe;
  };
} // namespace
