#pragma once

#include <iostream>
#include <vector>

namespace TS_SfM {

  class MapPoint;
  class Frame;

  class Map {
    public:
      Map(){};
      ~Map(){};

    private:


      std::list<MapPoint> m_l_map_point;
      std::list<Frame> m_l_keyframe;
  };

} // namespace
