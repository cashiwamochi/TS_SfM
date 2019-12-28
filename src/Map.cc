#include "Map.h"

namespace TS_SfM {
  void Map::Initialize(std::vector<KeyFrame> v_keyframes, std::vector<MapPoint> v_mappoints) {
    m_v_keyframes = v_keyframes;
    m_v_mappoints = v_mappoints;
    return;
  }

} //TS_SfM
