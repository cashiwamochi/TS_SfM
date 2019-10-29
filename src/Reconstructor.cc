#include "Reconstructor.h"

#include "ConfigLoader.h"

namespace TS_SfM {
  Reconstructor::Reconstructor(const std::string& str_config_file)
  {
    m_p_tracker.reset(new Tracker(ConfigLoader::LoadTrackerConfig(str_config_file)));
    m_p_mapper.reset(new Mapper(ConfigLoader::LoadMapperConfig(str_config_file)));
  }

  void Reconstructor::Run() {


    return;
  }

  void Reconstructor::SetMap(std::shared_ptr<Map> p_map) {
    m_p_map = p_map;
    return;
  }

}
