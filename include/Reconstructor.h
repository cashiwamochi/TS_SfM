#pragma once

#include <memory>
#include <vector>
#include <string>

#include "Tracker.h"
#include "Mapper.h"
#include "Map.h"

namespace TS_SfM {

  class Reconstructor {
    public:
      Reconstructor(const std::string& str_config_file);
      ~Reconstructor(){};

      void SetMap(std::shared_ptr<Map> p_map);

      // This is main.
      void Run();

    private:
      std::unique_ptr<Tracker> m_p_tracker;
      std::unique_ptr<Mapper> m_p_mapper;
      std::shared_ptr<Map> m_p_map;

  };
};
