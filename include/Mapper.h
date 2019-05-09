#pragma once

namespace TS_SfM {

  class Mapper {
    public:
      struct MapperConfig{
        int skip;
      };

      Mapper(){};
      Mapper(MapperConfig _config);
      ~Mapper(){};

    private:
      MapperConfig m_config;
  };

} // namespace
