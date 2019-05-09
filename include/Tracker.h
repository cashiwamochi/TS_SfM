#pragma once

namespace TS_SfM {
  class Tracker {
    public:
      struct TrackerConfig {
        int skip;
      };

      Tracker(){};
      Tracker(TrackerConfig _config);
      ~Tracker(){};

    private:
      TrackerConfig m_config;

  };
};
