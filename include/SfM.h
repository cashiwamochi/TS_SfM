#pragma once

namespace TS_SfM {
  class SfM {
    public:
      struct SfMConfig {
        int track_skip;
        int reconst_skip;
      };

      SfM();
      ~SfM();

    private:

  };
};
