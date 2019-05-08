#pragma once

namespace TS_SfM {
  class LoopClosure {
    public:
      struct LoopConfig {
        int start_id;
        int end_id;
      };

      LoopClosure(const LoopConfig& _config);
      ~LoopClosure();

    private:
      LoopConfig m_config;

  };
};
