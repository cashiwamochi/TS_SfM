#pragma once

#include "ConfigLoader.h"

namespace TS_SfM {

  class Frame;
  class KPExtractor;

  class System{
    public:
      System(const std::string& str_config_file);
      ~System();

    private:
      void run();
      void showConfig();
      Config m_config;
      Camera m_camera;
      std::vector<std::string> m_vstr_image_names; 
      std::vector<cv::Mat> m_vm_images; 
      std::vector<Frame> m_v_frames; 
  
  
  };

}// TS_SfM
