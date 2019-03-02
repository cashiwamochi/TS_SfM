#pragma once

#include "ConfigLoader.h"
#include <memory>

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
      unsigned int m_image_width, m_image_height;
      std::vector<std::string> m_vstr_image_names; 
      std::vector<cv::Mat> m_vm_images; 
      std::vector<Frame> m_v_frames; 
  

      std::shared_ptr<KPExtractor> m_p_extractor;
  
  };

}// TS_SfM
