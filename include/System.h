#pragma once

#include "ConfigLoader.h"

#include <opencv2/opencv.hpp>


namespace TS_SfM {

  class System{
    public:
      System(const std::string& str_config_file);
      ~System();

    private:
      void showConfig();
      Config m_config;
      Camera m_camera;
      std::vector<std::string> m_vstr_image_names; 
      std::vector<cv::Mat> m_vm_images; 
  
  
  };

}// TS_SfM
