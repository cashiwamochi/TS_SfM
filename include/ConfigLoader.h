#pragma once

#include "opencv2/core.hpp"
#include <iostream>
#include <string>

namespace TS_SfM {

  struct Config {
    std::string str_path_to_data; 
    std::pair<int, int> pair_loop_frames;
  };

  struct Camera {
    float f_cx;
    float f_cy;
    float f_fx;
    float f_fy;
  };

  namespace ConfigLoader {

    Config LoadConfig(const std::string str_config_file);
    Camera LoadCamera(const std::string str_camera_file);
  
  }

}
