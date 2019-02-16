#pragma once

#include <opencv2/opencv.hpp>


namespace TS_SfM {

  class System{
    public:
      System();
      ~System();
      bool loadConfigFile(const std::string str_config_file);
      bool loadCameraParams(const std::string str_camera_file);


    private:

  
  
  };

}// TS_SfM
