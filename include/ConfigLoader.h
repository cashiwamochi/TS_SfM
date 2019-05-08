#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace TS_SfM {

  struct SystemConfig {
    std::string str_path_to_images; 
  };

  struct Camera {
    float f_cx;
    float f_cy;
    float f_fx;
    float f_fy;
    float f_k1;
    float f_k2;
    float f_p1;
    float f_p2;
    float f_k3;
  };

  struct LoopConfig {
    int start;
    int end;
  };

  struct SfMConfig {
    int track_skip;
    int reconst_skip;
  };

  namespace ConfigLoader {
    std::pair<SystemConfig, Camera> LoadConfig(const std::string str_config_file);
    std::vector<std::string> ReadImagesInDir(const std::string& path_to_images);
    std::vector<cv::Mat> LoadImages(const std::vector<std::string>& vstr_image_names);
  }

}
