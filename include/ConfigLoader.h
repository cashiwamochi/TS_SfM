#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace TS_SfM {

  struct Config {
    std::string str_path_to_images; 
    std::pair<int, int> pair_loop_frames;
    unsigned int n_skip; 
  };

  struct Camera {
    float f_cx;
    float f_cy;
    float f_fx;
    float f_fy;
  };

  namespace ConfigLoader {
    std::pair<Config, Camera> LoadConfig(const std::string str_config_file);
    std::vector<std::string> readImagesInDir(const std::string& path_to_images);
    std::vector<cv::Mat> loadImages(const std::vector<std::string>& vstr_image_names);
  }

}
