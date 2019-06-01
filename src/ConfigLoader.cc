#include "ConfigLoader.h"

#include <iostream>
#include <algorithm>
#include "dirent.h"

using namespace TS_SfM;

std::pair<SystemConfig, Camera> ConfigLoader::LoadConfig(const std::string str_config_file) {
  SystemConfig config_params;
  Camera camera_params{-1.0,-1.0,-1.0,-1.0};
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  if(!fs_settings.isOpened())
  {
    std::cerr << "[FAILED]: Cannot open " << str_config_file << std::endl;
    std::pair<SystemConfig, Camera> pair_config_cam_params = std::make_pair(config_params, camera_params);
    return pair_config_cam_params;
  }

  config_params.str_path_to_images = static_cast<std::string>(fs_settings["Config.path2images"]);

  camera_params.f_cx = fs_settings["Camera.cx"];
  camera_params.f_fx = fs_settings["Camera.fx"];
  camera_params.f_cy = fs_settings["Camera.cy"];
  camera_params.f_fy = fs_settings["Camera.fy"];

  camera_params.f_k1 = fs_settings["Camera.k1"];
  camera_params.f_k2 = fs_settings["Camera.k2"];
  camera_params.f_p1 = fs_settings["Camera.p1"];
  camera_params.f_p2 = fs_settings["Camera.p2"];
  camera_params.f_k3 = fs_settings["Camera.k3"];

  std::pair<SystemConfig, Camera> pair_config_cam_params = std::make_pair(config_params, camera_params);
  return pair_config_cam_params;
}

std::vector<std::string> ConfigLoader::ReadImagesInDir(const std::string& str_path_to_images) {
  std::vector<std::string> vstr_image_names;
  DIR* dp=opendir(str_path_to_images.c_str());

  if (dp!=nullptr)
  {
    struct dirent* d;
    do{
      d = readdir(dp);
      if (d!=nullptr) {
        std::string file_name = d->d_name;
        if (file_name == "." or file_name == "..") continue;
        vstr_image_names.push_back(str_path_to_images +"/"+ file_name);
      }
    }while(d!=nullptr);
  }
  closedir(dp);
  
  std::sort(vstr_image_names.begin(), vstr_image_names.end());
  return vstr_image_names;
}


std::vector<cv::Mat> ConfigLoader::LoadImages(const std::vector<std::string>& vstr_image_names) {
  std::vector<cv::Mat> vm_images;
  vm_images.reserve((int)vstr_image_names.size());

  for(auto str_image_name : vstr_image_names) {
    cv::Mat image = cv::imread(str_image_name, 1);
    vm_images.push_back(image); 
  }

  return vm_images;
}

Tracker::TrackerConfig ConfigLoader::LoadTrackerConfig(const std::string str_config_file) {
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  Tracker::TrackerConfig tracker_config;
  tracker_config.skip = fs_settings["Tracker.skip"];

  return tracker_config;
}

Mapper::MapperConfig ConfigLoader::LoadMapperConfig(const std::string str_config_file) {
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  Mapper::MapperConfig mapper_config;
  mapper_config.skip = fs_settings["MapperConfig.skip"];

  return mapper_config;
}

LoopClosure::LoopConfig ConfigLoader::LoadLoopConfig(const std::string str_config_file) {
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  LoopClosure::LoopConfig lc_config;
  lc_config.start_id = fs_settings["LoopConfig.start"];
  lc_config.end_id = fs_settings["LoopConfig.end"];

  return lc_config;
}

KPExtractor::ExtractorConfig ConfigLoader::LoadExtractorConfig(const std::string str_config_file) {
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  KPExtractor::ExtractorConfig extractor_config;

  extractor_config.str_descriptor = static_cast<std::string>(fs_settings["Extractor.descriptor"]);
  extractor_config.threshold = static_cast<float>(fs_settings["Extractor.threshold"]);
  extractor_config.octaves = static_cast<int>(fs_settings["Extractor.octaves"]);
  extractor_config.octavelayers = static_cast<int>(fs_settings["Extractor.octavelayers"]);
  extractor_config.grid_width = static_cast<int>(fs_settings["Extractor.grid_width"]);
  extractor_config.grid_height = static_cast<int>(fs_settings["Extractor.grid_height"]);
  extractor_config.num_in_grid = static_cast<int>(fs_settings["Extractor.num_in_grid"]);

  return extractor_config;
}
