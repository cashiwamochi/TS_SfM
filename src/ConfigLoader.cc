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

  // config_params.pair_loop_frames
  //   = std::make_pair(fs_settings["Config.loop_start"],fs_settings["Config.loop_end"]);

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

SfM::SfMConfig LoadSfMConfig(const std::string str_config_file) {
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  SfM::SfMConfig sfm_config;
  sfm_config.track_skip = fs_settings["SfMConfig.track_skip"];
  sfm_config.reconst_skip = fs_settings["SfMConfig.reconst_skip"];

  return sfm_config;
}

LoopClosure::LoopConfig LoadLoopConfig(const std::string str_config_file) {
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  LoopClosure::LoopConfig lc_config;
  lc_config.start_id = fs_settings["LoopConfig.start"];
  lc_config.end_id = fs_settings["LoopConfig.end"];

  return lc_config;
}
