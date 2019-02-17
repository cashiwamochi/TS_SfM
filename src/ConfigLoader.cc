#include "ConfigLoader.h"

#include <algorithm>
#include "dirent.h"

using namespace TS_SfM;

std::pair<Config, Camera> ConfigLoader::LoadConfig(const std::string str_config_file) {
  Config config_params;
  Camera camera_params{-1.0,-1.0,-1.0,-1.0};
  cv::FileStorage fs_settings(str_config_file, cv::FileStorage::READ);
  if(!fs_settings.isOpened())
  {
    std::cerr << "[FAILED]: Cannot open " << str_config_file << std::endl;
    std::pair<Config, Camera> pair_config_cam_params = std::make_pair(config_params, camera_params);
    return pair_config_cam_params;
  }

  int skip = fs_settings["Config.skip"];
  config_params.n_skip = static_cast<unsigned int>(skip);
  config_params.str_path_to_images = static_cast<std::string>(fs_settings["Config.path2images"]);
  config_params.pair_loop_frames
    = std::make_pair(fs_settings["Config.loop_start"],fs_settings["Config.loop_end"]);

  camera_params.f_cx = fs_settings["Camera.cx"];
  camera_params.f_fx = fs_settings["Camera.fx"];
  camera_params.f_cy = fs_settings["Camera.cy"];
  camera_params.f_fy = fs_settings["Camera.fy"];


  std::pair<Config, Camera> pair_config_cam_params = std::make_pair(config_params, camera_params);
  return pair_config_cam_params;
}

std::vector<std::string> ConfigLoader::readImagesInDir(const std::string& str_path_to_images) {
  std::vector<std::string> vstr_image_names;
  DIR* dp=opendir(str_path_to_images.c_str());

  if (dp!=NULL)
  {
    struct dirent* d;
    do{
      d = readdir(dp);
      if (d!=NULL) {
        std::string file_name = d->d_name;
        if (file_name == "." or file_name == "..") continue;
        vstr_image_names.push_back(str_path_to_images +"/"+ file_name);
      }
    }while(d!=NULL);
  }
  closedir(dp);
  
  std::sort(vstr_image_names.begin(), vstr_image_names.end());
  // for(auto imagename : vstr_image_names)
    // std::cout << imagename << std::endl;
  return vstr_image_names;
}


std::vector<cv::Mat> ConfigLoader::loadImages(const std::vector<std::string>& vstr_image_names) {
  std::vector<cv::Mat> vm_images;
  vm_images.reserve((int)vstr_image_names.size());

  for(auto str_image_name : vstr_image_names) {
    cv::Mat image = cv::imread(str_image_name, 1);
    vm_images.push_back(image); 
  }

  return vm_images;
}
