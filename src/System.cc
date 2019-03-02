#include "System.h"
#include "Frame.h"
#include "KPExtractor.h"

namespace TS_SfM {
  System::System(const std::string& str_config_file) {
    std::pair<Config, Camera> _pair_config = ConfigLoader::LoadConfig(str_config_file);  
    m_config = _pair_config.first;
    m_camera = _pair_config.second;
    
    m_vstr_image_names = ConfigLoader::readImagesInDir(m_config.str_path_to_images);
    m_vm_images = ConfigLoader::loadImages(m_vstr_image_names);

    if (m_camera.f_cx < 1.0) {
      m_camera.f_cx *= (float)m_vm_images[0].rows; 
    }
    if (m_camera.f_cy < 1.0) {
      m_camera.f_cy *= (float)m_vm_images[0].cols; 
    }

    m_image_width = m_vm_images[0].cols;
    m_image_height = m_vm_images[0].rows;

    showConfig();
    m_v_frames = std::vector<Frame>((int)m_vm_images.size()); 

    // m_p_extractor.reset(new KPExtractor{m_image_width, m_image_height});
  }

  System::~System() {
  };

  void System::run() {




    return;
  };

  void System::showConfig() {
    std::cout << "[Config.path2images] "
              << m_config.str_path_to_images
              << std::endl
              << "[Config.n_skip] "
              << m_config.n_skip
              << std::endl
              << "[Config.loop_pair] "
              << m_config.pair_loop_frames.first << ":" 
              << m_config.pair_loop_frames.second
              << std::endl;
    
    std::cout << "[Camera.fx] "
              << m_camera.f_fx
              << std::endl
              << "[Camera.cx] "
              << m_camera.f_cx
              << std::endl
              << "[Camera.fy] "
              << m_camera.f_fy
              << std::endl
              << "[Camera.cy] "
              << m_camera.f_cy
              << std::endl;

    return;
  }

}

