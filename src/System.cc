#include "System.h"
#include "Frame.h"
#include "KPExtractor.h"

namespace TS_SfM {
  System::System(const std::string& str_config_file) {
    std::pair<SystemConfig, Camera> _pair_config = ConfigLoader::LoadConfig(str_config_file);  
    m_config = _pair_config.first;
    m_camera = _pair_config.second;
    
    m_vstr_image_names = ConfigLoader::ReadImagesInDir(m_config.str_path_to_images);
    m_vm_images = ConfigLoader::LoadImages(m_vstr_image_names);

    if (m_camera.f_cx < 1.0) {
      m_camera.f_cx *= (float)m_vm_images[0].rows; 
    }
    if (m_camera.f_cy < 1.0) {
      m_camera.f_cy *= (float)m_vm_images[0].cols; 
    }

    m_image_width = m_vm_images[0].cols;
    m_image_height = m_vm_images[0].rows;

    ShowConfig();
    m_v_frames.reserve((int)m_vm_images.size()); 
    // m_v_frames = std::vector<Frame>((int)m_vm_images.size(), Frame()); 

    m_p_extractor.reset(new KPExtractor(m_image_width, m_image_height,
                        ConfigLoader::LoadExtractorConfig(str_config_file)));
    m_p_tracker.reset(new Tracker(ConfigLoader::LoadTrackerConfig(str_config_file)));
    m_p_mapper.reset(new Mapper(ConfigLoader::LoadMapperConfig(str_config_file)));

  }

  System::~System() {

  };

  void System::Run() {
    std::cout << "[LOG] " 
              << "Start Processing ..."
              << std::endl;

    std::cout << "[LOG] "
              << "Extracting Feature points ...";
    for (size_t i = 0; i < m_vm_images.size(); i++) {
      Frame frame(i, m_vm_images[i], m_p_extractor); 
      m_v_frames.push_back(frame);
    }
    std::cout << " Done. " << std::endl;

    std::cout << "[LOG] "
              << "SfM pipeline starts ...";

    for (size_t i = 0; i < m_v_frames.size(); i++) {
      m_v_frames[i].ShowFeaturePoints();    
    }

    std::cout << " Done. " << std::endl;


    std::cout << std::endl;
    return;
  };

  void System::ShowConfig() {
    std::cout << "[Config.path2images] "
              << m_config.str_path_to_images
              << std::endl;

    std::cout << "[Images] " 
              << m_vm_images.size() 
              << std::endl;
    
    return;
  }

}

