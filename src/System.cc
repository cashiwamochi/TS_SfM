#include "System.h"

#include "Frame.h"
#include "KPExtractor.h"

#include "Matcher.h"

#include "Mapper.h"
#include "Map.h"
#include "MapPoint.h"

#include <functional>

namespace TS_SfM {
  System::System(const std::string& str_config_file) : m_config_file(str_config_file) {
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

    // Matcher::MatcherConfig m_matcher_config = ConfigLoader::LoadMatcherConfig(str_config_file);  

    m_p_extractor.reset(new KPExtractor(m_image_width, m_image_height,
                        ConfigLoader::LoadExtractorConfig(str_config_file)));
    m_p_tracker.reset(new Tracker(ConfigLoader::LoadTrackerConfig(str_config_file)));
    m_p_mapper.reset(new Mapper(ConfigLoader::LoadMapperConfig(str_config_file)));

  }

  System::~System() {

  };

  void System::InitializeFrames(std::vector<Frame>& v_frames, std::vector<cv::Mat>& vm_images,
                                const std::shared_ptr<KPExtractor>& p_extractor)
  {
    for (size_t i = 0; i < vm_images.size(); i++) {
      Frame frame(i, vm_images[i], p_extractor); 
      v_frames.push_back(frame);
    }
    std::cout << " Done. " << std::endl;

    std::cout << "[LOG] "
              << "SfM pipeline starts ...";

#if 0
    for (size_t i = 0; i < v_frames.size(); i++) {
      // v_frames[i].ShowFeaturePoints();    
      v_frames[i].ShowFeaturePointsInGrids();    
    }
#endif

    std::cout << "[LOG] "
              << "Extracting Feature points ...";
    std::cout << " Done. " << std::endl;

    return;
  }

  // Initialization is done in 3-view geometry
  int System::InitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames) {
    int num_map_points = -1; 
    assert(v_frames.size() == 3);  

    Frame& frame_1st = v_frames[0].get();
    Frame& frame_2nd = v_frames[1].get();
    Frame& frame_3rd = v_frames[2].get();

    Matcher matcher(ConfigLoader::LoadMatcherConfig(m_config_file));

    std::vector<cv::DMatch> v_matches_12 = matcher.GetMatches(frame_1st,frame_2nd);
    std::vector<cv::DMatch> v_matches_13 = matcher.GetMatches(frame_1st,frame_3rd);
    std::vector<cv::DMatch> v_matches_23 = matcher.GetMatches(frame_2nd,frame_3rd);

    return num_map_points;
  }

  void System::Run() {
    std::cout << "[LOG] " 
              << "Start Processing ..."
              << std::endl;

    InitializeFrames(m_v_frames, m_vm_images, m_p_extractor);

    std::vector<std::reference_wrapper<Frame>> 
      v_ini_frames{m_v_frames[0], m_v_frames[1],m_v_frames[2]};
    InitializeGlobalMap(v_ini_frames);

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

