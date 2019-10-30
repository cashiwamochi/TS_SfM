#pragma once

#include "ConfigLoader.h"
#include <memory>

namespace TS_SfM {

  class Frame;
  class KPExtractor;
  class Reconstructor;
  class Map;
  class MapPoint;

  class Matcher;

  class System{
    public:
      System(const std::string& str_config_file);
      ~System();
      void Run();

    private:
      void ShowConfig();
      const std::string m_config_file;
      SystemConfig m_config;
      Camera m_camera;
      unsigned int m_image_width, m_image_height;
      std::vector<std::string> m_vstr_image_names; 
      std::vector<cv::Mat> m_vm_images; 
      std::vector<Frame> m_v_frames; 
  
      std::shared_ptr<KPExtractor> m_p_extractor;

      // Those pointers are used globally in TS_SfM::System
      std::unique_ptr<Reconstructor> m_p_reconstructor;
      std::shared_ptr<Map> m_p_map;

      void InitializeFrames(std::vector<Frame>& v_frames, std::vector<cv::Mat>& vm_images,
                            const std::shared_ptr<KPExtractor>& p_extractor);
      int InitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames);
      int FlexibleInitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames);

      void DrawEpiLines(const Frame& f0, const Frame& f1, 
                        const std::vector<cv::DMatch>& v_matches01, const std::vector<bool>& vb_mask,
                        const cv::Mat& F) const;
  };

} //TS_SfM
