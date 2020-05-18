#pragma once

#include "ConfigLoader.h"
#include <memory>
#include <vector>
#include <string>

namespace TS_SfM {

  class Frame;
  class KeyFrame;
  class KPExtractor;
  class Reconstructor;
  class Map;
  class MapPoint;

  class Matcher;

  class Viewer;

  class System{
    public:
      struct InitializerConfig {
        int num_frames;
        int connect_distance;
      };

    private:
      struct InitialReconstruction {
        std::vector<KeyFrame> v_keyframes;
        std::vector<MapPoint> v_mappoints;
      };

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
  
      std::unique_ptr<KPExtractor> m_p_extractor;

      // Those pointers are used globally in TS_SfM::System
      std::unique_ptr<Reconstructor> m_p_reconstructor;
      std::shared_ptr<Map> m_p_map; // this is referenced from viewer and constructor instert infomation.
      std::unique_ptr<Viewer> m_p_viewer;

      InitializerConfig m_initializer_config;

      void InitializeFrames(std::vector<Frame>& v_frames, const int num_frames_in_initial_map = 6);
      int InitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames);
      InitialReconstruction FlexibleInitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames);
      int IncrementalSfM(std::vector<KeyFrame>& v_keyframes, 
                         std::vector<MapPoint>& v_mappoints,
                         Frame& f,
                         const std::vector<std::vector<cv::DMatch>>& v_matches,
                         const InitializerConfig _config);

      void DrawEpiLines(const Frame& f0, const Frame& f1, 
                        const std::vector<cv::DMatch>& v_matches01, const std::vector<bool>& vb_mask,
                        const cv::Mat& F) const;
  };

} //TS_SfM
