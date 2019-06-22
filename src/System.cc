#include "System.h"

#include "Frame.h"
#include "KPExtractor.h"

#include "Matcher.h"
#include "Solver.h"

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

  void System::DrawEpiLines(const Frame& f0, const Frame& f1, 
                            const std::vector<cv::DMatch>& v_matches01, const std::vector<bool>& vb_mask,
                            const cv::Mat& F) const
  {
    cv::Mat output = f1.GetImage().clone();
    int max_line_num = 10;
    int line_num = 0;
    std::vector<cv::KeyPoint> vkpts0 = f0.GetKeyPoints();
    std::vector<cv::KeyPoint> vkpts1 = f1.GetKeyPoints();
    for(int i = 0; i < vb_mask.size(); ++i) {
      if(vb_mask[i] && line_num < max_line_num) {
        cv::Mat pt0 = (cv::Mat_<float>(3,1) << vkpts0[i].pt.x, vkpts0[i].pt.y, 1.0);
        cv::Mat l = F * cv::Mat(pt0);
        cv::Vec3f line(l.reshape(3).at<cv::Vec3f>());

        cv::line(output,
                 cv::Point2f(0, -line(2)/line(1)),
                 cv::Point2f((float)output.cols-1.0, (-line(2)-line(0)*(output.cols-1))/line(1) ), 
                 cv::Scalar(0,255,0), 3);
      
        line_num++;
      } 
    }

    cv::imshow("epipolar-line", output);
    cv::waitKey();


    return;
  }

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

    // Compute Fundamental Matrix
    cv::Mat mK = (cv::Mat_<float>(3,3) << m_camera.f_fx, 0.0, m_camera.f_cx,
                                          0.0, m_camera.f_fy, m_camera.f_cy,
                                          0.0,           0.0,           1.0);

    cv::Mat mE, mF;
    std::vector<bool> vb_mask;
    int score;
    Solver::SolveEpipolarConstraintRANSAC(mK, 
                                          std::make_pair(frame_1st.GetKeyPoints(),frame_2nd.GetKeyPoints()),
                                          v_matches_12, mF, vb_mask, score);

    std::cout << "Score = " << score
              << " / " << v_matches_12.size() <<  std::endl;

    DrawEpiLines(frame_1st, frame_2nd, v_matches_12, vb_mask, mF);

    // decompose E

    // Triangulation



    // Bundle Adjustment

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

